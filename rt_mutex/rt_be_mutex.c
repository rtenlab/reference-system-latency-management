#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/preempt.h>
#include <linux/irqflags.h>
#include <linux/rtmutex.h>
#include <linux/sched/deadline.h>
#include <linux/tracepoint.h>
#include <linux/module.h>

#define DEVICE_NAME "rt_be_mutex"
#define IOCTL_LOCK _IOW('r', 1, int)
#define IOCTL_UNLOCK _IOW('r', 2, int)

#define TASK_TYPE_RT 1
#define TASK_TYPE_BE 2

static dev_t dev_number;
static struct class *dev_class;
static struct cdev rt_be_cdev;
static struct semaphore sem;

struct rt_be_mutex
{
    pid_t owner;
    struct list_head wait_queue; // Single queue instead of rt_queue & be_queue
};

static struct rt_be_mutex my_mutex;

static void boost_task(struct task_struct *task)
{
    // Only boost if this is a deadline task
    if (task->policy == SCHED_DEADLINE)
    {
        task_lock(task);
        // Direct access to dl_boosted field
        task->dl.dl_non_preemptible = 1;
        printk(KERN_INFO "rt_be_mutex: Boosting task %d\n", task->pid);
        task_unlock(task);
    }
    else
    {
        printk(KERN_INFO "rt_be_mutex: Task %d is not a deadline task, no boost applied.\n", task->pid);
    }
}

static void unboost_task(struct task_struct *task)
{
    if (task->policy == SCHED_DEADLINE)
    {
        task_lock(task);
        task->dl.dl_non_preemptible = 0;
        printk(KERN_INFO "rt_be_mutex: Unboosting task %d\n", task->pid);
        task_unlock(task);
        // set_tsk_need_resched(task);
    }
}

static void init_rt_be_mutex(struct rt_be_mutex *mutex)
{
    mutex->owner = -1;
    INIT_LIST_HEAD(&mutex->wait_queue);
}

static int rt_be_mutex_lock(struct rt_be_mutex *mutex, bool is_rt)
{
    down(&sem);

    if (mutex->owner == -1)
    {
        mutex->owner = current->pid;
        boost_task(current);
        // printk(KERN_INFO "rt_be_mutex: Thread %d acquired lock.\n", current->pid);
        up(&sem);
        return 0;
    }

    // Lock is busy, add current task directly to wait queue
    // No need for kmalloc anymore!
    get_task_struct(current); // Still need this reference count

    // Add to wait queue using the embedded list_head
    list_add_tail(&current->rt_be_mutex_list, &mutex->wait_queue);

    printk(KERN_INFO "rt_be_mutex: Thread %d queued for lock\n", current->pid);

    set_current_state(TASK_UNINTERRUPTIBLE);
    smp_mb();
    up(&sem);
    schedule();
    set_current_state(TASK_RUNNING);
    return 0;
}

// Modify rt_be_mutex_unlock function
static int rt_be_mutex_unlock(struct rt_be_mutex *mutex)
{
    struct task_struct *next_task;
    int current_cpu;
    unsigned long flags;

    down(&sem);

    if (mutex->owner != current->pid)
    {
        up(&sem);
        return -EPERM;
    }

    mutex->owner = -1;

    printk(KERN_INFO "rt_be_mutex: Thread %d released lock on CPU %d.\n",
           current->pid, current_cpu);
retry:
    if (!list_empty(&mutex->wait_queue))
    {
        // Get the task directly from the list_head in the task_struct
        next_task = list_first_entry(&mutex->wait_queue, struct task_struct, rt_be_mutex_list);

        // Check if task is dead or dying
        if (!refcount_inc_not_zero(&next_task->usage) ||
            !pid_task(find_vpid(next_task->pid), PIDTYPE_PID) ||
            (next_task->flags & PF_EXITING) ||
            (next_task->state == TASK_DEAD) ||
            (next_task->exit_state != 0))
        {
            // Task is invalid, remove from queue
            printk(KERN_WARNING "rt_be_mutex: Waiter task %d is dead/exiting\n", next_task->pid);
            list_del(&next_task->rt_be_mutex_list);
            put_task_struct(next_task); // Balance get_task_struct from enqueue
            goto retry;
        }

        // We got a reference for validation, but don't need it now
        put_task_struct(next_task);

        // Remove from wait queue and set as owner
        list_del(&next_task->rt_be_mutex_list);
        mutex->owner = next_task->pid;

        preempt_disable();
        local_irq_save(flags);
        
        // Boost next task before unboosting current
        boost_task(next_task);
        unboost_task(current);
        
        // Wake up next task
        wake_up_process(next_task);
        
        // Release semaphore inside preempt-disabled section
        up(&sem);
        
        // Re-enable preemption and IRQs
        local_irq_restore(flags);
        preempt_enable();

        put_task_struct(next_task);
        return 0;
    }
    else
    {
        preempt_disable();
        local_irq_save(flags);
        unboost_task(current);
        local_irq_restore(flags);
        preempt_enable();
        set_tsk_need_resched(current);
        printk(KERN_INFO "rt_be_mutex: No waiters in queue.\n");

        up(&sem);
        return 0;
    }
}
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int task_type;

    if (!dev_class)
    {
        printk(KERN_ALERT "rt_be_mutex: Device class destroyed, rejecting IOCTL\n");
        return -ENODEV;
    }

    switch (cmd)
    {
    case IOCTL_LOCK:
        if (copy_from_user(&task_type, (int __user *)arg, sizeof(task_type)))
            return -EFAULT;
        if (task_type == TASK_TYPE_RT)
            return rt_be_mutex_lock(&my_mutex, true);
        else if (task_type == TASK_TYPE_BE)
            return rt_be_mutex_lock(&my_mutex, false);
        else
            return -EINVAL;

    case IOCTL_UNLOCK:
        return rt_be_mutex_unlock(&my_mutex);

    default:
        return -EINVAL;
    }
}
// Update dev_release to clean up any waiters from this process
static int dev_release(struct inode *inode, struct file *file)
{
    struct task_struct *task;
    struct list_head *pos, *n;

    down(&sem);

    // If closing process owns the lock, unlock normally
    if (my_mutex.owner == current->pid)
    {
        printk(KERN_INFO "rt_be_mutex: Process %d closed FD while owning lock, forcing unlock.\n",
               current->pid);
        rt_be_mutex_unlock(&my_mutex);
    }

    // Remove leftover queue entries from this process
    list_for_each_safe(pos, n, &my_mutex.wait_queue)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        if (task->tgid == current->tgid)
        {
            list_del(pos);
            wake_up_process(task);
            put_task_struct(task); // Balance get_task_struct from enqueue
        }
    }

    up(&sem);
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = dev_ioctl,
    .release = dev_release,
};

static int __init rt_be_mutex_init(void)
{
    int result;

    sema_init(&sem, 1);
    init_rt_be_mutex(&my_mutex);

    result = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
    if (result < 0)
    {
        printk(KERN_ALERT "rt_be_mutex: Failed to allocate a major number\n");
        return result;
    }

    cdev_init(&rt_be_cdev, &fops);
    rt_be_cdev.owner = THIS_MODULE;
    result = cdev_add(&rt_be_cdev, dev_number, 1);
    if (result < 0)
    {
        unregister_chrdev_region(dev_number, 1);
        printk(KERN_ALERT "rt_be_mutex: Failed to add cdev\n");
        return result;
    }

    dev_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(dev_class))
    {
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        printk(KERN_ALERT "rt_be_mutex: Failed to create class\n");
        return PTR_ERR(dev_class);
    }

    if (!device_create(dev_class, NULL, dev_number, NULL, DEVICE_NAME))
    {
        class_destroy(dev_class);
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        printk(KERN_ALERT "rt_be_mutex: Failed to create device\n");
        return -1;
    }

    printk(KERN_INFO "rt_be_mutex: Module loaded, major=%d\n", MAJOR(dev_number));
    return 0;
}

static void cleanup_all_waiters(void)
{
    struct list_head *pos, *n;
    struct task_struct *task;

    down(&sem);
    // Clear the wait queue
    list_for_each_safe(pos, n, &my_mutex.wait_queue)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        list_del(pos);
        wake_up_process(task);
        put_task_struct(task); // Balance get_task_struct from enqueue
    }

    // Force unlock if still owned
    if (my_mutex.owner != -1)
    {
        printk(KERN_WARNING "Force-unlocking. Owner=%d\n", my_mutex.owner);
        my_mutex.owner = -1; // Direct unlock to avoid recursion
    }
    up(&sem);
}

static void __exit rt_be_mutex_exit(void)
{
    cleanup_all_waiters();
    device_destroy(dev_class, dev_number);
    class_destroy(dev_class);
    cdev_del(&rt_be_cdev);
    unregister_chrdev_region(dev_number, 1);
    printk(KERN_INFO "rt_be_mutex: Module unloaded\n");
}

EXPORT_SYMBOL(rt_be_mutex_lock);
EXPORT_SYMBOL(rt_be_mutex_unlock);
module_init(rt_be_mutex_init);
module_exit(rt_be_mutex_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Enright");
MODULE_DESCRIPTION("RT-BE Mutex Module for LaME ROS 2 Executor.");
