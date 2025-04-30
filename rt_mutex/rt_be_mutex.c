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

static DEFINE_RT_MUTEX(test_rt_mutex);


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
    bool is_boosted;
    struct list_head wait_queue;  // Single queue instead of rt_queue & be_queue

    //struct list_head rt_queue;
    //struct list_head be_queue;
};

struct task_node
{
    struct task_struct *task;
    struct list_head list;
};

static struct rt_be_mutex my_mutex;


static void boost_task(struct task_struct *task)
{
    // Only boost if this is a deadline task
    if (task->policy == SCHED_DEADLINE) {
        task_lock(task);
        // Direct access to dl_boosted field
        task->dl.dl_non_preemptible = 1;
        printk(KERN_INFO "rt_be_mutex: Boosting task %d\n", task->pid);
        task_unlock(task);
    }
}

static void unboost_task(struct task_struct *task)
{
    if (task->policy == SCHED_DEADLINE) {
        task_lock(task);
        task->dl.dl_non_preemptible = 0;
        printk(KERN_INFO "rt_be_mutex: Unboosting task %d\n", task->pid);
        task_unlock(task);
    }
}

static void init_rt_be_mutex(struct rt_be_mutex *mutex)
{
    mutex->owner = -1;
    INIT_LIST_HEAD(&mutex->wait_queue);
}

static int rt_be_mutex_lock(struct rt_be_mutex *mutex, bool is_rt)
{
    struct task_node *node;

    down(&sem);

    if (mutex->owner == -1)
    {
        mutex->owner = current->pid;

        if (!mutex->is_boosted)
        {
            boost_task(current);
            mutex->is_boosted = true;
        }

        printk(KERN_INFO "rt_be_mutex: Thread %d acquired lock.\n", current->pid);
        up(&sem);
        return 0;
    }

    // Lock is busy, enqueue in single FIFO queue
    node = kmalloc(sizeof(*node), GFP_KERNEL);
    if (!node)
    {
        up(&sem);
        return -ENOMEM;
    }
    node->task = current;
    
    // Add to single wait queue
    list_add_tail(&node->list, &mutex->wait_queue);

    printk(KERN_INFO "rt_be_mutex: Thread %d queued for lock\n", current->pid);

    set_current_state(TASK_UNINTERRUPTIBLE);
    up(&sem);

    schedule();

    return 0;
}

static int rt_be_mutex_unlock(struct rt_be_mutex *mutex)
{
    struct task_node *next_task;

    down(&sem);

    if (mutex->owner != current->pid)
    {
        up(&sem);
        return -EPERM;
    }

    mutex->owner = -1;

    if (mutex->is_boosted)
    {
        unboost_task(current);
        mutex->is_boosted = false;
    }

    printk(KERN_INFO "rt_be_mutex: Thread %d released lock.\n", current->pid);

    // Check single wait queue
    if (!list_empty(&mutex->wait_queue))
    {
        next_task = list_first_entry(&mutex->wait_queue, struct task_node, list);
        list_del(&next_task->list);
        mutex->owner = next_task->task->pid;
        boost_task(next_task->task);
        mutex->is_boosted = true;
        wake_up_process(next_task->task);

        kfree(next_task);
    }
    else{
        printk(KERN_INFO "rt_be_mutex: No waiters in queue.\n");
    }
    
    up(&sem);
    return 0;
}
/*
static int rt_be_mutex_lock(struct rt_be_mutex *mutex, bool is_rt)
{
    struct task_node *node;

    // Protect queue operations with a semaphore
    down(&sem);

    if (mutex->owner == -1)
    {
        mutex->owner = current->pid;

        // Boost the task instead of disabling preemption
        if (!mutex->is_boosted)
        {
            boost_task(current);
            mutex->is_boosted = true;
        }

        printk(KERN_INFO "rt_be_mutex: Thread %d acquired lock.\n", current->pid);
        up(&sem);
        return 0;
    }

    // Lock is busy, enqueue
    node = kmalloc(sizeof(*node), GFP_KERNEL);
    if (!node)
    {
        up(&sem);
        return -ENOMEM;
    }
    node->task = current;

    if (is_rt)
    {
        list_add_tail(&node->list, &mutex->rt_queue);
    }
    else
    {
        list_add_tail(&node->list, &mutex->be_queue);
    }

    printk(KERN_INFO "rt_be_mutex: Thread %d queued for lock\n", current->pid);
    up(&sem);

    // Sleep until woken in unlock
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule();

    return 0;
}

static int rt_be_mutex_unlock(struct rt_be_mutex *mutex)
{
    struct task_node *next_task;
    struct list_head *queue;

    down(&sem);

    if (mutex->owner != current->pid)
    {
        up(&sem);
        return -EPERM;
    }

    mutex->owner = -1;

    // Unboost the task instead of enabling preemption
    if (mutex->is_boosted)
    {
        unboost_task(current);
        mutex->is_boosted = false;
    }

    printk(KERN_INFO "rt_be_mutex: Thread %d released lock.\n", current->pid);


    // Check RT queue first, then BE queue
    if (!list_empty(&mutex->rt_queue))
    {
        queue = &mutex->rt_queue;
    }
    else if (!list_empty(&mutex->be_queue))
    {
        queue = &mutex->be_queue;
    }
    else
    {
        // No waiters
        up(&sem);
        return 0;
    }

    next_task = list_first_entry(queue, struct task_node, list);
    list_del(&next_task->list);

    // The new owner
    mutex->owner = next_task->task->pid;
    wake_up_process(next_task->task);

    kfree(next_task);
    up(&sem);

    return 0;
}
*/
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
static int dev_release(struct inode *inode, struct file *file)
{
    struct task_node *node;
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
        node = list_entry(pos, struct task_node, list);
        if (node->task->tgid == current->tgid)
        {
            list_del(pos);
            wake_up_process(node->task);
            kfree(node);
        }
    }

    up(&sem);
    return 0;
}

/*
static int dev_release(struct inode *inode, struct file *file)
{
    struct task_node *node, *tmp;
    struct list_head *pos, *n;

    down(&sem);

    // If closing process owns the lock, unlock normally
    if (my_mutex.owner == current->pid)
    {
        printk(KERN_INFO "rt_be_mutex: Process %d closed FD while owning lock, forcing unlock.\n",
               current->pid);
        rt_be_mutex_unlock(&my_mutex);
    }
    //down(&sem);
    // Remove leftover queue entries from this process
    list_for_each_safe(pos, n, &my_mutex.rt_queue)
    {
        node = list_entry(pos, struct task_node, list);
        if (node->task->tgid == current->tgid)
        {
            list_del(pos);
            wake_up_process(node->task);
            kfree(node);
        }
    }
    list_for_each_safe(pos, n, &my_mutex.be_queue)
    {
        node = list_entry(pos, struct task_node, list);
        if (node->task->tgid == current->tgid)
        {
            list_del(pos);
            wake_up_process(node->task);
            kfree(node);
        }
    }

    up(&sem);
    return 0;
}
*/
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
    struct task_node *node;

    down(&sem);
    // Clear the single wait queue
    list_for_each_safe(pos, n, &my_mutex.wait_queue)
    {
        node = list_entry(pos, struct task_node, list);
        list_del(pos);
        wake_up_process(node->task);
        kfree(node);
    }
    
    // Force unlock if still owned
    if (my_mutex.owner != -1)
    {
        printk(KERN_WARNING "Force-unlocking. Owner=%d\n", my_mutex.owner);
        rt_be_mutex_unlock(&my_mutex);
    }
    up(&sem);
}
// static void cleanup_all_waiters(void)
// {
//     struct list_head *pos, *n;
//     struct task_node *node;

//     down(&sem);
//     // Clear RT queue
//     list_for_each_safe(pos, n, &my_mutex.rt_queue)
//     {
//         node = list_entry(pos, struct task_node, list);
//         list_del(pos);
//         wake_up_process(node->task);
//         kfree(node);
//     }
//     // Clear BE queue
//     list_for_each_safe(pos, n, &my_mutex.be_queue)
//     {
//         node = list_entry(pos, struct task_node, list);
//         list_del(pos);
//         wake_up_process(node->task);
//         kfree(node);
//     }
//     // Force unlock if still owned
//     if (my_mutex.owner != -1)
//     {
//         printk(KERN_WARNING "Force-unlocking. Owner=%d\n", my_mutex.owner);
//         rt_be_mutex_unlock(&my_mutex);
//     }
//     up(&sem);
// }

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
