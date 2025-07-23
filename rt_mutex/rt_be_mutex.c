// rt_deferred_mutex.c
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/sched/deadline.h>
#include <linux/sched/task.h>
#include <linux/pid.h>
#include <linux/ktime.h>

#define DEVICE_NAME "rt_be_mutex"                 // Device name in /dev
#define IOCTL_LOCK _IOW('r', 1, struct lock_args) // Lock ioctl (write struct)
#define IOCTL_UNLOCK _IOW('r', 2, int)            // Unlock ioctl (dummy int)

#define TASK_TYPE_RT 1 // Real-time (DL) type
#define TASK_TYPE_BE 2 // Best-effort type (unused)

struct lock_args
{                         // Arg struct for IOCTL_LOCK
    int task_type;        // RT or BE
    bool account_overrun; // Enable overrun debt
};

static dev_t dev_number;        // Allocated dev number
static struct class *dev_class; // Sysfs class
static struct cdev rt_be_cdev;  // Char dev struct

struct deferred_waiter
{                              // Entry for throttled deferred tasks
    struct list_head list;     // Node for deferred_queue
    struct task_struct *task;  // Throttled task
    struct hrtimer timer;      // Timer to replenishment
    ktime_t fire_time;         // Fire time for sorting
    struct rt_deferred_mutex *mutex; // Mutex backptr
};

struct rt_deferred_mutex
{                                    // Mutex state
    pid_t owner;                     // Owner PID (-1 free)
    struct list_head wait_queue;     // FIFO for non-throttled
    struct list_head deferred_queue; // Sorted for throttled
    struct list_head ready_deferred; // Fired timers (to front)
    spinlock_t lock;                 // Protects all above
};

static struct rt_deferred_mutex my_mutex; // Single global mutex

static void boost_task(struct task_struct *task, bool account_overrun)
{
    if (!task || task->policy != SCHED_DEADLINE)
        return;

    task->dl.dl_non_preemptible = 1;
    if (account_overrun)
        task->dl.dl_account_overrun = 1;

    //trace_rt_deferred_mutex_boost(task->pid, task->dl.dl_non_preemptible, task->dl.dl_account_overrun);
}

static void unboost_task(struct task_struct *task)
{
    if (!task || task->policy != SCHED_DEADLINE)
        return;

    task->dl.dl_non_preemptible = 0;
    task->dl.dl_account_overrun = 0;

    //trace_rt_deferred_mutex_unboost(task->pid);
}

static void process_ready_deferred(struct rt_deferred_mutex *mutex)
{ // Grant to ready deferred if free
    struct task_struct *next_task;

    if (mutex->owner != -1 || list_empty(&mutex->ready_deferred))
        return;

    list_splice_init(&mutex->ready_deferred, &mutex->wait_queue); 

    if (list_empty(&mutex->wait_queue))
        return;

retry:
    next_task = list_first_entry(&mutex->wait_queue, struct task_struct, rt_be_mutex_list);

    if (!pid_task(find_vpid(next_task->pid), PIDTYPE_PID) ||
        (next_task->flags & PF_EXITING) ||
        (next_task->state == TASK_DEAD) ||
        (next_task->exit_state != 0))
    {
        pr_warn_ratelimited("rt_deferred_mutex: Ready deferred task %d is dead/exiting\n", next_task->pid);
        list_del(&next_task->rt_be_mutex_list);
        put_task_struct(next_task);
        if (!list_empty(&mutex->wait_queue))
            goto retry;
        return;
    }

    list_del(&next_task->rt_be_mutex_list);
    mutex->owner = next_task->pid;
    boost_task(next_task, true); // TODO: Handle overrun toggle using account_overrun arg 
    //trace_rt_deferred_mutex_acquire(next_task->pid, mutex->owner, next_task->policy == SCHED_DEADLINE, false);
    wake_up_process(next_task);
    put_task_struct(next_task); 
}

static enum hrtimer_restart deferred_timer_cb(struct hrtimer *timer)
{ // Timer fires: move to ready
    struct deferred_waiter *dw = container_of(timer, struct deferred_waiter, timer);
    struct rt_deferred_mutex *mutex = dw->mutex;
    struct task_struct *task = dw->task; 
    unsigned long flags;

    spin_lock_irqsave(&mutex->lock, flags);
    list_del(&dw->list);
    kfree(dw);
    list_add_tail(&task->rt_be_mutex_list, &mutex->ready_deferred);
    //trace_rt_deferred_mutex_deferred_fire(task->pid);
    process_ready_deferred(mutex);
    spin_unlock_irqrestore(&mutex->lock, flags);

    return HRTIMER_NORESTART;
}

static void init_rt_deferred_mutex(struct rt_deferred_mutex *mutex)
{ // Setup mutex
    mutex->owner = -1;
    INIT_LIST_HEAD(&mutex->wait_queue);
    INIT_LIST_HEAD(&mutex->deferred_queue);
    INIT_LIST_HEAD(&mutex->ready_deferred);
    spin_lock_init(&mutex->lock);
}

static int rt_deferred_mutex_lock(struct rt_deferred_mutex *mutex, bool is_rt, bool account_overrun)
{ // Acquire loop
    unsigned long flags;
    struct deferred_waiter *dw = NULL;
    struct list_head *pos;

    spin_lock_irqsave(&mutex->lock, flags);

    if (mutex->owner == -1) {
        mutex->owner = current->pid;
        boost_task(current, account_overrun);
        //trace_rt_deferred_mutex_acquire(current->pid, mutex->owner, is_rt, account_overrun);
        spin_unlock_irqrestore(&mutex->lock, flags);
        return 0;
    }

    if (mutex->owner == current->pid) {
        boost_task(current, account_overrun);
        //trace_rt_deferred_mutex_acquire(current->pid, mutex->owner, is_rt, account_overrun);
        spin_unlock_irqrestore(&mutex->lock, flags);
        return 0;
    }

    if (current->policy == SCHED_DEADLINE && current->dl.dl_throttled) {
        ktime_t fire_time = ns_to_ktime(current->dl.deadline);

        dw = kmalloc(sizeof(*dw), GFP_ATOMIC);
        if (!dw) {
            spin_unlock_irqrestore(&mutex->lock, flags);
            return -ENOMEM;
        }

        get_task_struct(current); // Reference for deferred waiter
        dw->task = current;
        dw->mutex = mutex;
        dw->fire_time = fire_time;

        hrtimer_init(&dw->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
        dw->timer.function = deferred_timer_cb;
        hrtimer_start(&dw->timer, dw->fire_time, HRTIMER_MODE_ABS);

        pos = &mutex->deferred_queue;
        while (pos->next != &mutex->deferred_queue) {
            struct deferred_waiter *tmp = list_entry(pos->next, struct deferred_waiter, list);
            if (ktime_after(dw->fire_time, tmp->fire_time)) {
                pos = &tmp->list;
            } else {
                break;
            }
        }
        list_add(&dw->list, pos);

        //trace_rt_deferred_mutex_enter_deferred(current->pid, ktime_to_ns(fire_time));
    } else {
        get_task_struct(current); 
        list_add_tail(&current->rt_be_mutex_list, &mutex->wait_queue); 
    }

    while (mutex->owner != current->pid) {
        set_current_state(TASK_UNINTERRUPTIBLE);
        spin_unlock_irqrestore(&mutex->lock, flags);
        schedule();
        spin_lock_irqsave(&mutex->lock, flags);
        set_current_state(TASK_RUNNING);
    }

    //trace_rt_deferred_mutex_acquire(current->pid, mutex->owner, is_rt, account_overrun);
    spin_unlock_irqrestore(&mutex->lock, flags);
    return 0;
}

static int rt_deferred_mutex_unlock(struct rt_deferred_mutex *mutex)
{ // Release and grant next
    struct task_struct *next_task;
    pid_t next_owner = -1;
    unsigned long flags;

    spin_lock_irqsave(&mutex->lock, flags);

    if (mutex->owner != current->pid)
    {
        spin_unlock_irqrestore(&mutex->lock, flags);
        pr_warn_ratelimited("rt_deferred_mutex: Unlock permission error: owner=%d, current=%d, process=%d\n",
                            mutex->owner, current->pid, current->tgid);
        return -EPERM;
    }

    mutex->owner = -1;
    unboost_task(current);

    // Move ready deferred to front of wait queue
    list_splice_init(&mutex->ready_deferred, &mutex->wait_queue);

retry:
    if (!list_empty(&mutex->wait_queue))
    {
        next_task = list_first_entry(&mutex->wait_queue, struct task_struct, rt_be_mutex_list);

        if (!pid_task(find_vpid(next_task->pid), PIDTYPE_PID) ||
            (next_task->flags & PF_EXITING) ||
            (next_task->state == TASK_DEAD) ||
            (next_task->exit_state != 0))
        {
            pr_warn_ratelimited("rt_deferred_mutex: Waiter task %d is dead/exiting\n", next_task->pid);
            list_del(&next_task->rt_be_mutex_list);
            put_task_struct(next_task);
            goto retry;
        }

        list_del(&next_task->rt_be_mutex_list);
        mutex->owner = next_task->pid;
        next_owner = next_task->pid;
        boost_task(next_task, true); // TODO: Handle overrun toggle using account_overrun arg
        spin_unlock_irqrestore(&mutex->lock, flags);
        wake_up_process(next_task);
        put_task_struct(next_task); 

        //trace_rt_deferred_mutex_release(current->pid, next_owner);

        // Handle current task throttling properly
        if (current->policy == SCHED_DEADLINE && current->dl.runtime <= 0)
        {
            set_tsk_need_resched(current);
            schedule();
        }

        return 0;
    }

    spin_unlock_irqrestore(&mutex->lock, flags);

    //trace_rt_deferred_mutex_release(current->pid, next_owner);

    // Handle current task throttling
    if (current->policy == SCHED_DEADLINE && current->dl.runtime <= 0)
    {
        set_tsk_need_resched(current);
        schedule();
    }

    return 0;
}

static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct lock_args args;
    int dummy;

    if (!dev_class)
    {
        pr_alert("rt_deferred_mutex: Device class destroyed, rejecting IOCTL\n");
        return -ENODEV;
    }

    switch (cmd)
    {
    case IOCTL_LOCK:
        if (copy_from_user(&args, (void __user *)arg, sizeof(args)))
            return -EFAULT;
        return rt_deferred_mutex_lock(&my_mutex, args.task_type == TASK_TYPE_RT, args.account_overrun);

    case IOCTL_UNLOCK:
        if (copy_from_user(&dummy, (void __user *)arg, sizeof(int)))
            return -EFAULT;
        return rt_deferred_mutex_unlock(&my_mutex);

    default:
        return -EINVAL;
    }
}

static int dev_release(struct inode *inode, struct file *file)
{
    struct task_struct *task;
    struct deferred_waiter *dw;
    struct list_head *pos, *n;
    struct list_head deferred_to_cancel;

    unsigned long flags;

    INIT_LIST_HEAD(&deferred_to_cancel);

    spin_lock_irqsave(&my_mutex.lock, flags);

    if (my_mutex.owner == current->pid)
    {
        pr_info("rt_deferred_mutex: Process %d closed FD while owning lock, forcing unlock.\n", current->pid);
        my_mutex.owner = -1;
        unboost_task(current);
    }

    // Clean up wait queue
    list_for_each_safe(pos, n, &my_mutex.wait_queue)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        if (task->tgid == current->tgid)
        {
            list_del(pos);
            put_task_struct(task);
        }
    }

    // Clean up ready deferred
    list_for_each_safe(pos, n, &my_mutex.ready_deferred)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        if (task->tgid == current->tgid)
        {
            list_del(pos);
            put_task_struct(task);
        }
    }

    // Move deferred waiters to cancel list (don't cancel timer under spinlock)
    list_for_each_safe(pos, n, &my_mutex.deferred_queue)
    {
        dw = list_entry(pos, struct deferred_waiter, list);
        if (dw->task->tgid == current->tgid)
        {
            list_del(pos);
            list_add(&dw->list, &deferred_to_cancel);
        }
    }

    spin_unlock_irqrestore(&my_mutex.lock, flags);

    // Cancel timers outside spinlock to avoid deadlock
    list_for_each_safe(pos, n, &deferred_to_cancel)
    {
        dw = list_entry(pos, struct deferred_waiter, list);
        hrtimer_cancel(&dw->timer); 
        list_del(pos);
        put_task_struct(dw->task);
        kfree(dw);
    }

    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = dev_ioctl,
    .release = dev_release,
};

static void cleanup_all_waiters(void)
{
    struct list_head *pos, *n;
    struct task_struct *task;
    struct deferred_waiter *dw;
    unsigned long flags;

    spin_lock_irqsave(&my_mutex.lock, flags);

    list_for_each_safe(pos, n, &my_mutex.wait_queue)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        list_del(pos);
        wake_up_process(task);
        put_task_struct(task);
    }

    list_for_each_safe(pos, n, &my_mutex.ready_deferred)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        list_del(pos);
        wake_up_process(task);
        put_task_struct(task);
    }

    list_for_each_safe(pos, n, &my_mutex.deferred_queue)
    {
        dw = list_entry(pos, struct deferred_waiter, list);
        hrtimer_cancel(&dw->timer);
        list_del(pos);
        wake_up_process(dw->task);
        put_task_struct(dw->task);
        kfree(dw);
    }

    if (my_mutex.owner != -1)
    {
        pr_warn("rt_deferred_mutex: Force-unlocking. Owner=%d\n", my_mutex.owner);
        my_mutex.owner = -1;
    }

    spin_unlock_irqrestore(&my_mutex.lock, flags);
}

static int __init rt_deferred_mutex_init(void)
{
    int result;

    init_rt_deferred_mutex(&my_mutex);

    result = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
    if (result < 0)
    {
        pr_alert("rt_deferred_mutex: Failed to allocate a major number\n");
        return result;
    }

    cdev_init(&rt_be_cdev, &fops);
    rt_be_cdev.owner = THIS_MODULE;
    result = cdev_add(&rt_be_cdev, dev_number, 1);
    if (result < 0)
    {
        unregister_chrdev_region(dev_number, 1);
        pr_alert("rt_deferred_mutex: Failed to add cdev\n");
        return result;
    }

    dev_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(dev_class))
    {
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        pr_alert("rt_deferred_mutex: Failed to create class\n");
        return PTR_ERR(dev_class);
    }

    if (!device_create(dev_class, NULL, dev_number, NULL, DEVICE_NAME))
    {
        class_destroy(dev_class);
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        pr_alert("rt_deferred_mutex: Failed to create device\n");
        return -1;
    }

    pr_info("rt_deferred_mutex: Module loaded, major=%d\n", MAJOR(dev_number));
    return 0;
}

static void __exit rt_deferred_mutex_exit(void)
{
    cleanup_all_waiters();
    device_destroy(dev_class, dev_number);
    class_destroy(dev_class);
    cdev_del(&rt_be_cdev);
    unregister_chrdev_region(dev_number, 1);
    pr_info("rt_deferred_mutex: Module unloaded\n");
}

module_init(rt_deferred_mutex_init);
module_exit(rt_deferred_mutex_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Enright");
MODULE_DESCRIPTION("RT Deferred Mutex Kernel Module.");