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

    pr_info("RT_MUTEX: Boost task PID=%d, non_preemptible=%d, account_overrun=%d, runtime=%lld, deadline=%lld\n",
            task->pid, task->dl.dl_non_preemptible, task->dl.dl_account_overrun, task->dl.runtime, task->dl.deadline);
}

static void unboost_task(struct task_struct *task)
{
    if (!task || task->policy != SCHED_DEADLINE)
        return;

    pr_info("RT_MUTEX: Unboost task PID=%d, runtime=%lld, deadline=%lld\n",
            task->pid, task->dl.runtime, task->dl.deadline);

    task->dl.dl_non_preemptible = 0;
    task->dl.dl_account_overrun = 0;
}

static void process_ready_deferred(struct rt_deferred_mutex *mutex)
{ // Grant to ready deferred if free
    struct task_struct *next_task;

    pr_info("RT_MUTEX: Processing ready deferred, owner=%d, ready_deferred empty=%d\n",
            mutex->owner, list_empty(&mutex->ready_deferred));

    if (mutex->owner != -1 || list_empty(&mutex->ready_deferred))
        return;

    list_splice_init(&mutex->ready_deferred, &mutex->wait_queue); 

    pr_info("RT_MUTEX: Spliced ready_deferred to wait_queue, wait_queue empty=%d\n",
            list_empty(&mutex->wait_queue));

    if (list_empty(&mutex->wait_queue))
        return;

retry:
    next_task = list_first_entry(&mutex->wait_queue, struct task_struct, rt_be_mutex_list);

    pr_info("RT_MUTEX: Checking next task in ready deferred PID=%d\n", next_task->pid);

    if (!pid_task(find_vpid(next_task->pid), PIDTYPE_PID) ||
        (next_task->flags & PF_EXITING) ||
        (next_task->state == TASK_DEAD) ||
        (next_task->exit_state != 0))
    {
        pr_warn_ratelimited("RT_MUTEX: Ready deferred task %d is dead/exiting\n", next_task->pid);
        list_del(&next_task->rt_be_mutex_list);
        put_task_struct(next_task);
        if (!list_empty(&mutex->wait_queue))
            goto retry;
        return;
    }

    list_del(&next_task->rt_be_mutex_list);
    mutex->owner = next_task->pid;
    boost_task(next_task, true); // TODO: Handle overrun toggle using account_overrun arg
    pr_info("RT_MUTEX: Granted mutex to ready deferred task PID=%d\n", next_task->pid);
    wake_up_process(next_task);
    put_task_struct(next_task); 
}

static enum hrtimer_restart deferred_timer_cb(struct hrtimer *timer)
{ // Timer fires: move to ready
    struct deferred_waiter *dw = container_of(timer, struct deferred_waiter, timer);
    struct rt_deferred_mutex *mutex = dw->mutex;
    struct task_struct *task = dw->task; 
    unsigned long flags;

    pr_info("RT_MUTEX: Deferred timer fired for task PID=%d, fire_time=%lld\n", task->pid, ktime_to_ns(dw->fire_time));

    spin_lock_irqsave(&mutex->lock, flags);
    list_del(&dw->list);
    kfree(dw);
    list_add_tail(&task->rt_be_mutex_list, &mutex->ready_deferred);
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
    pr_info("RT_MUTEX: Initialized mutex\n");
}

static int rt_deferred_mutex_lock(struct rt_deferred_mutex *mutex, bool is_rt, bool account_overrun)
{ // Acquire loop
    unsigned long flags;

    pr_info("RT_MUTEX: Task PID=%d attempting lock, is_rt=%d, account_overrun=%d\n", current->pid, is_rt, account_overrun);

    spin_lock_irqsave(&mutex->lock, flags);

    if (mutex->owner == -1) {
        mutex->owner = current->pid;
        boost_task(current, account_overrun);
        pr_info("RT_MUTEX: Mutex acquired by PID=%d (was free)\n", current->pid);
        spin_unlock_irqrestore(&mutex->lock, flags);
        return 0;
    }

    if (mutex->owner == current->pid) {
        boost_task(current, account_overrun);
        pr_info("RT_MUTEX: Mutex re-acquired by PID=%d (already owned)\n", current->pid);
        spin_unlock_irqrestore(&mutex->lock, flags);
        return 0;
    }

    // Always add to wait_queue, regardless of throttling state
    get_task_struct(current); 
    list_add_tail(&current->rt_be_mutex_list, &mutex->wait_queue); 
    pr_info("RT_MUTEX: Task PID=%d added to wait_queue\n", current->pid);

    while (mutex->owner != current->pid) {
        set_current_state(TASK_UNINTERRUPTIBLE);
        spin_unlock_irqrestore(&mutex->lock, flags);
        schedule();
        spin_lock_irqsave(&mutex->lock, flags);
        set_current_state(TASK_RUNNING);
    }

    pr_info("RT_MUTEX: Mutex acquired by PID=%d after waiting\n", current->pid);
    spin_unlock_irqrestore(&mutex->lock, flags);
    return 0;
}

static int rt_deferred_mutex_unlock(struct rt_deferred_mutex *mutex)
{ // Release and grant next
    struct task_struct *next_task;
    pid_t next_owner = -1;
    unsigned long flags;

    pr_info("RT_MUTEX: Task PID=%d attempting unlock\n", current->pid);

    spin_lock_irqsave(&mutex->lock, flags);

    if (mutex->owner != current->pid)
    {
        spin_unlock_irqrestore(&mutex->lock, flags);
        pr_warn_ratelimited("RT_MUTEX: Unlock permission error: owner=%d, current=%d, process=%d\n",
                            mutex->owner, current->pid, current->tgid);
        return -EPERM;
    }

    mutex->owner = -1;
    unboost_task(current);
    pr_info("RT_MUTEX: Mutex released by PID=%d\n", current->pid);

    // Move ready deferred to front of wait queue
    list_splice_init(&mutex->ready_deferred, &mutex->wait_queue);
    pr_info("RT_MUTEX: Spliced ready_deferred to wait_queue\n");

retry:
    if (!list_empty(&mutex->wait_queue))
    {
        next_task = list_first_entry(&mutex->wait_queue, struct task_struct, rt_be_mutex_list);

        pr_info("RT_MUTEX: Considering next task PID=%d for grant\n", next_task->pid);

        if (!pid_task(find_vpid(next_task->pid), PIDTYPE_PID) ||
            (next_task->flags & PF_EXITING) ||
            (next_task->state == TASK_DEAD) ||
            (next_task->exit_state != 0))
        {
            pr_warn_ratelimited("RT_MUTEX: Waiter task %d is dead/exiting\n", next_task->pid);
            list_del(&next_task->rt_be_mutex_list);
            put_task_struct(next_task);
            goto retry;
        }

        // Check if the next task is throttled
        //if (next_task->policy == SCHED_DEADLINE && (next_task->dl.dl_throttled || !next_task->on_cpu)) {
                if (next_task->policy == SCHED_DEADLINE && next_task->dl.dl_throttled) {
  
        // Move to deferred queue instead of granting immediately
            struct deferred_waiter *dw;
            struct list_head *pos;
            ktime_t fire_time = ns_to_ktime(next_task->dl.deadline);

            // ktime_t fire_time;
            // ktime_t now = ktime_get();

            // if(next_task->dl.dl_throttled){
            //     fire_time = ns_to_ktime(next_task->dl.deadline + next_task->dl.dl_period - next_task->dl.dl_runtime);
            //     if(ktime_before(fire_time, now))
            //     {
            //         pr_info("RT_MUTEX: Next task PID=%d is throttled, but fire_time %lld is before now %lld, deferring\n",
            //                 next_task->pid, ktime_to_ns(fire_time), ktime_to_ns(now));
            //     }
            // }
            // else { // if not throttled, but also not active yet, select latest possible fire time
            //     fire_time = ns_to_ktime(next_task->dl.deadline - next_task->dl.dl_runtime);
            //     if(ktime_before(fire_time, now))
            //     {
            //         pr_info("RT_MUTEX: Next task PID=%d is not throttled, but fire_time %lld is before now %lld, deferring\n",
            //                 next_task->pid, ktime_to_ns(fire_time), ktime_to_ns(now));
            //     }
            // }

            // pr_info("RT_MUTEX: Next task PID=%d is throttled, deferring (fire_time=%lld, now=%lld)\n",
            //         next_task->pid, ktime_to_ns(fire_time), ktime_to_ns(now));

            dw = kmalloc(sizeof(*dw), GFP_ATOMIC);
            if (!dw) {
                spin_unlock_irqrestore(&mutex->lock, flags);
                return -ENOMEM;
            }

            // Transfer the task ref from wait_queue to deferred_waiter
            dw->task = next_task;
            dw->mutex = mutex;
            dw->fire_time = fire_time;

            hrtimer_init(&dw->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
            dw->timer.function = deferred_timer_cb;
            hrtimer_start(&dw->timer, dw->fire_time, HRTIMER_MODE_ABS);

            // Remove from wait_queue
            list_del(&next_task->rt_be_mutex_list);

            // Insert in sorted order into deferred_queue
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

            // Retry for next waiter
            goto retry;
        } else {
            // Not throttled, grant ownership
            list_del(&next_task->rt_be_mutex_list);
            mutex->owner = next_task->pid;
            next_owner = next_task->pid;
            boost_task(next_task, true); // TODO: Handle overrun toggle using account_overrun arg
            spin_unlock_irqrestore(&mutex->lock, flags);
            pr_info("RT_MUTEX: Granted mutex to non-throttled task PID=%d\n", next_task->pid);
            wake_up_process(next_task);
            put_task_struct(next_task); 

            // Handle current task throttling properly
            if (current->policy == SCHED_DEADLINE && current->dl.runtime <= 0)
            {
                pr_info("RT_MUTEX: Current task PID=%d throttled after unlock, rescheduling\n", current->pid);
                set_tsk_need_resched(current);
                schedule();
            }

            return 0;
        }
    }

    spin_unlock_irqrestore(&mutex->lock, flags);

    // Handle current task throttling
    if (current->policy == SCHED_DEADLINE && current->dl.runtime <= 0)
    {
        pr_info("RT_MUTEX: Current task PID=%d throttled after unlock (no waiters), rescheduling\n", current->pid);
        set_tsk_need_resched(current);
        schedule();
    }

    pr_info("RT_MUTEX: Unlock completed, no waiters\n");
    return 0;
}

static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct lock_args args;
    int dummy;

    pr_info("RT_MUTEX: IOCTL called, cmd=%u\n", cmd);

    if (!dev_class)
    {
        pr_alert("RT_MUTEX: Device class destroyed, rejecting IOCTL\n");
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

    pr_info("RT_MUTEX: File release for PID=%d\n", current->pid);

    INIT_LIST_HEAD(&deferred_to_cancel);

    spin_lock_irqsave(&my_mutex.lock, flags);

    if (my_mutex.owner == current->pid)
    {
        pr_info("RT_MUTEX: Process %d closed FD while owning lock, forcing unlock.\n", current->pid);
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
            pr_info("RT_MUTEX: Removed task PID=%d from wait_queue during release\n", task->pid);
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
            pr_info("RT_MUTEX: Removed task PID=%d from ready_deferred during release\n", task->pid);
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
            pr_info("RT_MUTEX: Moved deferred waiter PID=%d to cancel list during release\n", dw->task->pid);
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
        pr_info("RT_MUTEX: Canceled timer and freed deferred waiter during release\n");
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

    pr_info("RT_MUTEX: Cleaning all waiters on module exit\n");

    spin_lock_irqsave(&my_mutex.lock, flags);

    list_for_each_safe(pos, n, &my_mutex.wait_queue)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        list_del(pos);
        wake_up_process(task);
        put_task_struct(task);
        pr_info("RT_MUTEX: Cleaned task PID=%d from wait_queue on exit\n", task->pid);
    }

    list_for_each_safe(pos, n, &my_mutex.ready_deferred)
    {
        task = list_entry(pos, struct task_struct, rt_be_mutex_list);
        list_del(pos);
        wake_up_process(task);
        put_task_struct(task);
        pr_info("RT_MUTEX: Cleaned task PID=%d from ready_deferred on exit\n", task->pid);
    }

    list_for_each_safe(pos, n, &my_mutex.deferred_queue)
    {
        dw = list_entry(pos, struct deferred_waiter, list);
        hrtimer_cancel(&dw->timer);
        list_del(pos);
        wake_up_process(dw->task);
        put_task_struct(dw->task);
        kfree(dw);
        pr_info("RT_MUTEX: Cleaned deferred waiter PID=%d on exit\n", dw->task->pid);
    }

    if (my_mutex.owner != -1)
    {
        pr_warn("RT_MUTEX: Force-unlocking. Owner=%d\n", my_mutex.owner);
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
        pr_alert("RT_MUTEX: Failed to allocate a major number\n");
        return result;
    }

    cdev_init(&rt_be_cdev, &fops);
    rt_be_cdev.owner = THIS_MODULE;
    result = cdev_add(&rt_be_cdev, dev_number, 1);
    if (result < 0)
    {
        unregister_chrdev_region(dev_number, 1);
        pr_alert("RT_MUTEX: Failed to add cdev\n");
        return result;
    }

    dev_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(dev_class))
    {
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        pr_alert("RT_MUTEX: Failed to create class\n");
        return PTR_ERR(dev_class);
    }

    if (!device_create(dev_class, NULL, dev_number, NULL, DEVICE_NAME))
    {
        class_destroy(dev_class);
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        pr_alert("RT_MUTEX: Failed to create device\n");
        return -1;
    }

    pr_info("RT_MUTEX: Module loaded, major=%d\n", MAJOR(dev_number));
    return 0;
}

static void __exit rt_deferred_mutex_exit(void)
{
    cleanup_all_waiters();
    device_destroy(dev_class, dev_number);
    class_destroy(dev_class);
    cdev_del(&rt_be_cdev);
    unregister_chrdev_region(dev_number, 1);
    pr_info("RT_MUTEX: Module unloaded\n");
}

module_init(rt_deferred_mutex_init);
module_exit(rt_deferred_mutex_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Enright");
MODULE_DESCRIPTION("RT Deferred Mutex Kernel Module.");