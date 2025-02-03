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

#define DEVICE_NAME "rt_be_mutex"
#define IOCTL_LOCK   _IOW('r', 1, int)
#define IOCTL_UNLOCK _IOW('r', 2, int)

#define TASK_TYPE_RT 1
#define TASK_TYPE_BE 2

static dev_t dev_number;           // Device number (major + minor)
static struct class *dev_class;    // Device class
static struct cdev rt_be_cdev;     // Character device structure
static struct semaphore sem;       // Semaphore for protecting critical sections

// Mutex structure with separate RT and BE queues
struct rt_be_mutex {
    pid_t owner;                   // PID of the task holding the lock
    struct list_head rt_queue;      // RT queue (linked list)
    struct list_head be_queue;      // BE queue (linked list)
};

struct task_node {
    struct task_struct *task;       // Task waiting for the lock
    struct list_head list;          // List node
};

static struct rt_be_mutex my_mutex;

// Initialize the mutex
static void init_rt_be_mutex(struct rt_be_mutex *mutex) {
    mutex->owner = -1;
    INIT_LIST_HEAD(&mutex->rt_queue);
    INIT_LIST_HEAD(&mutex->be_queue);
}

// Lock function
static int rt_be_mutex_lock(struct rt_be_mutex *mutex, bool is_rt) {
    struct task_node *node;

    down(&sem); // Protect the critical section

    if (mutex->owner == -1) {
        // Lock is free
        mutex->owner = current->pid;
        up(&sem);
        return 0;
    }

    // Lock is busy, enqueue the task based on its type
    node = kmalloc(sizeof(struct task_node), GFP_KERNEL);
    if (!node) {
        up(&sem);
        return -ENOMEM;
    }
    node->task = current;

    if (is_rt) {
        list_add_tail(&node->list, &mutex->rt_queue);
    } else {
        list_add_tail(&node->list, &mutex->be_queue);
    }

    up(&sem);
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule(); // Put the task to sleep until woken up

    return 0;
}

static int rt_be_mutex_unlock(struct rt_be_mutex *mutex) {
    struct task_node *next_task;
    struct list_head *queue;

    down(&sem);

    printk(KERN_INFO "rt_be_mutex: Unlock requested by PID %d (current owner: %d)\n", current->pid, mutex->owner);

    if (mutex->owner != current->pid) {
        printk(KERN_ALERT "rt_be_mutex: Unlock failed! PID %d is not the owner\n", current->pid);
        up(&sem);
        return -EPERM;
    }

    mutex->owner = -1;

    // Check the RT queue first
    if (!list_empty(&mutex->rt_queue)) {
        queue = &mutex->rt_queue;
    } else if (!list_empty(&mutex->be_queue)) {
        queue = &mutex->be_queue;
    } else {
        // No tasks are waiting
        up(&sem);
        return 0;
    }

    next_task = list_first_entry(queue, struct task_node, list);
    list_del(&next_task->list); // Remove from queue
    mutex->owner = next_task->task->pid;

    wake_up_process(next_task->task); // Wake up the next task
    kfree(next_task);

    printk(KERN_INFO "rt_be_mutex: Lock transferred to PID %d\n", mutex->owner);

    up(&sem);
    return 0;
}


static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int task_type;

    switch (cmd) {
        case IOCTL_LOCK:
            if (copy_from_user(&task_type, (int __user *)arg, sizeof(task_type)))
                return -EFAULT;

            if (task_type == TASK_TYPE_RT) {
                return rt_be_mutex_lock(&my_mutex, true);
            } else if (task_type == TASK_TYPE_BE) {
                return rt_be_mutex_lock(&my_mutex, false);
            } else {
                return -EINVAL;
            }

        case IOCTL_UNLOCK:
            return rt_be_mutex_unlock(&my_mutex);  // 🔹 Fix: No copy_from_user

        default:
            return -EINVAL;
    }
}


// File operations
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = dev_ioctl,
};

// Module initialization
static int __init rt_be_mutex_init(void) {
    int result;

    // Initialize semaphore
    sema_init(&sem, 1);

    // Initialize the mutex
    init_rt_be_mutex(&my_mutex);

    // Allocate a major number dynamically
    result = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
    if (result < 0) {
        printk(KERN_ALERT "rt_be_mutex: Failed to allocate a major number\n");
        return result;
    }

    // Initialize and add cdev
    cdev_init(&rt_be_cdev, &fops);
    rt_be_cdev.owner = THIS_MODULE;
    result = cdev_add(&rt_be_cdev, dev_number, 1);
    if (result < 0) {
        unregister_chrdev_region(dev_number, 1);
        printk(KERN_ALERT "rt_be_mutex: Failed to add cdev\n");
        return result;
    }

    // Create device class
    dev_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(dev_class)) {
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        printk(KERN_ALERT "rt_be_mutex: Failed to create device class\n");
        return PTR_ERR(dev_class);
    }

    // Create device
    if (device_create(dev_class, NULL, dev_number, NULL, DEVICE_NAME) == NULL) {
        class_destroy(dev_class);
        cdev_del(&rt_be_cdev);
        unregister_chrdev_region(dev_number, 1);
        printk(KERN_ALERT "rt_be_mutex: Failed to create device\n");
        return -1;
    }

    printk(KERN_INFO "rt_be_mutex: Module loaded with major number %d\n", MAJOR(dev_number));
    return 0;
}

// Module cleanup
static void __exit rt_be_mutex_exit(void) {
    device_destroy(dev_class, dev_number);
    class_destroy(dev_class);
    cdev_del(&rt_be_cdev);
    unregister_chrdev_region(dev_number, 1);
    printk(KERN_INFO "rt_be_mutex: Module unloaded\n");
}

module_init(rt_be_mutex_init);
module_exit(rt_be_mutex_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Enright");
MODULE_DESCRIPTION("RT-BE Mutex Module with Dynamic Major Allocation");
