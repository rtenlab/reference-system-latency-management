# RT Deferred Mutex Kernel Module

## Overview

This kernel module implements a custom mutex mechanism called "RT Deferred Mutex" (Real-Time Deferred Mutex) designed for synchronization between real-time (SCHED_DEADLINE) and best-effort tasks in a Linux kernel environment. It is an out-of-tree module, meaning it is compiled and loaded separately from the main kernel. The module exposes a character device (`/dev/rt_be_mutex`) that user-space applications can interact with via ioctls to lock and unlock the mutex.

The mutex supports special handling for throttled SCHED_DEADLINE tasks by deferring their wait using high-resolution timers (hrtimers) until their deadline replenishment. It also includes "boosting" for real-time tasks to make them non-preemptible and optionally account for overruns (using custom kernel patches for fields like `dl_non_preemptible` and `dl_account_overrun`).

Key features:
- FIFO wait queue for non-throttled tasks.
- Sorted deferred queue for throttled DL tasks (sorted by replenishment time).
- Boosting/unboosting of DL tasks upon acquiring/releasing the mutex.
- Cleanup on file release or module unload to avoid leaks or deadlocks.

**Note**: This module assumes a patched kernel with custom fields in `struct sched_dl_entity` (`dl_non_preemptible`, `dl_account_overrun`) and `struct task_struct` (`rt_be_mutex_list`). It also assumes special accounting for these additional fields inside of deadline.c. It will not work on a standard kernel without these modifications.

## Dependencies and Assumptions

- Linux kernel version: Tested on 5.10.120-rt70 (custom RT patchset).
- Custom kernel patches: 
  - Addition of `dl_non_preemptible` and `dl_account_overrun` in `struct sched_dl_entity`.
  - Addition of `struct list_head rt_be_mutex_list` in `struct task_struct` for queuing tasks.
  - Addition of logic for allowing non-preemptible dl entity status, as well as budget overrun accounting.  
- Build environment: Kernel headers and build tools matching your running kernel.

## Data Structures

### `struct lock_args`
- Used for IOCTL_LOCK ioctl.
- Fields:
  - `int task_type`: TASK_TYPE_RT (1) for real-time, TASK_TYPE_BE (2) for best-effort (currently unused in logic).
  - `bool account_overrun`: If true, enables overrun accounting during boosting.

### `struct deferred_waiter`
- For throttled tasks in the deferred queue.
- Fields:
  - `struct list_head list`: For linking in `deferred_queue`.
  - `struct task_struct *task`: Pointer to the throttled task (refcount held).
  - `struct hrtimer timer`: Timer set to fire at replenishment time.
  - `ktime_t fire_time`: Absolute time for timer (based on task's deadline).
  - `struct rt_be_mutex *mutex`: Backpointer to the mutex.

### `struct rt_be_mutex`
- Core mutex state (single global instance: `my_mutex`).
- Fields:
  - `pid_t owner`: PID of current owner (-1 if free).
  - `struct list_head wait_queue`: FIFO list for non-throttled waiters (uses task's `rt_be_mutex_list`).
  - `struct list_head deferred_queue`: Sorted list for throttled waiters (by `fire_time`).
  - `struct list_head ready_deferred`: List for deferred tasks whose timers have fired (moved to front of `wait_queue`).
  - `spinlock_t lock`: Protects all fields for concurrency.

## Key Functions

### Initialization and Cleanup
- `init_rt_be_mutex(struct rt_be_mutex *mutex)`: Initializes the mutex (owner = -1, empty lists, spinlock init).
- `__init rt_be_mutex_init(void)`: Module init – allocates char device, registers cdev, creates sysfs class and device.
- `__exit rt_be_mutex_exit(void)`: Module exit – cleans up waiters, destroys device/class/cdev.
- `cleanup_all_waiters(void)`: Wakes and removes all waiters/deferred tasks on unload (with timer cancels).
- `dev_release(struct inode *inode, struct file *file)`: Called on file close – forces unlock if owned, cleans up per-process waiters/deferred entries.

### Locking and Unlocking
- `rt_be_mutex_lock(struct rt_be_mutex *mutex, bool is_rt, bool account_overrun)`:
  - Acquires spinlock.
  - If free or already owned by current, sets owner, boosts task, releases spinlock.
  - If throttled DL task: Allocates `deferred_waiter`, sets timer to deadline, inserts sorted into `deferred_queue`.
  - Else: Adds task to tail of `wait_queue`.
  - Loops: Sleeps uninterruptibly until owner matches current (woken by unlock or timer).
- `rt_be_mutex_unlock(struct rt_be_mutex *mutex)`:
  - Acquires spinlock.
  - Checks permission (must be owner).
  - Clears owner, unboosts current.
  - Moves `ready_deferred` to front of `wait_queue`.
  - If queue non-empty: Validates next task (skips dead/exiting), grants ownership, boosts, wakes.
  - Handles throttling for current task by rescheduling if runtime <= 0.


### Boosting and Deferred Handling
- `boost_task(struct task_struct *task, bool account_overrun)`: Sets `dl_non_preemptible=1` and optionally `dl_account_overrun=1` for DL tasks.
- `unboost_task(struct task_struct *task)`: Resets above fields to 0.
- `process_ready_deferred(struct rt_be_mutex *mutex)`: If mutex free and ready_deferred non-empty, splices to wait_queue front, grants to first valid task (with validation, boost, wake).
- `deferred_timer_cb(struct hrtimer *timer)`: Timer callback – removes from deferred_queue, adds to ready_deferred, calls process_ready_deferred.

### IOCTL Interface
- Device: `/dev/rt_be_mutex`.
- `IOCTL_LOCK`: Copies `lock_args` from user, calls `rt_be_mutex_lock`.
- `IOCTL_UNLOCK`: Dummy int from user, calls `rt_be_mutex_unlock`.

## Build and Installation

1. **Files Needed**:
   - `rt_be_mutex.c`: Main source.
   - `Makefile`: As provided above.

2. **Build**:
   - Run `make` in the module directory. This invokes the kernel build system.
   - Outputs `rt_be_mutex.ko`.

3. **Load/Unload**:
   - `insmod rt_be_mutex.ko` (as root).
   - Check `dmesg` for "Module loaded, major=...".
   - `rmmod rt_be_mutex` to unload.

4. **Usage Example**:
   - Open `/dev/rt_be_mutex` in user-space.
   - Use `ioctl(fd, IOCTL_LOCK, &args)` to lock.
   - Use `ioctl(fd, IOCTL_UNLOCK, &dummy)` to unlock.
   - Enable tracing before running your app.
