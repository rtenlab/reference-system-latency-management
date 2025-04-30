#define _GNU_SOURCE
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <linux/sched.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ftw.h>

#define DEVICE_PATH "/dev/rt_be_mutex"

// ioctl commands
#define IOCTL_LOCK _IOW('r', 1, int)
#define IOCTL_UNLOCK _IOW('r', 2, int)

// Task types
#define TASK_TYPE_RT 1
#define TASK_TYPE_BE 2

// Number of threads
#define NUM_THREADS 4

// SCHED_DEADLINE parameters (in nanoseconds)
#define DL_RUNTIME_NS 50000000   // 20ms runtime
#define DL_PERIOD_NS 100000000   // 100ms period
#define DL_DEADLINE_NS 100000000 // 100ms deadline (same as period)

// Cgroup paths - for cgroups v1
#define CGROUP_CPUSET_PATH "/sys/fs/cgroup/cpuset"
#define CGROUP_BASE_PATH "/sys/fs/cgroup/cpuset/test_mutex"
#define CGROUP_CPU0_PATH "/sys/fs/cgroup/cpuset/test_mutex/cpu0"
#define CGROUP_CPU1_PATH "/sys/fs/cgroup/cpuset/test_mutex/cpu1"
struct sched_attr
{
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};
// Thread state tracking
typedef struct
{
    int thread_id;
    int cpu;
    uint64_t start_time;
    uint64_t lock_acquire_time;
    uint64_t lock_release_time;
    int acquired_lock;
    int was_throttled;
    int task_type;
    pthread_t thread;
    char cgroup_path[256]; // Store the cgroup path for this thread
} thread_data_t;

// Shared state
int shared_fd = -1;
pthread_mutex_t reporting_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_barrier_t barrier;
int cgroups_initialized = 0;

// Create a directory if it doesn't exist
int create_directory(const char *path)
{
    struct stat st = {0};
    if (stat(path, &st) == -1)
    {
        if (mkdir(path, 0755) == -1)
        {
            perror("mkdir");
            return -1;
        }
    }
    return 0;
}

// Write a string to a file
int write_to_file(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        perror("open");
        return -1;
    }

    size_t len = strlen(value);
    ssize_t ret = write(fd, value, len);
    if (ret == -1)
    {
        perror("write");
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

// Initialize cgroups for CPU binding
// Initialize cgroups for CPU binding
int init_cgroups(void)
{
    // Create the base cgroup directory
    if (create_directory(CGROUP_BASE_PATH) == -1)
    {
        return -1;
    }

    // Create cgroup for CPU 0
    if (create_directory(CGROUP_CPU0_PATH) == -1)
    {
        return -1;
    }

    // Create cgroup for CPU 1
    if (create_directory(CGROUP_CPU1_PATH) == -1)
    {
        return -1;
    }

    // For cgroups v1, we need to initialize the cpuset parameters

    // For base cgroup
    char base_cpus_path[512], base_mems_path[512];
    snprintf(base_cpus_path, sizeof(base_cpus_path), "%s/cpuset.cpus", CGROUP_BASE_PATH);
    snprintf(base_mems_path, sizeof(base_mems_path), "%s/cpuset.mems", CGROUP_BASE_PATH);

    // Get parent settings first (required for cgroups v1)
    char parent_cpus[32] = "0-1"; // Default value, assuming 2 CPUs
    char parent_mems[32] = "0";   // Default value, assuming 1 NUMA node

    // Copy parent settings
    if (write_to_file(base_cpus_path, parent_cpus) == -1)
    {
        return -1;
    }
    if (write_to_file(base_mems_path, parent_mems) == -1)
    {
        return -1;
    }

    // Set CPU affinity for CPU 0 cgroup
    char cpu0_cpus_path[512], cpu0_mems_path[512];
    snprintf(cpu0_cpus_path, sizeof(cpu0_cpus_path), "%s/cpuset.cpus", CGROUP_CPU0_PATH);
    snprintf(cpu0_mems_path, sizeof(cpu0_mems_path), "%s/cpuset.mems", CGROUP_CPU0_PATH);

    if (write_to_file(cpu0_mems_path, parent_mems) == -1)
    { // Must set mems before cpus
        return -1;
    }
    if (write_to_file(cpu0_cpus_path, "0") == -1)
    {
        return -1;
    }

    // Set CPU affinity for CPU 1 cgroup
    char cpu1_cpus_path[512], cpu1_mems_path[512];
    snprintf(cpu1_cpus_path, sizeof(cpu1_cpus_path), "%s/cpuset.cpus", CGROUP_CPU1_PATH);
    snprintf(cpu1_mems_path, sizeof(cpu1_mems_path), "%s/cpuset.mems", CGROUP_CPU1_PATH);

    if (write_to_file(cpu1_mems_path, parent_mems) == -1)
    { // Must set mems before cpus
        return -1;
    }
    if (write_to_file(cpu1_cpus_path, "1") == -1)
    {
        return -1;
    }

    return 0;
}

// Add thread to a specific CPU cgroup
int add_thread_to_cgroup(pid_t tid, int cpu)
{
    char cgroup_path[256];
    if (cpu == 0)
    {
        strcpy(cgroup_path, CGROUP_CPU0_PATH);
    }
    else
    {
        strcpy(cgroup_path, CGROUP_CPU1_PATH);
    }

    // In cgroups v1, we write to the tasks file
    char tasks_path[512];
    snprintf(tasks_path, sizeof(tasks_path), "%s/tasks", cgroup_path);

    // Convert TID to string
    char tid_str[32];
    snprintf(tid_str, sizeof(tid_str), "%d", tid);

    // Add thread to cgroup
    if (write_to_file(tasks_path, tid_str) == -1)
    {
        return -1;
    }

    return 0;
}

// Cleanup callback for nftw()
int remove_cgroup_callback(const char *path, const struct stat *sb, int typeflag, struct FTW *ftwbuf)
{
    int ret = remove(path);
    if (ret)
        perror(path);
    return ret;
}

// Cleanup cgroups
void cleanup_cgroups(void)
{
    // Remove cgroup directories recursively
    nftw(CGROUP_BASE_PATH, remove_cgroup_callback, 64, FTW_DEPTH | FTW_PHYS);
}
// System call wrapper for sched_setattr
static int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
    return syscall(SYS_sched_setattr, pid, attr, flags);
}

// System call wrapper for sched_getattr
static int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags)
{
    return syscall(SYS_sched_getattr, pid, attr, size, flags);
}

// Get current time in milliseconds
uint64_t get_time_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000ULL) + (ts.tv_nsec / 1000000ULL);
}

// Busy wait function
void busy_wait_nop(uint64_t iterations)
{
    // Use volatile to prevent optimization
    volatile uint64_t i = iterations;
    while (i--)
    {
        asm volatile("nop");
    }
}

// Check if thread is being throttled
int is_throttled()
{
    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);

    // Get current scheduling attributes
    if (sched_getattr(0, &attr, sizeof(attr), 0) < 0)
    {
        perror("sched_getattr");
        return -1;
    }

    // Check if runtime is depleted (simplified throttling check)
    return (attr.sched_runtime < DL_RUNTIME_NS / 2);
}
// In the is_throttled() function, add a check for actual preemption
int is_throttled_or_preempted()
{
    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);

    // Current implementation - checks runtime depletion
    if (sched_getattr(0, &attr, sizeof(attr), 0) < 0)
    {
        perror("sched_getattr");
        return -1;
    }

    // Check if runtime is depleted (simplified throttling check)
    if (attr.sched_runtime < DL_RUNTIME_NS / 2)
        return 1;

    // Add a preemption check - record timestamp deltas to detect gaps
    static uint64_t last_check_time = 0;
    uint64_t current_time = get_time_ms();

    if (last_check_time > 0)
    {
        // If there was a significant gap between checks, might indicate preemption
        if ((current_time - last_check_time) > 10)
        { // 10ms threshold
            printf("Possible preemption detected: %lu ms gap\n",
                   current_time - last_check_time);
            last_check_time = current_time;
            return 1;
        }
    }

    last_check_time = current_time;
    return 0;
}

// Set thread to SCHED_DEADLINE
int set_deadline_scheduling(int runtime_us, int period_us, int deadline_us)
{
    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));

    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = runtime_us * 1000;   // convert to ns
    attr.sched_period = period_us * 1000;     // convert to ns
    attr.sched_deadline = deadline_us * 1000; // convert to ns

    if (sched_setattr(0, &attr, 0) < 0)
    {
        perror("sched_setattr");
        return -1;
    }

    return 0;
}

// Set CPU affinity
int set_cpu_affinity(int cpu)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);

    if (sched_setaffinity(0, sizeof(set), &set) < 0)
    {
        perror("sched_setaffinity");
        return -1;
    }

    return 0;
}
// Improved detection of throttling and preemption
typedef struct
{
    uint64_t timestamp;         // Timestamp of the check
    uint64_t runtime_remaining; // Runtime remaining at check
    uint64_t iterations;        // Iteration count since last check
    uint64_t sched_count;       // Number of times scheduled
} check_point_t;

#define CHECK_HISTORY_SIZE 10 // Number of checkpoints to keep
static check_point_t check_history[CHECK_HISTORY_SIZE];
static int check_history_idx = 0;
static uint64_t expected_iterations_per_ms = 0;
static int calibration_done = 0;

// Get scheduler stats from /proc
int get_sched_stats(uint64_t *switches)
{
    char path[64];
    char buf[1024];
    FILE *f;

    snprintf(path, sizeof(path), "/proc/self/sched");
    f = fopen(path, "r");
    if (!f)
        return -1;

    *switches = 0;
    while (fgets(buf, sizeof(buf), f))
    {
        if (strncmp(buf, "nr_switches", 11) == 0)
        {
            sscanf(buf, "nr_switches                    : %lu", switches);
            break;
        }
    }
    fclose(f);
    return 0;
}

// Calibrate the expected iterations per millisecond
void calibrate_iterations_per_ms()
{
    uint64_t start_time = get_time_ms();
    uint64_t iterations = 0;
    uint64_t end_time;

    // Run for at least 10ms to get a stable measurement
    do
    {
        for (int i = 0; i < 1000000; i++)
        {
            asm volatile("nop");
            iterations++;
        }
        end_time = get_time_ms();
    } while (end_time - start_time < 10);

    expected_iterations_per_ms = iterations / (end_time - start_time);
    calibration_done = 1;

    printf("Calibration: %lu iterations per ms\n", expected_iterations_per_ms);
}

// Enhanced detection of throttling and preemption
int detect_throttling_or_preemption(uint64_t iterations_since_last_check)
{
    if (!calibration_done)
    {
        calibrate_iterations_per_ms();
    }

    // Get current data
    struct sched_attr attr;
    uint64_t switches = 0;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);

    if (sched_getattr(0, &attr, sizeof(attr), 0) < 0)
    {
        perror("sched_getattr");
        return -1;
    }

    get_sched_stats(&switches);
    uint64_t current_time = get_time_ms();

    // Store current checkpoint
    check_point_t *curr = &check_history[check_history_idx];
    curr->timestamp = current_time;
    curr->runtime_remaining = attr.sched_runtime;
    curr->iterations = iterations_since_last_check;
    curr->sched_count = switches;

    // Analyze for preemption or throttling
    int result = 0;

    // Get previous checkpoint
    int prev_idx = (check_history_idx + CHECK_HISTORY_SIZE - 1) % CHECK_HISTORY_SIZE;
    check_point_t *prev = &check_history[prev_idx];

    if (prev->timestamp > 0)
    { // Valid previous checkpoint
        uint64_t elapsed_time = current_time - prev->timestamp;

        // Check 1: Runtime depletion
        if (attr.sched_runtime < DL_RUNTIME_NS / 10)
        {
            printf("Critical runtime depletion: %lu ns remaining\n", attr.sched_runtime);
            result = 1;
        }

        // Check 2: Performance drop
        if (elapsed_time > 0)
        {
            uint64_t expected_iterations = expected_iterations_per_ms * elapsed_time;
            if (iterations_since_last_check < expected_iterations * 0.7 && elapsed_time > 5)
            {
                printf("Performance drop detected: %lu/%lu iterations (%.1f%%)\n",
                       iterations_since_last_check, expected_iterations,
                       (double)iterations_since_last_check * 100 / expected_iterations);
                result = 1;
            }
        }

        // Check 3: Detect context switches (definitive sign of preemption)
        if (switches > prev->sched_count)
        {
            printf("Context switches detected: %lu new switches\n",
                   switches - prev->sched_count);
            result = 1;
        }

        // Check 4: Excessive delay
        if (elapsed_time > 20)
        {
            printf("Excessive delay between checks: %lu ms\n", elapsed_time);
            result = 1;
        }
    }

    // Move to next slot in circular buffer
    check_history_idx = (check_history_idx + 1) % CHECK_HISTORY_SIZE;

    return result;
}
// Thread function
void *thread_worker(void *arg)
{
    thread_data_t *data = (thread_data_t *)arg;
    int throttled_count = 0;
    char thread_name[16];

    // Set thread name for easier identification
    snprintf(thread_name, sizeof(thread_name), "test_thd_%d", data->thread_id);
    pthread_setname_np(pthread_self(), thread_name);

    // Get thread ID (TID)
    pid_t tid = syscall(SYS_gettid);

    // Add thread to appropriate cgroup
    if (add_thread_to_cgroup(tid, data->cpu) < 0)
    {
        fprintf(stderr, "Thread %d: Failed to add to cgroup for CPU %d\n",
                data->thread_id, data->cpu);
    }
    else
    {
        printf("Thread %d: Successfully added to cgroup for CPU %d\n",
               data->thread_id, data->cpu);
    }

    // Set SCHED_DEADLINE parameters
    if (set_deadline_scheduling(
            DL_RUNTIME_NS / 1000000,
            DL_PERIOD_NS / 1000000,
            DL_DEADLINE_NS / 1000000) < 0)
    {
        fprintf(stderr, "Thread %d: Failed to set SCHED_DEADLINE\n", data->thread_id);
    }

    // Rest of the function remains the same
    // Wait for all threads to be ready
    pthread_barrier_wait(&barrier);

    // Record start time
    data->start_time = get_time_ms();

    printf("Thread %d (CPU %d): Started at %lu ms\n",
           data->thread_id, data->cpu, data->start_time);

    // Try to acquire the lock
    printf("Thread %d: Requesting lock as %s task...\n",
           data->thread_id, data->task_type == TASK_TYPE_RT ? "RT" : "BE");

    if (ioctl(shared_fd, IOCTL_LOCK, &data->task_type) < 0)
    {
        perror("Failed to acquire lock");
        return NULL;
    }

    // Successfully acquired the lock
    data->acquired_lock = 1;
    data->lock_acquire_time = get_time_ms();

    printf("Thread %d: Lock acquired at %lu ms (waited %lu ms)\n",
           data->thread_id, data->lock_acquire_time,
           data->lock_acquire_time - data->start_time);

    // Simulate critical section with busy wait
    // Also periodically check if we're being throttled
    uint64_t check_interval = 50000000ULL; // Check every ~50M iterations
    // uint64_t total_iterations = 2200000000ULL * 3; // ~3 seconds of work
    uint64_t total_iterations = 220000000ULL * 2; // ~0.1 sec of work
                                                  // In thread_worker function
    uint64_t iterations_since_last_check = 0;

    // During the critical section loop
    for (uint64_t i = 0; i < total_iterations; i++)
    {
        asm volatile("nop");
        iterations_since_last_check++;

        // Periodically check throttling
        if (i % check_interval == 0)
        {
            if (detect_throttling_or_preemption(iterations_since_last_check))
            {
                throttled_count++;

                pthread_mutex_lock(&reporting_mutex);
                printf("Thread %d: [PREEMPTION/THROTTLING DETECTED] at %lu ms\n",
                       data->thread_id, get_time_ms());
                pthread_mutex_unlock(&reporting_mutex);

                iterations_since_last_check = 0;
            }
        }
    }

    // Record throttling status
    data->was_throttled = (throttled_count > 0);

    // Release the lock
    printf("Thread %d: Releasing lock...\n", data->thread_id);

    if (ioctl(shared_fd, IOCTL_UNLOCK, NULL) < 0)
    {
        perror("Failed to release lock");
    }

    data->lock_release_time = get_time_ms();

    printf("Thread %d: Lock released at %lu ms (held for %lu ms)\n",
           data->thread_id, data->lock_release_time,
           data->lock_release_time - data->lock_acquire_time);

    return NULL;
}

int main(int argc, char *argv[])
{
    thread_data_t thread_data[NUM_THREADS];
    int i;

    // Open the mutex device
    shared_fd = open(DEVICE_PATH, O_RDWR);
    if (shared_fd < 0)
    {
        perror("Failed to open the device");
        return -1;
    }

    // Initialize cgroups
    printf("Setting up cgroups for CPU binding...\n");
    if (init_cgroups() < 0)
    {
        fprintf(stderr, "Failed to initialize cgroups. Are you running as root?\n");
        fprintf(stderr, "Make sure cgroup v2 is mounted at /sys/fs/cgroup\n");
        close(shared_fd);
        return -1;
    }
    cgroups_initialized = 1;

    // Initialize barrier
    pthread_barrier_init(&barrier, NULL, NUM_THREADS);

    // Set up thread data
    for (i = 0; i < NUM_THREADS; i++)
    {
        thread_data[i].thread_id = i;
        thread_data[i].cpu = i < 2 ? 0 : 1; // First two threads on CPU 0, others on CPU 1
        thread_data[i].start_time = 0;
        thread_data[i].lock_acquire_time = 0;
        thread_data[i].lock_release_time = 0;
        thread_data[i].acquired_lock = 0;
        thread_data[i].was_throttled = 0;
        thread_data[i].task_type = (i % 2 == 0) ? TASK_TYPE_RT : TASK_TYPE_BE;
    }
    // Create threads
    printf("Creating %d threads...\n", NUM_THREADS);
    for (i = 0; i < NUM_THREADS; i++)
    {
        if (pthread_create(&thread_data[i].thread, NULL, thread_worker, &thread_data[i]) != 0)
        {
            perror("Failed to create thread");
            return -1;
        }
    }

    // Wait for all threads to finish
    for (i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(thread_data[i].thread, NULL);
    }

    // Print summary report
    printf("\n===== TEST RESULTS =====\n");
    printf("Thread | CPU | Type | Acquired Lock | Was Throttled | Wait Time (ms) | Hold Time (ms)\n");
    printf("-------|-----|------|---------------|---------------|----------------|---------------\n");

    for (i = 0; i < NUM_THREADS; i++)
    {
        printf("  %d    |  %d  |  %s  |      %s      |      %s      |      %5lu      |      %5lu\n",
               thread_data[i].thread_id,
               thread_data[i].cpu,
               thread_data[i].task_type == TASK_TYPE_RT ? "RT" : "BE",
               thread_data[i].acquired_lock ? "YES" : "NO ",
               thread_data[i].was_throttled ? "YES" : "NO ",
               thread_data[i].lock_acquire_time > 0 ? thread_data[i].lock_acquire_time - thread_data[i].start_time : 0,
               thread_data[i].lock_release_time > 0 && thread_data[i].lock_acquire_time > 0 ? thread_data[i].lock_release_time - thread_data[i].lock_acquire_time : 0);
    }

    printf("\nConclusion: ");
    int throttle_count = 0;
    for (i = 0; i < NUM_THREADS; i++)
    {
        if (thread_data[i].acquired_lock && thread_data[i].was_throttled)
        {
            throttle_count++;
        }
    }

    if (throttle_count == 0)
    {
        printf("BOOSTING WORKS! No thread was throttled while holding the lock.\n");
    }
    else
    {
        printf("BOOSTING FAILED! %d thread(s) were throttled while holding the lock.\n",
               throttle_count);
    }

    // Clean up
    pthread_barrier_destroy(&barrier);
    close(shared_fd);

    if (cgroups_initialized)
    {
        printf("Cleaning up cgroups...\n");
        cleanup_cgroups();
    }

    return 0;
}
