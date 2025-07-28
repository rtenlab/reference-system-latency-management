#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cstring>
#include <sched.h>
#include <pthread.h>
#include <argp.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <sys/syscall.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/sched.h>
#include <sys/vfs.h>
#include <nvtx3/nvToolsExt.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <csignal>
#include <dirent.h>
#define THREAD_PERIOD_US 10000 // 10ms period for SCHED_DEADLINE
// Device definitions
#define DEVICE_NAME "rt_be_mutex"                 // Device name in /dev
#define IOCTL_LOCK _IOW('r', 1, struct lock_args) // Lock ioctl (write struct)
#define IOCTL_UNLOCK _IOW('r', 2, int)            // Unlock ioctl (dummy int)

#define TASK_TYPE_RT 1 // Real-time (DL) type
#define TASK_TYPE_BE 2 // Best-effort type (unused)

struct sched_attr
{
    uint32_t size;

    uint32_t sched_policy;
    uint64_t sched_flags;

    /* SCHED_NORMAL, SCHED_BATCH */
    uint32_t sched_nice;

    /* SCHED_FIFO, SCHED_RR */
    uint32_t sched_priority;

    /* SCHED_DEADLINE (nsec) */
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;

    /* Utilization hints */
    uint32_t sched_util_min;
    uint32_t sched_util_max;
};

struct lock_args
{                         // Arg struct for IOCTL_LOCK
    int task_type;        // RT or BE
    bool account_overrun; // Enable overrun debt
};

// Custom FIFO Mutex (fair, waiter-order preserving)
class FifoMutex
{
private:
    std::mutex m_internal;
    std::condition_variable m_cv;
    std::atomic<uint64_t> m_ticket{0};
    std::atomic<uint64_t> m_serving{0};

public:
    void lock()
    {
        uint64_t my_ticket = m_ticket.fetch_add(1, std::memory_order_relaxed);
        std::unique_lock<std::mutex> lock(m_internal);
        m_cv.wait(lock, [this, my_ticket]()
                  { return m_serving.load(std::memory_order_acquire) == my_ticket; });
    }
    void unlock()
    {
        {
            std::lock_guard<std::mutex> lock(m_internal);
            m_serving.fetch_add(1, std::memory_order_release);
        }
        m_cv.notify_one();
    }
};

// Kernel RT-BE Mutex via char device
class RtBeMutex
{
private:
    int m_fd;

public:
    RtBeMutex()
    {
        m_fd = open("/dev/" DEVICE_NAME, O_RDWR);
        if (m_fd < 0)
        {
            perror("Failed to open RT-BE mutex device");
            exit(1);
        }
    }
    ~RtBeMutex()
    {
        if (close(m_fd) < 0)
        {
            perror("Failed to close RT-BE mutex device");
        }
    }
    void lock()
    {
        struct lock_args args = {TASK_TYPE_RT, true};
        if (ioctl(m_fd, IOCTL_LOCK, &args) < 0)
        {
            perror("IOCTL_LOCK failed");
        }
    }
    void unlock()
    {
        int dummy = 0;
        if (ioctl(m_fd, IOCTL_UNLOCK, &dummy) < 0)
        {
            perror("IOCTL_UNLOCK failed");
        }
    }
};

// Union for mutex variants
struct MutexUnion
{
    std::mutex std_mux;
    pthread_mutex_t pi_mux;
    FifoMutex fifo_mux;
    RtBeMutex rtbe_mux;
    ~MutexUnion() {} // Destructor handled in main
};

// Args struct
struct Args
{
    int num_threads = 4;
    std::string cpuset = "0-3";
    std::string scheduling = "global";
    int threads_per_core = 1;
    long long budget = 0; // 0 = auto
    long long period = 0; // 0 = auto
    std::string mutex_type = "std";
    long long cs_work = 0; // 0 = auto calibrate
    int duration = 10;
    int test_case = 0;
};

// Argp parser
static char doc[] = "DL Mutex Benchmark Suite";
static char args_doc[] = "";
static struct argp_option options[] = {
    {"num-threads", 'n', "N", 0, "Number of threads", 0},
    {"cpuset", 'c', "CPUS", 0, "CPU list e.g. 0-3", 0},
    {"scheduling", 's', "MODE", 0, "global or partitioned", 0},
    {"threads-per-core", 't', "T", 0, "Threads per core for partitioned", 0},
    {"budget", 'b', "B", 0, "Runtime ns (0=auto)", 0},
    {"period", 'p', "P", 0, "Period=deadline ns (0=auto)", 0},
    {"mutex-type", 'm', "TYPE", 0, "std|pi|fifo|rtbe", 0},
    {"cs-work", 'w', "W", 0, "CS spin loops (0=auto calibrate to ~budget)", 0},
    {"duration", 'd', "S", 0, "Run duration seconds", 0},
    {"test-case", 'e', "C", 0, "Pre-configured test case (1-4, overrides others)", 0},
    {0}};

static int parse_opt(int key, char *arg, struct argp_state *state)
{
    Args *args = static_cast<Args *>(state->input);
    switch (key)
    {
    case 'n':
        args->num_threads = std::stoi(arg);
        break;
    case 'c':
        args->cpuset = arg;
        break;
    case 's':
        args->scheduling = arg;
        break;
    case 't':
        args->threads_per_core = std::stoi(arg);
        break;
    case 'b':
        args->budget = std::stoll(arg);
        break;
    case 'p':
        args->period = std::stoll(arg);
        break;
    case 'm':
        args->mutex_type = arg;
        break;
    case 'w':
        args->cs_work = std::stoll(arg);
        break;
    case 'd':
        args->duration = std::stoi(arg);
        break;
    case 'e':
        args->test_case = std::stoi(arg);
        break;
    case ARGP_KEY_END:
        if (args->scheduling != "global" && args->scheduling != "partitioned")
        {
            argp_error(state, "Invalid scheduling mode");
        }
        if (args->mutex_type != "std" && args->mutex_type != "pi" && args->mutex_type != "fifo" && args->mutex_type != "rtbe")
        {
            argp_error(state, "Invalid mutex type");
        }
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
// Function to set RT runtime to unlimited
void set_rt_runtime_unlimited()
{
    std::ofstream ofs("/proc/sys/kernel/sched_rt_runtime_us");
    if (ofs.is_open())
    {
        ofs << "-1";
        ofs.close();
    }
}

void set_rt_period()
{
    std::ofstream ofs("/proc/sys/kernel/sched_rt_period_us");
    if (ofs.is_open())
    {
        ofs << THREAD_PERIOD_US; //
        ofs.close();
    }
}

static struct argp argp = {options, parse_opt, args_doc, doc, 0, 0, 0};

bool is_cgroup_v2()
{
    struct statfs fs;
    if (statfs("/sys/fs/cgroup", &fs) != 0)
        return false;
    return fs.f_type == 0x63677270 && std::ifstream("/sys/fs/cgroup/cgroup.controllers").good(); // cgroup2 magic
}

int create_cgroup(const std::string &cpuset_str)
{

    if (is_cgroup_v2())
    {
        std::string path = "/sys/fs/cgroup/bench";
        if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST)
            return -1;
        printf("Using cgroup v2\n");
        std::ofstream cpus(path + "/cpuset.cpus");
        if (!cpus)
            return -1;
        cpus << cpuset_str;
    }
    else
    {
        printf("Using cgroup v1\n");
        std::string cpuset_path = "/sys/fs/cgroup/cpuset/bench";
        if (mkdir(cpuset_path.c_str(), 0755) != 0 && errno != EEXIST)
            return -1;
        std::ofstream cpus(cpuset_path + "/cpuset.cpus");
        if (!cpus)
            return -1;
        cpus << cpuset_str;
    }
    return 0;
}

int add_to_cgroup(pid_t pid)
{
    if (is_cgroup_v2())
    {
        std::ofstream procs("/sys/fs/cgroup/bench/cgroup.procs", std::ios::app);
        if (!procs)
            return -1;
        procs << pid;
    }
    else
    {
        std::ofstream tasks("/sys/fs/cgroup/cpuset/bench/tasks", std::ios::app);
        if (!tasks)
            return -1;
        tasks << pid;
    }
    return 0;
}

// Parse cpuset "0-3,5" to vector of CPUs
std::vector<int> parse_cpuset(const std::string &cpuset_str)
{
    std::vector<int> cpus;
    std::stringstream ss(cpuset_str);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        size_t dash = token.find('-');
        if (dash != std::string::npos)
        {
            int start = std::stoi(token.substr(0, dash));
            int end = std::stoi(token.substr(dash + 1));
            for (int i = start; i <= end; ++i)
                cpus.push_back(i);
        }
        else
        {
            cpus.push_back(std::stoi(token));
        }
    }
    return cpus;
}
// Global shutdown flag
std::atomic<bool> shutdown_requested{false};

// Signal handler
void sigint_handler(int sig)
{
    shutdown_requested = true;
    printf("SIGINT received, shutting down gracefully...\n");
}

void remove_cgroup(const std::string &path)
{
    if (is_cgroup_v2())
    {
        std::ofstream procs(path + "/cgroup.procs");
        if (procs)
            procs << "";
    }
    else
    {
        // Instead of writing empty, kill all tasks in the cgroup
        std::ifstream tasks_file(path + "/tasks");
        std::string line;
        while (std::getline(tasks_file, line))
        {
            pid_t pid = std::stoi(line);
            if (pid > 0)
            {
                kill(pid, SIGKILL);
            }
        }
    }

    DIR *dir = opendir(path.c_str());
    // if (dir)
    // {
    //     struct dirent *entry;
    //     // while ((entry = readdir(dir)) != nullptr)
    //     // {
    //     //     if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
    //     //         continue;
    //     //     //std::string subpath = path + "/" + entry->d_name;
    //     //     //remove_cgroup(subpath);
    //     // }
    //     closedir(dir);
    // }
    if (rmdir(path.c_str()) != 0)
    {
        perror(("Failed to remove " + path).c_str());
    }
    else
    {
        printf("Removed %s\n", path.c_str());
    }
}

int set_dl_attr(pid_t tid, long long runtime_ns, long long deadline_ns, long long period_ns)
{
    struct sched_attr attr = {};
    attr.size = sizeof(attr);
    attr.sched_flags = 0;
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = runtime_ns;
    attr.sched_deadline = deadline_ns;
    attr.sched_period = period_ns;
    return syscall(SYS_sched_setattr, tid, &attr, 0);
}

int set_affinity(pid_t tid, int cpu)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return sched_setaffinity(tid, sizeof(set), &set);
}

// Calibrate spin loops to approximate time_ns
long long calibrate_spin(long long time_ns)
{
    auto start = std::chrono::high_resolution_clock::now();
    volatile long long count = 0;
    while (std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::high_resolution_clock::now() - start)
               .count() < time_ns)
    {
        ++count;
    }
    if (time_ns == 0)
        return 0;
    return count * 1000000000LL / time_ns; // Loops per ns
}
const uint32_t thread_colors[] = {
    0xFFFF0000, // Red
    0xFF00FF00, // Green
    0xFF0000FF, // Blue
    0xFFFFFF00, // Yellow
    0xFFFF00FF, // Magenta
    0xFF00FFFF, // Cyan
    0xFF800000, // Maroon
    0xFF008000, // Dark Green
    0xFF000080, // Navy
    0xFF808000, // Olive
    0xFF800080, // Purple
    0xFF008080, // Teal
    0xFFA52A2A, // Brown
    0xFF808080, // Gray
    0xFFFFA500, // Orange
    0xFF4B0082  // Indigo
};

void *thread_func(void *arg)
{
    auto params = static_cast<std::tuple<int, MutexUnion *, std::string, long long, int, int, long long, long long> *>(arg);
    int id = std::get<0>(*params);
    MutexUnion *mux = std::get<1>(*params);
    std::string type = std::get<2>(*params);
    long long cs_work = std::get<3>(*params);
    int duration = std::get<4>(*params);
    int affinity_cpu = std::get<5>(*params);
    long long budget = std::get<6>(*params);
    long long period = std::get<7>(*params);
    printf("Params \n id=%d, type=%s, cs_work=%lld, duration=%d, affinity_cpu=%d, budget=%lld, period=%lld\n",
           id, type.c_str(), cs_work, duration, affinity_cpu, budget, period);
    pid_t tid = syscall(SYS_gettid);
    if (add_to_cgroup(tid) < 0)
    {
        std::cerr << "Failed to add thread " << id << " to cgroup\n";
    }
    if (affinity_cpu >= 0)
    {
        if (set_affinity(tid, affinity_cpu) < 0)
        {
            perror(("set_affinity failed for thread " + std::to_string(id)).c_str());
        }
    }
    long long offset = id * 100000LL; // Stagger by 0.1ms
    if (set_dl_attr(tid, budget, period + offset, period + offset) < 0)
    {
        perror(("set_dl_attr failed for thread " + std::to_string(id)).c_str());
    }
    printf("Thread %d (TID %d) started with budget %lld, period %lld, affinity CPU %d\n", id, tid, budget, period, affinity_cpu);

    std::string thread_name = "Thread " + std::to_string(id);
    pthread_setname_np(pthread_self(), thread_name.c_str());
    // nvtxNameOsThreadA(static_cast<uint32_t>(pthread_self()), thread_name.c_str());
    nvtxNameOsThreadA(static_cast<uint32_t>(tid), thread_name.c_str());
    char wait_msg[64];
    snprintf(wait_msg, sizeof(wait_msg), "Mutex Wait %s", thread_name.c_str());
    char cs_msg[64];
    snprintf(cs_msg, sizeof(cs_msg), "CS %s", thread_name.c_str());
    auto start = std::chrono::steady_clock::now();
    while (!shutdown_requested && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration)
    {

        // Mutex Wait range
        nvtxEventAttributes_t wait_attr = {0};
        wait_attr.version = NVTX_VERSION;
        wait_attr.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
        wait_attr.colorType = NVTX_COLOR_ARGB;
        wait_attr.color = thread_colors[id % (sizeof(thread_colors) / sizeof(uint32_t))];
        wait_attr.messageType = NVTX_MESSAGE_TYPE_ASCII;
        wait_attr.message.ascii = wait_msg;
        nvtxRangeId_t wait_range_id = nvtxRangeStartEx(&wait_attr);

        if (type == "std")
            mux->std_mux.lock();
        else if (type == "pi")
            pthread_mutex_lock(&mux->pi_mux);
        else if (type == "fifo")
            mux->fifo_mux.lock();
        else if (type == "rtbe")
            mux->rtbe_mux.lock();
        printf("Thread %d acquired mutex\n", id);
        nvtxRangeEnd(wait_range_id);

        // CS range
        nvtxEventAttributes_t cs_attr = {0};
        cs_attr.version = NVTX_VERSION;
        cs_attr.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
        cs_attr.colorType = NVTX_COLOR_ARGB;
        cs_attr.color = thread_colors[id % (sizeof(thread_colors) / sizeof(uint32_t))];
        cs_attr.messageType = NVTX_MESSAGE_TYPE_ASCII;
        cs_attr.message.ascii = cs_msg;
        nvtxRangeId_t cs_range_id = nvtxRangeStartEx(&cs_attr);
        volatile long long sum = 0;
        for (long long i = 0; i < cs_work; ++i)
            sum += i;

        if (type == "std")
            mux->std_mux.unlock();
        else if (type == "pi")
            pthread_mutex_unlock(&mux->pi_mux);
        else if (type == "fifo")
            mux->fifo_mux.unlock();
        else if (type == "rtbe")
            mux->rtbe_mux.unlock();
        nvtxRangeEnd(cs_range_id);

        printf("Thread %d released mutex, CS sum = %lld\n", id, sum);
    }
    printf("Thread %d shutting down\n", id);
    delete params;
    return nullptr;
}

int main(int argc, char **argv)
{
    nvtxInitialize(nullptr);
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    set_rt_runtime_unlimited();
    set_rt_period();
    Args args;
    argp_parse(&argp, argc, argv, 0, 0, &args);

    // Handle test_case overrides
    if (args.test_case == 1)
    {
        // Std::mutex depletion
        args.num_threads = 3;
        args.cpuset = "5-6";
        args.scheduling = "partitioned";
        args.threads_per_core = 2;
        args.budget = 1000000;
        args.period = 10000000;
        args.mutex_type = "std";
        args.cs_work = 0; // Auto, will be ~1.5x budget
    }
    else if (args.test_case == 2)
    {
        // PI -- PCP
        args.num_threads = 4;
        args.cpuset = "3-6";
        args.scheduling = "global";
        args.budget = 2000000;
        args.period = 20000000;
        args.mutex_type = "pi";
        args.cs_work = 0;
    }
    else if (args.test_case == 3)
    {
        // FIFO blocked by throttled
        args.num_threads = 5;
        args.cpuset = "3-7";
        args.scheduling = "partitioned";
        args.threads_per_core = 1;
        args.budget = 500000;
        args.period = 5000000;
        args.mutex_type = "fifo";
        args.cs_work = 100000;
    }
    else if (args.test_case == 4)
    {
        // RT-BE avoids issues
        args.num_threads = 3;
        args.cpuset = "5-6";
        args.scheduling = "partitioned";
        args.threads_per_core = 2;
        args.budget = 1000000;
        args.period = 10000000;
        args.mutex_type = "rtbe";
        args.cs_work = 0;
    }

    // Parse cpuset
    auto cpu_list = parse_cpuset(args.cpuset);
    int num_cores = cpu_list.size();
    if (num_cores == 0)
    {
        std::cerr << "Invalid cpuset\n";
        return 1;
    }

    // Auto budget/period
    if (args.budget == 0)
        args.budget = 1000000LL; // 1ms default
    double target_util = 0.95;
    if (args.period == 0)
    {
        if (args.scheduling == "global")
        {
            args.period = static_cast<long long>(args.budget * args.num_threads / (target_util * num_cores));
        }
        else
        {
            args.period = static_cast<long long>(args.budget * args.threads_per_core / target_util);
        }
        args.period = std::max(args.period, args.budget); // Ensure period >= budget
    }

    // Utilization check (warn only)
    double util_per_thread = static_cast<double>(args.budget) / args.period;
    double total_util = util_per_thread * args.num_threads;
    double max_util = (args.scheduling == "global" ? num_cores : num_cores) * target_util;
    if (total_util > max_util)
    {
        std::cerr << "Warning: Potential overload: util=" << total_util << " > " << max_util << std::endl;
    }

    if (args.cs_work == 0)
    {
        long long calibration_time_ns = 100000000LL; // 100ms
        uint64_t calibration_count = calibrate_spin(calibration_time_ns);
        long long desired_cs_time_ns = args.budget * 2; // Target ~2x budget time
        // Correct scaling to avoid overflow: use double for precision
        double loops_per_ns = static_cast<double>(calibration_count) / calibration_time_ns;
        args.cs_work = static_cast<long long>(loops_per_ns * desired_cs_time_ns);
        // Safety cap to prevent infinite loops (e.g., if calibration fails)
        if (args.cs_work > 10000000000LL)
        { // Arbitrary max ~1e9 loops
            args.cs_work = 1000000000LL;
            std::cerr << "Warning: Capped cs_work to prevent excessive loop time\n";
        }
        printf("Calibrated cs_work = %lld loops for ~%lld ns\n", args.cs_work, desired_cs_time_ns);
    }

    // Create cgroup
    if (create_cgroup(args.cpuset) < 0)
    {
        perror("create_cgroup failed");
        return 1;
    }

    // Init mutex
    MutexUnion mux;
    if (args.mutex_type == "pi")
    {
        pthread_mutexattr_t attr;
        if (pthread_mutexattr_init(&attr) != 0 ||
            pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_PROTECT) != 0 ||
            pthread_mutex_init(&mux.pi_mux, &attr) != 0)
        {
            std::cerr << "PI mutex init failed\n";
            return 1;
        }
        pthread_mutexattr_destroy(&attr);
    }
    else if (args.mutex_type == "rtbe")
    {
        new (&mux.rtbe_mux) RtBeMutex();
    } // std and fifo default constructed

    // Compute affinity
    std::vector<int> affinity_cpus(args.num_threads, -1);
    if (args.scheduling == "partitioned")
    {
        for (int i = 0; i < args.num_threads; ++i)
        {
            int group = i / args.threads_per_core;
            int core = group % num_cores;
            affinity_cpus[i] = cpu_list[core];
        }
    }
    printf("Creating threads \n");
    // Create threads
    std::vector<pthread_t> threads(args.num_threads);
    for (int i = 0; i < args.num_threads; ++i)
    {
        auto *params = new std::tuple<int, MutexUnion *, std::string, long long, int, int, long long, long long>(
            i, &mux, args.mutex_type, args.cs_work, args.duration, affinity_cpus[i], args.budget, args.period);
        if (pthread_create(&threads[i], nullptr, thread_func, params) != 0)
        {
            perror("pthread_create failed");
            delete params;
            return 1;
        }
    }
    printf("Threads created\n");
    // Join threads
    for (auto &th : threads)
    {
        if (pthread_join(th, nullptr) != 0)
        {
            perror("pthread_join failed");
        }
    }

    // Destroy mutex
    if (args.mutex_type == "pi")
    {
        pthread_mutex_destroy(&mux.pi_mux);
    }
    else if (args.mutex_type == "rtbe")
    {
        mux.rtbe_mux.~RtBeMutex();
    }
    std::string cgroup_path = is_cgroup_v2() ? "/sys/fs/cgroup/bench" : "/sys/fs/cgroup/cpuset/bench";
    remove_cgroup(cgroup_path);

    return 0;
}