#ifndef THREAD_CPP
#define THREAD_CPP

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* See feature_test_macros(7) */
#endif

#include <thread.hpp>
#include <stdio.h>
#define sched_setattr(pid, attr, flags) syscall(__NR_sched_setattr, pid, attr, flags)
#define sched_getattr(pid, attr, size, flags) syscall(__NR_sched_getattr, pid, attr, size, flags)

void create_cgroup(const std::string &cgroup_name, const std::string &cpus)
{
    std::ofstream cg_file;
    std::string cgroup_path = "/sys/fs/cgroup/cpuset/" + cgroup_name;

    // Create cgroup directory
    std::system(("sudo mkdir -p " + cgroup_path).c_str());

    // Write the list of allowed CPUs to cpuset.cpus
    cg_file.open(cgroup_path + "/cpuset.cpus");
    if (!cg_file.is_open())
    {
        std::cerr << "Failed to open cgroup file for cpuset.cpus" << std::endl;
        return;
    }
    cg_file << cpus; // Specify the CPU(s) to bind the cgroup to (e.g., "0" for core 0, "0-3" for cores 0 to 3)
    cg_file.close();

    // // Enable memory/memory nodes to make cpuset controller valid
    cg_file.open(cgroup_path + "/cpuset.mems");
    if (!cg_file.is_open())
    {
        std::cerr << "Failed to open cgroup file for cpuset.mems" << std::endl;
        return;
    }
    cg_file << "0"; // By default, assign to memory node 0
    cg_file.close();

    cg_file.open(cgroup_path + "/cpuset.cpu_exclusive");
    if (!cg_file.is_open())
    {
        std::cerr << "Failed to open cgroup file for cpu_exclusive" << std::endl;
        return;
    }
    cg_file << "0"; // By default, do not use exclusive CPUs
    cg_file.close();

    std::cout << "Cgroup " << cgroup_name << " created and bound to CPU(s): " << cpus << std::endl;
}

// Function to add a process/thread to a cgroup
void add_thread_to_cgroup(const std::string &cgroup_name, pid_t tid)
{
    std::ofstream cg_file;
    // std::string cgroup_tasks_path = "/sys/fs/cgroup/cpuset/" + cgroup_name + "/cgroup.procs";
    std::string cgroup_tasks_path = "/sys/fs/cgroup/cpuset/" + cgroup_name + "/tasks";
    // Add thread to the cgroup by writing to cgroup.procs
    cg_file.open(cgroup_tasks_path);
    if (!cg_file.is_open())
    {
        std::cerr << "Failed to open cgroup tasks file" << std::endl;
        return;
    }
    cg_file << tid;
    cg_file.close();

    std::cout << "Thread with TID " << tid << " added to cgroup " << cgroup_name << std::endl;
}

executor_thread::executor_thread(ordered_mutex *wait_mutex, executor *exec, int logical_thread_id)
    : wait_mutex_(wait_mutex), exec(exec), logical_thread_id(logical_thread_id)
{
}


executor_thread::~executor_thread()
{
    // t->join();
}

void executor_thread::set_budget(int budget)
{
    this->budget = budget;
}

int executor_thread::get_budget()
{
    return budget;
}

/*
void executor_thread::set_global_queue_mutex(FutexMutex *global_queue_mutex)
{
    this->global_queue_mutex = global_queue_mutex;
}
void executor_thread::spin()
{
    this->threadID = syscall(SYS_gettid);
    CPU_ZERO(&cpuSet);
    sched_getaffinity(threadID, sizeof(cpu_set_t), &cpuSet);
    policy = sched_getscheduler(threadID);

    while (this->exec->running)
    {
        // Lock the waitset mutex before accessing the waitsets
        this->global_waitset_mutex->lock();

        if (this->exec->global_waitset.empty())
        {
            this->global_waitset_mutex->unlock(); // Unlock before calling update
            this->exec->update_waitset();         // Populate the waitset
        }
        else
        {
            bool callback_executed = false;

            // Temporary container for callbacks that aren't executed
            std::vector<std::pair<std::shared_ptr<Callback>, int>> temp_queue;

            // Loop through the priority queue until we find a suitable callback or the queue is empty
            while (!this->exec->global_waitset.empty())
            {
                // Get the top callback from the priority queue
                auto callback = this->exec->global_waitset.top();
                this->exec->global_waitset.pop(); // Remove it temporarily for checking
                auto temp_result = this->callbacks.find(callback.first->getUUID());
                // Check if the current callback belongs to the current thread's callbacks
                if (temp_result != this->callbacks.end())
                {
                    // Re-add all the callbacks that were not executed back to the priority queue
                    for (const auto &cb : temp_queue)
                    {
                        this->exec->global_waitset.push(cb);
                    }
                    // Unlock the mutex before executing the callback to avoid deadlocks
                    this->global_waitset_mutex->unlock();

                    // Execute the callback
                    this->exec->execute_and_time(callback.first, callback.second);

                    callback_executed = true;
                    break;
                }
                else
                {
                    // If the callback doesn't belong to this thread, store it temporarily
                    temp_queue.push_back(callback);
                }
            }

            // If no callback was executed, unlock the mutex and update the waitset
            if (!callback_executed)
            {
                for (const auto &cb : temp_queue)
                {
                    this->exec->global_waitset.push(cb);
                }
                this->global_waitset_mutex->unlock();
                this->exec->update_waitset(); // Repopulate the waitset
            }
        }
    }
}
*/

// void executor_thread::spin()
// {
//     this->threadID = syscall(SYS_gettid);
//     // this->threadID = gettid();
//     CPU_ZERO(&cpuSet);
//     sched_getaffinity(threadID, sizeof(cpu_set_t), &cpuSet);
//     policy = sched_getscheduler(threadID);
//     while (this->exec->running)
//     {
//         // Lock the waitset mutex before accessing the waitsets
//         this->global_waitset_mutex->lock();

//         if (this->exec->partitioned)
//         {
//             // Partitioned case: handle global_partitioned_waitset
//             if (this->exec->global_partitioned_waitset.empty())
//             {
//                 this->global_waitset_mutex->unlock();     // Unlock before calling update
//                 this->exec->update_waitset_partitioned(); // Populate the partitioned waitset
//             }
//             else
//             {
//                 bool callback_executed = false;

//                 // Iterate over the global_partitioned_waitset to find a callback to execute
//                 for (auto it = this->exec->global_partitioned_waitset.begin(); it != this->exec->global_partitioned_waitset.end(); ++it)
//                 {
//                     auto &callback = *it;

//                     // Check if the current callback belongs to the current thread's callbacks
//                     if (this->callbacks.find(callback.first->getUUID()) != this->callbacks.end())
//                     {
//                         // Remove the callback from the partitioned waitset
//                         this->exec->global_partitioned_waitset.erase(it);

//                         // Unlock the mutex before executing the callback to avoid deadlocks
//                         this->global_waitset_mutex->unlock();

//                         // Execute the callback
//                         this->exec->execute_and_time(callback.first, callback.second);

//                         callback_executed = true;
//                         break;
//                     }
//                 }

//                 // If no callback was executed, unlock the mutex and update the partitioned waitset
//                 if (!callback_executed)
//                 {
//                     this->global_waitset_mutex->unlock();
//                     this->exec->update_waitset_partitioned(); // Populate the partitioned waitset
//                 }
//             }
//         }
//         else
//         {
//             // Non-partitioned case: handle global_waitset
//             if (this->exec->global_waitset.empty())
//             {
//                 this->global_waitset_mutex->unlock();
//                 this->exec->update_waitset();
//             }
//             else
//             {
//                 std::pair<std::shared_ptr<Callback>, int> callback = this->exec->global_waitset.top();
//                 this->exec->global_waitset.pop();
//                 this->global_waitset_mutex->unlock();

//                 // Execute the callback
//                 this->exec->execute_and_time(callback.first, callback.second);
//             }
//         }
//     }
// }

void executor_thread::assign_thread_ptr(std::shared_ptr<std::thread> t)
{
    this->t = t;
}

void executor_thread::set_priority(int priority)
{
    this->priority = priority;
    param.sched_priority = priority;
    sched_setscheduler(threadID, SCHED_FIFO, &param);
}
void executor_thread::set_policy(int policy)
{
    this->policy = policy;
    sched_setscheduler(threadID, policy, &param);
}

int get_first_cpu_from_set(const cpu_set_t *cpu_set)
{
    // Iterate over all possible CPUs, depending on the size of the CPU set
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu)
    {
        if (CPU_ISSET(cpu, cpu_set))
        {
            return cpu; // Return the first CPU found
        }
    }
    return -1; // If no CPUs are set, return -1
}

void executor_thread::set_affinity(cpu_set_t cpuSet, bool rt)
{
    // if (CPU_EQUAL(&cpuSet, &this->cpuSet))
    // {
    //     return;
    // }
    this->cpuSet = cpuSet;
    // sched_setaffinity(threadID, sizeof(cpu_set_t), &cpuSet);
    std::stringstream thread_name_stream;
    std::stringstream cpu_core_stream;

    // Format the thread name and CPU core using stringstream
    if (rt)
    {
        thread_name_stream << "RT_Thread_" << get_first_cpu_from_set(&cpuSet);
    }
    else
    {
        thread_name_stream << "BE_Thread_" << get_first_cpu_from_set(&cpuSet);
    }
    cpu_core_stream << get_first_cpu_from_set(&cpuSet);

    // Convert stringstream to string and call create_cgroup with the results

    // if sched deadline policy, set affinity using cgroups
    if (policy == SCHED_DEADLINE)
    {
        std::string thread_name = thread_name_stream.str();
        std::string cpu_core = cpu_core_stream.str();
        // Create cgroup for the thread
        create_cgroup(thread_name.c_str(), cpu_core_stream.str().c_str());
        // Add thread to cgroup
        add_thread_to_cgroup(thread_name.c_str(), threadID);
    }
}
void executor_thread::set_rt(bool rt)
{
    this->rt = rt;
}
bool executor_thread::get_rt()
{
    return rt;
}

void executor_thread::add_callback(std::shared_ptr<Callback> callback)
{
    //callbacks.insert(std::make_pair(callback->getUUID(), callback));
    callbacks.try_emplace(callback->getUUID(), callback);
}

void executor_thread::add_chain_to_thread(std::shared_ptr<Chain> chain)
{
    for (auto &callback : chain->getCallbacks())
    {
        // compute the utilization of the callback
        // this->callbacks.emplace(std::make_pair(callback->getUUID(), callback));
        std::shared_ptr<Callback> real_callback_shared_ptr = callback->getChain()->getCallback(callback->getUUID());
        //this->callbacks.insert(std::make_pair(callback->getUUID(), real_callback_shared_ptr));
        this->add_callback(real_callback_shared_ptr);
    }
}
void executor_thread::remove_callback(boost::uuids::uuid uuid)
{
    callbacks.erase(uuid);
}
void executor_thread::getusage()
{
    getrusage(who, &usage);
}
void executor_thread::set_threadID(int threadID)
{
    this->threadID = threadID;
}
pid_t executor_thread::get_threadID()
{
    return threadID;
}
cpu_set_t *executor_thread::get_cpuSet()
{
    return &cpuSet;
}
int executor_thread::get_priority()
{
    return priority;
}
int executor_thread::get_policy()
{
    return policy;
}
struct sched_param executor_thread::get_param()
{
    return param;
}
std::shared_ptr<std::thread> executor_thread::get_thread()
{
    return t;
}
struct rusage executor_thread::get_usage()
{
    return usage;
}
std::unordered_map<boost::uuids::uuid, std::shared_ptr<Callback>, boost::hash<boost::uuids::uuid>>& executor_thread::get_callbacks()
{
    return callbacks;
}
int executor_thread::set_sched_deadline(struct sched_attr attr, unsigned int flags)
{
    struct sched_attr attr_2 = attr;
    (void)flags;
    if (!sched_getattr(threadID, &attr, sizeof(attr), 0))
    {
        std::cout << "Current Thread Parameters: " << std::endl;
        std::cout << "Size: " << attr.size << std::endl;
        std::cout << "Policy: " << attr.sched_policy << std::endl;
        std::cout << "Flags: " << attr.sched_flags << std::endl;
        std::cout << "Nice: " << attr.sched_nice << std::endl;
        std::cout << "Priority: " << attr.sched_priority << std::endl;
        std::cout << "Runtime: " << attr.sched_runtime << std::endl;
        std::cout << "Period: " << attr.sched_period << std::endl;
        std::cout << "Deadline: " << attr.sched_deadline << std::endl;
        std::cout << "Sched Util Min: " << attr.sched_util_min << std::endl;
        std::cout << "Sched Util Max: " << attr.sched_util_max << std::endl;
    }
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = attr_2.sched_runtime;
    attr.sched_period = attr_2.sched_period;
    attr.sched_deadline = attr_2.sched_deadline;
    attr.sched_flags = 0 | SCHED_FLAG_RECLAIM;
    attr.sched_nice = 0;
    attr.sched_priority = 0;
    attr.sched_util_min = 0;
    attr.sched_util_max = 1024;

    attr_2 = attr;
    std::cout << "Setting deadline for thread " << threadID << " to " << attr.sched_policy << " / " << attr.sched_runtime << " / " << attr.sched_period << " / " << attr.sched_deadline << std::endl;
    int ret = sched_setattr(threadID, &attr, 0);
    if (ret == -1)
    {
        std::cout << "Error setting deadline for thread " << threadID << " with error code " << strerror(errno) << std::endl;
    }
    return ret;
    // pthread_t native_handle = t->native_handle();
}

#endif // THREAD_CPP
