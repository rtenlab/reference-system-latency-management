#pragma once

#ifndef EXECUTOR_THREAD_HPP
#define EXECUTOR_THREAD_HPP
class Callback;
class Chain;
class executor;

#include <thread>
//#include <ftxmtx.hpp>
#include <executor.hpp>
#include <chain.hpp>

#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <callback.hpp>
#include <boost/uuid/uuid.hpp>
#include <unordered_map>
#include <memory>
#include <boost/uuid/uuid_hash.hpp>

#include <linux/sched.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/picas.hpp"

class executor_thread
{
public:
    //executor_thread(FutexMutex* global_waitset_mutex, executor* exec);
    executor_thread(ordered_mutex *wait_mutex, executor *exec, int logical_thread_id);
    executor_thread(executor* exec);
    ~executor_thread();
    void set_priority(int priority);
    void set_policy(int policy);
    void set_affinity(cpu_set_t cpuSet, bool rt);
    void add_callback(std::shared_ptr<Callback> callback);
    void add_chain_to_thread(std::shared_ptr<Chain> chain);

    void remove_callback(boost::uuids::uuid uuid);
    void getusage();
    void set_threadID(int threadID);
    pid_t get_threadID();
    cpu_set_t *get_cpuSet();
    int get_priority();
    int get_policy();
    struct sched_param get_param();
    void assign_thread_ptr(std::shared_ptr<std::thread> t);
    std::shared_ptr<std::thread> get_thread();
    struct rusage get_usage();
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<Callback>, boost::hash<boost::uuids::uuid>>& get_callbacks();
    //void spin();
    void set_budget(int budget);
    void set_rt(bool rt);
    bool get_rt();
    int get_budget();
    //void set_global_queue_mutex(ordered_mutex* global_queue_mutex);
    int set_sched_deadline(struct sched_attr attr, unsigned int flags);

    int logical_thread_id;

private:
    //FutexMutex *global_waitset_mutex, *global_queue_mutex;
    ordered_mutex *wait_mutex_;
    executor* exec;
    int who = RUSAGE_THREAD;
    pid_t threadID;
    cpu_set_t cpuSet;
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<Callback>, boost::hash<boost::uuids::uuid>> callbacks;
    int priority;
    int policy;
    int budget;
    bool rt;
    struct rusage usage;
    struct sched_param param;
    std::shared_ptr<std::thread> t;
    friend class executor;
};

#endif // EXECUTOR_THREAD_HPP
