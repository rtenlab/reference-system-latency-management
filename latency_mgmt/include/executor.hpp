#pragma once

#ifndef EXECUTOR_HPP
#define EXECUTOR_HPP
class Callback;
class State;
class Chain;
class executor_thread;
struct callback_entry;

#include <thread.hpp>
#include <chain.hpp>
#include <callback.hpp>
#include <state.hpp>
#include <queue>
#include <csignal>
#include <boost/lockfree/queue.hpp>

#include "rclcpp/rclcpp.hpp"

// In executor.hpp
struct CompareCallback {
    bool operator()(const std::pair<std::shared_ptr<Callback>, int>& a,
                    const std::pair<std::shared_ptr<Callback>, int>& b) const;
};


class executor : public rclcpp::executors::MultiThreadedExecutor
{
public:
    //executor();
    virtual ~executor();
    explicit executor(int num_threads);
    //executor(int num_threads, bool partitioned);
    void execute_and_time(rclcpp::AnyExecutable &any_exec);
    void add_thread(std::shared_ptr<executor_thread> thread);
    void remove_thread(int threadID);
    void set_thread_priority(int threadID, int priority);
    void set_thread_policy(int threadID, int policy);
    void set_thread_affinity(int threadID, cpu_set_t cpuSet, bool rt);
    void add_callback_to_thread(int threadID, std::shared_ptr<Callback> callback);
    void remove_callback_from_thread(int threadID, std::shared_ptr<Callback> callback);
    void get_thread_usage(int threadID);
    void set_thread_budget(int threadID, int budget);
    void add_next_callback_to_global_waitset_queue(std::shared_ptr<Callback> callback, int chain_instance_id);
    void add_all_callbacks_to_all_threads();
    void remove_all_callbacks_from_all_threads();
    void remove_all_callbacks_from_thread(int threadID);
    void add_chain(std::shared_ptr<Chain> chain);
    void remove_chain(std::shared_ptr<Chain> chain);
    void add_callback_to_chain(int chainID, std::shared_ptr<Callback> callback);
    void remove_callback_from_chain(int chainID, boost::uuids::uuid callbackUUID);
    void update_waitset();
    void update_waitset_partitioned();
    void remove_callback_from_global_waitset_and_queue(std::shared_ptr<Callback> callback);
    void print_threads();
    static void handle_sigint(int signum);
    static void register_instance(executor *instance);
    static void unregister_instance(executor *instance);
    void stop();
    void start();
    void pause();
    void join();
    void set_partitioned(bool partitioned);
    bool get_partitioned();
    void make_threads(int num_threads);
    void run(std::shared_ptr<executor_thread> thread);
    void spin() {} // make_threads() does this job
    void schedule_timer_callback(std::shared_ptr<Callback> callback);
    void print_chains();
    void print_callbacks();
    void enable_priority_scheduling();
    void disable_priority_scheduling();
    bool is_priority_scheduling_enabled();
    bool is_running();
    void print_waitset();
    void print_waitset_queue();
    void set_callback_priorities();
    void print_chains_and_callbacks();
    void add_chain_to_thread(std::shared_ptr<Chain> chain, std::shared_ptr<executor_thread> thread);
    std::vector<std::vector<std::shared_ptr<Chain>>> parse_and_sort_chains(std::vector<std::shared_ptr<Chain>> *chains);
    std::vector<std::shared_ptr<executor_thread>> get_threads();
    std::vector<std::shared_ptr<Chain>>* get_chains();
    State get_state();
    std::shared_ptr<executor_thread> get_thread(int threadID);
    void apply_callback_to_thread_assignment();
    void assign_cv(std::shared_ptr<std::condition_variable> cv_ptr, std::shared_ptr<std::mutex> mtx_ptr);

private:
    std::shared_ptr<std::condition_variable> cv_ptr;
    std::shared_ptr<std::mutex> mtx_ptr;

    bool priority_scheduling = false;
    static std::vector<executor *> instances; // Track all instances
    std::vector<std::shared_ptr<executor_thread>> threads;
    std::vector<std::shared_ptr<Chain>> chains;
    std::deque<callback_entry> global_waitset_queue;
    //FutexMutex global_waitset_mutex, registration_mutex, partitioned_mutex, global_queue_mutex;
    std::priority_queue<std::pair<std::shared_ptr<Callback>, int>, std::vector<std::pair<std::shared_ptr<Callback>, int>>, CompareCallback> global_waitset;
    std::set<std::pair<std::shared_ptr<Callback>, int>> global_partitioned_waitset;
    std::atomic<int> global_sequence_number{0};
    int callback_count = 0;
    bool partitioned = false;
    std::atomic<bool> running{true};
    std::vector<std::shared_ptr<std::thread>> raw_threads;
    std::vector<std::shared_ptr<Chain>> sorted_rt_chains;
    std::vector<std::shared_ptr<Chain>> sorted_be_chains;
    friend class executor_thread;
};

#endif
