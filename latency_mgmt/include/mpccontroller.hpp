#ifndef MPC_CONTROLLER_HPP
#define MPC_CONTROLLER_HPP
#pragma once
class executor;
class threadclass;
#include <mpccontroller.hpp>
#include <threadclass.hpp>
#include <executor.hpp>
#include <state.hpp>
#include <thread.hpp>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <state.hpp>
#include <sys/syscall.h>
#include <unistd.h>
#include <stdio.h>
// struct threadclass{
//     std::vector<std::shared_ptr<Chain>> chainset;
//     bool RT_TG;
//     std::vector<std::shared_ptr<executor_thread>> threads;
//     std::vector<struct timeval > latency_targets;
// };

class MPCController
{
public:
    MPCController();
    ~MPCController();
    void assign_executor(executor *exec);
    void run();
    void chain_test();
    double MLP(double, std::vector<std::shared_ptr<Chain>>, int, double);
    void create_threadclass(void);
    double compute_chain_utilization(std::shared_ptr<Chain> chain);
    bool reduce_rt_budget(std::shared_ptr<threadclass> tc, unsigned int *analysis_count, int min_budget);
    void reallocate_chains(unsigned int *analysis_count);
    void update_tc_utilization(std::shared_ptr<threadclass> tc);
    void reallocate_be_chains();
    timeval do_partial_ad_analysis(std::vector<std::shared_ptr<Chain>>, std::shared_ptr<threadclass>, int, std::shared_ptr<Chain>, std::shared_ptr<Callback>);
    timeval do_partial_cd_analysis(std::vector<std::shared_ptr<Chain>>, std::shared_ptr<threadclass>, int, std::shared_ptr<Chain>, std::shared_ptr<Callback>);
    void verify_starvation_freedom(std::shared_ptr<threadclass> be_tc);
    std::vector<struct timeval> pwa_cd(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget);
    std::vector<struct timeval> pwa_ad(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget);
    void worker_thread_run(std::atomic<bool> &complete);

private:
    executor *exec;
    State current_state;
    std::map<State, std::vector<std::deque<struct timeval>>> state_associated_execution_times;
    std::chrono::milliseconds period = std::chrono::milliseconds(8000);
    std::vector<std::shared_ptr<threadclass>> threadclasses;
    // condition variable for alerting the mpc controller of a timing violation
    std::shared_ptr<std::condition_variable> cv_ptr;
    std::shared_ptr<std::mutex> mtx_ptr;
    std::vector<std::shared_ptr<std::thread>> analysis_threadpool;
    std::deque<std::shared_ptr<threadclass>> work_queue;
    std::condition_variable work_cv;
    std::mutex work_mtx;
    std::vector<std::unique_ptr<std::atomic<bool>>> thread_complete;
};

#endif // MPC_CONTROLLER_H
