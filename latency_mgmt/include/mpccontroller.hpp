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
    double MLP(double, std::vector<std::shared_ptr<Chain>>, int, double, State);
    void create_threadclass(void);
    double compute_chain_utilization(std::shared_ptr<Chain> chain, State current_state);
    void control_budget(State &current_state, State &next_state);
    void reduce_rt_budget(std::shared_ptr<threadclass> tc, const State& current_state);

    std::vector<struct timeval> pwa_cd(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget, State current_state);
    std::vector<struct timeval> hoora_analysis(State* current_state);
private:
    executor *exec;
    State current_state;
    void apply_next_state(const State &next_state);
    struct timeval predict_response_time(uint64_t new_budget, timeval current_response_time, uint64_t current_budget);
    void optimize_sched_runtime(State &current_state, State &next_state);
    std::map<State, std::vector<std::deque<struct timeval>>> state_associated_execution_times;
    std::chrono::milliseconds period = std::chrono::milliseconds(8000);
    std::vector<std::shared_ptr<threadclass>> threadclasses;
};

#endif // MPC_CONTROLLER_H
