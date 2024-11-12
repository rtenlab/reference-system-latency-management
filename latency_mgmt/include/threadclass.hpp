#pragma once
#ifndef THREADCLASS_HPP
#define THREADCLASS_HPP
class MPCController;
class executor_thread;
class Chain;
class Callback;
class State;

#include <iostream>
#include <thread.hpp>
#include <callback.hpp>
#include <chain.hpp>
#include <state.hpp>
#include <queue>
#include <vector>
#include <mpccontroller.hpp>
class threadclass
{
    public:
        threadclass();
        threadclass(bool rt_threadclass);
        threadclass(bool rt_threadclass, int id);
        void merge_threadclasss(std::shared_ptr<threadclass> tg1, std::shared_ptr<threadclass> tg2);
        void add_thread(std::shared_ptr<executor_thread> thread);
        void remove_thread(std::shared_ptr<executor_thread> thread);
        void set_utilization(double utilization);
        double get_utilization();
        void apply_budgets(int budget_us);
        int assign_chain(std::shared_ptr<Chain> chain);
        void add_chain(std::shared_ptr<Chain> chain);
        void remove_chain(std::shared_ptr<Chain> chain);
        std::vector<std::shared_ptr<Chain>> get_chains();
        void set_period(int period);
        int get_period();
    private:
        double utilization;
        int num_threads;
        bool rt_threadclass;
        uint64_t total_budget;
        int thread_period;
        std::vector<std::shared_ptr<executor_thread>> threads;
        std::vector<std::shared_ptr<Chain>> chains;
        std::vector<std::shared_ptr<Callback>> callbacks;
        friend class MPCController;
        int id;
};



#endif // THREADCLASS_HPP