#ifndef STATE_HPP
#define STATE_HPP
#define GNU_SOURCE
#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <executor.hpp>
// #include <mpccontroller.hpp>
#include <chain.hpp>
#include <callback.hpp>
// class Chain;
class MPCController;

class State
{
public:
    static constexpr double weight_num_threads = 100.0;
    static constexpr double weight_thread_prio = 1.0;
    static constexpr double weight_policy = 10.0;
    static constexpr double weight_deadline_switch = 50.0;
    static constexpr double weight_cpu_set = 100.0;
    static constexpr double weight_deadline_budget = 5.0;
    static constexpr double weight_partitioned_callbacks = 75.0;
    static constexpr double weight_partitioned = 50.0;
    static constexpr double weight_prioritized = 20.0;
    static constexpr double weight_latency_target = 5.0;
    static constexpr double weight_best_effort_latency = 1.0;
    static constexpr double weight_best_effort = 10.0;
    static constexpr double weight_large_difference = 1000.0; // For significant differences, like different partitioned sets

    double distance(State &other);
    int hamming_distance(cpu_set_t &set1, cpu_set_t &set2);
    double euclidean_distance(const struct timeval &t1, const struct timeval &t2);
    bool is_deadline_switch(int policy1, int policy2);
    int boolean_vector_distance(const std::vector<bool> &v1, const std::vector<bool> &v2);
    double compare_partitioned_callbacks(const std::vector<std::vector<std::shared_ptr<Callback>>> &callbacks1,
                                         const std::vector<std::vector<std::shared_ptr<Callback>>> &callbacks2);
    int boolean_distance(const bool &b1, const bool &b2);
    bool operator<(const State &other) const
    {
        // Compare num_threads
        if (num_threads != other.num_threads)
        {
            return num_threads < other.num_threads;
        }

        // Compare thread_prio vectors lexicographically
        if (thread_prio != other.thread_prio)
        {
            return thread_prio < other.thread_prio;
        }

        // Compare thread_policy vectors lexicographically
        if (thread_policy != other.thread_policy)
        {
            return thread_policy < other.thread_policy;
        }

        // Compare sched_deadline_budget vectors lexicographically
        if (sched_deadline_budget != other.sched_deadline_budget)
        {
            return sched_deadline_budget < other.sched_deadline_budget;
        }

        // Compare thread_cpu_set vector lexicographically (by converting to arrays or using custom comparison)
        for (size_t i = 0; i < thread_cpu_set.size(); ++i)
        {
            if (CPU_EQUAL(&thread_cpu_set[i], &other.thread_cpu_set[i]) == 0)
            {
                return CPU_COUNT(&thread_cpu_set[i]) < CPU_COUNT(&other.thread_cpu_set[i]);
            }
        }

        // Compare partitioned
        if (partitioned != other.partitioned)
        {
            return partitioned < other.partitioned;
        }

        // Compare prioritized
        if (prioritized != other.prioritized)
        {
            return prioritized < other.prioritized;
        }

        // Compare chain_latency_targets vector lexicographically
        for (size_t i = 0; i < chain_latency_targets.size(); ++i)
        {
            if (chain_latency_targets[i].tv_sec != other.chain_latency_targets[i].tv_sec)
            {
                return chain_latency_targets[i].tv_sec < other.chain_latency_targets[i].tv_sec;
            }
            if (chain_latency_targets[i].tv_usec != other.chain_latency_targets[i].tv_usec)
            {
                return chain_latency_targets[i].tv_usec < other.chain_latency_targets[i].tv_usec;
            }
        }

        // Compare best_effort vectors lexicographically
        if (best_effort != other.best_effort)
        {
            return best_effort < other.best_effort;
        }

        // If all fields are equal, return false
        return false;
    }

    bool operator!=(const State &other) const
    {
        if(num_threads != other.num_threads)
        {
            return true;
        }
        if(thread_prio != other.thread_prio)
        {
            return true;
        }
        if(thread_policy != other.thread_policy)
        {
            return true;
        }
        if(sched_deadline_budget != other.sched_deadline_budget)
        {
            return true;
        }
        for(size_t i = 0; i < thread_cpu_set.size(); ++i)
        {
            if(CPU_EQUAL(&thread_cpu_set[i], &other.thread_cpu_set[i]) == 0)
            {
                return true;
            }
        }
        if(partitioned != other.partitioned)
        {
            return true;
        }
        if(prioritized != other.prioritized)
        {
            return true;
        }
        for(size_t i = 0; i < chain_latency_targets.size(); ++i)
        {
            if(chain_latency_targets[i].tv_sec != other.chain_latency_targets[i].tv_sec)
            {
                return true;
            }
            if(chain_latency_targets[i].tv_usec != other.chain_latency_targets[i].tv_usec)
            {
                return true;
            }
        }
        if(best_effort != other.best_effort)
        {
            return true;
        }
        return false;
    }
    bool operator==(const State &other) const
    {
        return !(*this != other);
    }

private:
    int num_threads;
    std::vector<int> thread_prio;
    std::vector<int> thread_policy;
    std::vector<uint64_t> sched_deadline_budget;
    std::vector<cpu_set_t> thread_cpu_set;
    std::vector<std::shared_ptr<Chain>> chains;
    std::vector<std::vector<std::shared_ptr<Callback>>> partitioned_callbacks;
    // std::vector<bool> partitioned;
    // std::vector<bool> prioritized;
    bool partitioned;
    bool prioritized;
    std::vector<struct timeval> chain_latency_targets;
    std::vector<bool> best_effort;
    friend class MPCController;
    friend class executor;
};

#endif // STATE_HPP
