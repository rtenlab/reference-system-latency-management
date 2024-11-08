#ifndef STATE_CPP
#define STATE_CPP
#include <state.hpp>

int State::boolean_vector_distance(const std::vector<bool> &v1, const std::vector<bool> &v2)
{
    int distance = 0;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        if (v1[i] != v2[i])
        {
            ++distance;
        }
    }
    return distance;
}
int State::boolean_distance(const bool &b1, const bool &b2)
{
    return b1 != b2;
}
bool State::is_deadline_switch(int policy1, int policy2)
{
    return (policy1 == SCHED_DEADLINE || policy2 == SCHED_DEADLINE) && policy1 != policy2;
}
double State::euclidean_distance(const struct timeval &t1, const struct timeval &t2)
{
    double sec_diff = t1.tv_sec - t2.tv_sec;
    double usec_diff = t1.tv_usec - t2.tv_usec;
    return std::sqrt(sec_diff * sec_diff + usec_diff * usec_diff);
}
int State::hamming_distance(cpu_set_t &set1, cpu_set_t &set2)
{
    int distance = 0;
    for (int i = 0; i < CPU_SETSIZE; ++i)
    {
        if (CPU_ISSET(i, &set1) != CPU_ISSET(i, &set2))
        {
            ++distance;
        }
    }
    return distance;
}

double State::compare_partitioned_callbacks(const std::vector<std::vector<std::shared_ptr<Callback>>> &callbacks1,
                                     const std::vector<std::vector<std::shared_ptr<Callback>>> &callbacks2)
{
    double distance = 0.0;

    if (callbacks1.size() != callbacks2.size())
    {
        // If the size of partitioned callbacks differs, assign a large penalty
        return weight_large_difference;
    }

    for (size_t i = 0; i < callbacks1.size(); ++i)
    {
        // Compare each thread's callback set
        std::set<boost::uuids::uuid> set1, set2;
        for (const auto &callback : callbacks1[i])
        {
            set1.insert(callback->getUUID());
        }
        for (const auto &callback : callbacks2[i])
        {
            set2.insert(callback->getUUID());
        }

        // Find the difference between the two sets
        std::vector<boost::uuids::uuid> diff;
        std::set_symmetric_difference(set1.begin(), set1.end(), set2.begin(), set2.end(), std::back_inserter(diff));
        distance += diff.size(); // The number of differing callbacks contributes to the distance
    }

    return distance;
}

double State::distance(State &other) 
{
    double distance = 0.0;

    // Number of threads has a huge impact
    distance += weight_num_threads * std::abs(num_threads - other.num_threads);

    // Thread priority has a minor impact
    for (size_t i = 0; i < thread_prio.size(); ++i)
    {
        distance += weight_thread_prio * std::abs(thread_prio[i] - other.thread_prio[i]);
    }

    // Thread policy has a huge impact, especially SCHED_DEADLINE
    for (size_t i = 0; i < thread_policy.size(); ++i)
    {
        if (thread_policy[i] != other.thread_policy[i])
        {
            distance += weight_policy;
            if (is_deadline_switch(thread_policy[i], other.thread_policy[i]))
            {
                distance += weight_deadline_switch;
            }
        }
    }

    // CPU set (affinity) has a huge impact
    for (size_t i = 0; i < thread_cpu_set.size(); ++i)
    {
        distance += weight_cpu_set * hamming_distance(thread_cpu_set[i], other.thread_cpu_set[i]);
    }

    // SCHED_DEADLINE budget causes predictable, linear changes
    // for (size_t i = 0; i < sched_deadline_budget.size(); ++i)
    // {
    //     if (thread_policy[i] == SCHED_DEADLINE)
    //     {
    //         distance += weight_deadline_budget * std::abs(sched_deadline_budget[i] - other.sched_deadline_budget[i]);
    //     }
    // }

    // Partitioned callbacks and partitioning cause huge differences
    distance += weight_partitioned_callbacks * compare_partitioned_callbacks(partitioned_callbacks, other.partitioned_callbacks);
    distance += weight_partitioned * boolean_distance(partitioned, other.partitioned);

    // Prioritization impacts scheduling
    distance += weight_prioritized * boolean_distance(prioritized, other.prioritized);

    // Chain latency targets differ based on RT or best-effort chains
    for (size_t i = 0; i < chain_latency_targets.size(); ++i)
    {
        if (!best_effort[i])
        {
            distance += weight_latency_target * euclidean_distance(chain_latency_targets[i], other.chain_latency_targets[i]);
        }
        else
        {
            distance += weight_best_effort_latency * euclidean_distance(chain_latency_targets[i], other.chain_latency_targets[i]);
        }
    }

    // Best effort affects chain performance
    distance += weight_best_effort * boolean_vector_distance(best_effort, other.best_effort);

    return distance;
}

#endif