#ifndef MPC_CONTROLLER_CPP
#define MPC_CONTROLLER_CPP

#include <mpccontroller.hpp>
#define THREAD_PERIOD 1000000 // 1ms
#define THREAD_PERIOD_US 1000 // 1ms
MPCController::MPCController() {}
MPCController::~MPCController() {}

void MPCController::assign_executor(executor *exec)
{
    this->exec = exec;
}

void MPCController::run()
{
    this->create_threadclass();
    while (exec->is_running())
    {
        std::this_thread::sleep_for(period);

        // Get the current state from the executor
        State current_state = exec->get_state();

        // Create a new state for the next prediction
        State next_state = current_state;

        for (auto &tc : threadclasses)
        {
            if (tc->rt_threadclass)
            {
                reduce_rt_budget(tc, current_state);
            }
        }
        // Optimize the sched_deadline_budget
        // optimize_sched_runtime(current_state, next_state);

        // Apply the next state
        // if (current_state != next_state)
        // {
        //     apply_next_state(next_state);
        // }
        
        // IMPORTANT: Apply callback-to-thread assignment to rclcpp
        exec->apply_callback_to_thread_assignment();
    }
}

timeval add_timevals(const timeval &t1, const timeval &t2)
{
    timeval result;
    result.tv_sec = t1.tv_sec + t2.tv_sec;
    result.tv_usec = t1.tv_usec + t2.tv_usec;

    // If the sum of microseconds exceeds 1 million, adjust the result
    if (result.tv_usec >= 1000000)
    {
        result.tv_sec += result.tv_usec / 1000000;
        result.tv_usec %= 1000000;
    }
    return result;
}

double interference(double delta, double alpha, double T, double C)
{
    return std::floor((delta + alpha) / T) * C + std::min(C, delta + alpha - std::floor((delta + alpha) / T) * T);
}

double sbf(double delta, int budget, int period)
{
    double sbf_delta = 0;
    if (delta >= 2 * (double)(period - budget))
    {
        sbf_delta = ((double)budget / (double)period) * (delta - 2 * ((double)period - (double)budget));
    }
    else
    {
        sbf_delta = 0;
    }
    return sbf_delta;
}
double pseudo_inv_sbf(double x, int budget_us, int period_us)
{
    if (x > 0)
    {
        return ((double)period_us / (double)budget_us) * x + 2 * ((double)period_us - (double)budget_us);
    }
    return -1;
}
// double pseudo_inv_sbf(double delta, double x, int period_us, int budget_us)
// {
//     // find the minimum delta such that sbf(delta, budget, period) == x
//     double delta_min = 2 * (period_us - budget_us);
//     double delta_max = delta_min + 2 * x;
//     double delta_mid = 0;
//     while (delta_max - delta_min > 1e-6)
//     { // this may cause inf loop
//         delta_mid = (delta_min + delta_max) / 2;
//         if (sbf(delta_mid, period_us, budget_us) < x)
//         {
//             delta_min = delta_mid;
//         }
//         else
//         {
//             delta_max = delta_mid;
//         }
//     }
//     std::cout << "Calculated delta: " << delta_mid << std::endl;
//     return delta_mid;
// }

double MPCController::MLP(double M, std::vector<std::shared_ptr<Chain>> chainset, int self_priority, double delta, State current_state)
{
    std::vector<double> possible_lp;
    double retval = 0;
    // for each chain
    for (size_t k = 0; k < chainset.size(); k++)
    {
        std::vector<double> each_chain_lp;
        // for each callback in the chain
        for (size_t j = 0; j < chainset[k]->getCallbacks().size(); j++)
        {
            // if the callback has a lower priority than the current chain
            if (chainset[k]->getCallbacks()[j]->getPriority() < self_priority) // get callback
            {

                double b = std::min((double)(chainset[k]->getCallbacks()[j]->getExecutionTime(current_state).tv_sec * 1e6 + chainset[k]->getCallbacks()[j]->getExecutionTime(current_state).tv_usec - 1), delta);
                each_chain_lp.push_back(b);
            }
        }
        if (each_chain_lp.empty())
        {
            continue;
        }
        possible_lp.push_back(*std::max_element(each_chain_lp.begin(), each_chain_lp.end()));
    }
    auto n = std::min(M, double(possible_lp.size()));
    // Get the n largest elements from possible_lp and store to vector
    std::vector<double> mlp;
    mlp.reserve(n);
    std::partial_sort_copy(possible_lp.begin(), possible_lp.end(), mlp.begin(), mlp.end(), std::greater<double>());
    for (size_t c = 0; c < mlp.size(); c++)
    {
        retval += mlp[c];
    }
    return retval;
}

std::vector<struct timeval> MPCController::pwa_cd(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget, State current_state)
{
    // M is the number of threads in the threadclass * the total budget (per thread)
    // double M = tg->threads.size() * (double)tg->total_budget / 10000;

    double M = (double)tg->threads.size(); //* (double)budget / (double)THREAD_PERIOD;
    int k = 0;
    std::vector<double> response_times;
    response_times.resize(chainset.size());
    std::vector<double> delta_values;
    delta_values.resize(chainset.size());
    if (budget == 0)
    {
        std::cout << "Threadclass budget is 0" << std::endl;
        for (size_t i = 0; i < response_times.size(); i++)
        {
            response_times[i] = 20e6;
        }
        goto out;
    }
    if (chainset.size() == 0)
    {
        std::cout << "No chains in chainset" << std::endl;
        for (size_t i = 0; i < response_times.size(); i++)
        {
            response_times[i] = 0;
        }
        goto out;
    }
    // for each chain assigned to the chainset
    for (auto &chain : chainset)
    {
        // Initialize Delta and E_k
        double delta = 1;
        auto E_k = 1;
        // Calculate the total execution time of all callbacks in the chain
        for (auto &callback : chain->getCallbacks())
        {
            if (callback->getPlaceInChain() != chain->getNumCallbacks()) // this assumes linear chains only, no branches
            {
                // the state-wise WCET time of the callback
                // auto ex_time = callback->getExecutionTime(exec->get_state());
                auto ex_time = callback->getExecutionTime(current_state);
                // Chain K's execution time is the sum of all callback execution times (except the last callback)
                E_k += ex_time.tv_sec * 1e6 + ex_time.tv_usec;
            }
        }

        // Calculate the response time for the chain

        while (true && this->exec->is_running())
        {
            double W = 0.0;

            // initialize W, intf
            double intf = 0.0;
            // interference from all interfering chains
            // for each interfering chain in the same chainset
            for (auto &interf_chain : chainset)
            {
                // if the interfering chain is not the same as the current chain
                if (interf_chain != chain)
                {
                    // calculate the interference
                    // T is the period of the interfering chain
                    auto T = interf_chain->getPeriod().tv_sec * 1e6 + interf_chain->getPeriod().tv_usec;
                    // D is the deadline of the interfering chain
                    auto D = interf_chain->getDeadline().tv_sec * 1e6 + interf_chain->getDeadline().tv_usec;
                    // C is the total execution time of all callbacks in the interfering chain
                    auto C = 0;
                    // for all callbacks in the interfering chain
                    for (auto &callback : interf_chain->getCallbacks())
                    {
                        auto ex_time = callback->getExecutionTime(current_state);
                        C += ex_time.tv_sec * 1e6 + ex_time.tv_usec;
                    }
                    // if the interfering chain is a RT chain
                    if (tg->rt_threadclass)
                    {
                        // assuming linear chains only: if the interfering chain has a higher priority than the current chain
                        if (interf_chain->getBranchPriority(0) > chain->getBranchPriority(0))
                        {
                            // calculate the interference
                            intf += interference(delta, D - C, T, C);
                        }
                    }
                    else // if the interfering chain is a BE chain
                    {
                        // calculate the interference
                        intf += interference(delta, D - C, T, C);
                    }
                }
            }
            // calculate the response time
            // if the threadclass is a BE threadclass
            if (!tg->rt_threadclass)
            {
                // W = M * E_k + intf
                W = M * double(E_k) + intf;
            }
            else
            {
                // if the threadclass is a RT threadclass
                // calculate the B term

                std::vector<std::shared_ptr<Chain>> exclusive_chainset;
                for (auto &candidate_chain : chainset)
                {
                    if (candidate_chain != chain)
                    {
                        exclusive_chainset.push_back(candidate_chain);
                    }
                }
                // Perform the MLP calculation
                double B = MLP(M, exclusive_chainset, chain->getBranchPriority(0), delta, current_state);
                // W = M * E_k + intf + B
                W = M * double(E_k) + intf + B;
            }
            auto sbfd = std::ceil(sbf(delta, budget, THREAD_PERIOD_US));
            // if W is negative, increment delta
            if (W < 0)
            {
                delta++;
            }
            // else if (W < M * delta)
            else if (W < M * sbfd) // change for period and budget
            {
                // if W is less than M * delta, set the response time and delta
                // x is the execution time of the last callback in the chain - 1
                auto x = (double)chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime(current_state).tv_sec * 1e6 + chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime(current_state).tv_usec - 1;
                // the response times are calculated as the sum of the execution time of the last callback in the chain and the pseudo-inverse of the SBF function
                response_times[k] = delta + pseudo_inv_sbf(x, budget, THREAD_PERIOD_US); // units are in microseconds, not nanoseconds
                // update delta value
                delta_values[k] = delta;
                k++;
                break;
            }
            else if (delta > (double)(1000000)) // thread period
            {                                   // higher limit
                // not schedulable
                response_times[k] = 20e6;
                delta_values[k] = delta;
                k++;
                break;
            }
            else
            {
                auto delta_prev = delta;
                // if W is greater than M * delta, increment delta
                delta = 1 + std::floor(W / M);
                if (delta <= delta_prev)
                {
                    delta = delta_prev + 1;
                    // delta += std::floor(W/M);
                }
            }
        }
    }
out:
    std::vector<struct timeval> return_response_times;
    for (size_t i = 0; i < chainset.size(); i++)
    {
        struct timeval response_time;
        if (response_times[i] == -1)
        {
            response_time.tv_sec = -1;
            response_time.tv_usec = -1;
            return_response_times.push_back(response_time);
            continue;
        }
        response_time.tv_sec = static_cast<long>(response_times[i] / 1e6);
        response_time.tv_usec = static_cast<long>(response_times[i] - response_time.tv_sec * 1e6);
        return_response_times.push_back(response_time);
    }
    return return_response_times;
}

// Function to predict the response time based on a new budget
struct timeval MPCController::predict_response_time(uint64_t new_budget, timeval current_response_time, uint64_t current_budget)
{
    timeval predicted_time;

    // Check if current or new budget is 0
    if (current_budget == 0 || new_budget == 0)
    {
        std::cerr << "Error: Budget cannot be 0. Returning original response time." << std::endl;
        return current_response_time;
    }

    double ratio = static_cast<double>(new_budget) / current_budget;

    // Log the current and new budgets and the ratio
    std::cout << "Predicting response time with new budget: " << new_budget
              << ", current budget: " << current_budget
              << ", ratio: " << ratio << std::endl;

    // Calculate the predicted response time
    predicted_time.tv_sec = static_cast<long>(current_response_time.tv_sec / ratio);
    predicted_time.tv_usec = static_cast<long>(current_response_time.tv_usec / ratio);

    // Log the predicted response time
    std::cout << "Predicted response time: " << predicted_time.tv_sec
              << "s " << predicted_time.tv_usec << "us" << std::endl;

    return predicted_time;
}

double MPCController::compute_chain_utilization(std::shared_ptr<Chain> chain, State current_state)
{
    double utilization = 0;
    for (auto &callback : chain->getCallbacks())
    {
        auto ex_time = callback->getExecutionTime(current_state);
        utilization += ex_time.tv_sec * 1e6 + ex_time.tv_usec;
    }
    return utilization / (chain->getPeriod().tv_sec * 1e6 + chain->getPeriod().tv_usec);
}

void MPCController::control_budget(State &current_state, State &next_state)
{
    (void)current_state; (void)next_state;
    // for each threadclass
    for (auto &tg : threadclasses)
    {
        // if the threadclass is a RT threadclass
        if (tg->rt_threadclass)
        {
            // for each chain in the threadclass
        }
    }
}

void MPCController::create_threadclass(void)
{
    State current_state = exec->get_state();
    // Start with half and half allocation between RT and BE
    std::cout << "Creating threadclasses" << std::endl;
    int rt_id = 0;
    int be_id = 0;
    for (auto &thread : exec->get_threads())
    {

        if (thread->get_rt() == true)
        {
            std::cout << "Creating RT threadclass" << std::endl;
            // threadclass rt_tg(true);
            std::shared_ptr<threadclass> rt_tg = std::make_shared<threadclass>(true, rt_id++);
            rt_tg->add_thread(thread);
            rt_tg->set_utilization(0);
            rt_tg->set_period(THREAD_PERIOD);
            rt_tg->apply_budgets(THREAD_PERIOD - 1024);
            threadclasses.push_back(rt_tg);
        }
        else
        {
            std::cout << "Creating BE threadclass" << std::endl;
            // threadclass be_tg(false);
            std::shared_ptr<threadclass> be_tg = std::make_shared<threadclass>(false, be_id++);
            be_tg->add_thread(thread);
            be_tg->set_period(THREAD_PERIOD);
            be_tg->set_utilization(0);
            be_tg->apply_budgets(1024);
            threadclasses.push_back(be_tg);
        }
    }
    std::cout << "Threadclasses created" << std::endl;
    std::cout << "Parsing and sorting chains" << std::endl;
    std::vector<std::vector<std::shared_ptr<Chain>>> split_chainsets = exec->parse_and_sort_chains(exec->get_chains());
    std::vector<std::shared_ptr<Chain>> rt_chains = split_chainsets[0];
    std::vector<std::shared_ptr<Chain>> be_chains = split_chainsets[1];
    bool merge = false;
    // for each chainset sorted in ascending order
    // for each RT chain, perform a worst fit decreasing assignment to RT threadclass based on utilization
    // for each chain in the RT chainset
merge_min_tgs:
    if (merge)
    {
        // merge the two threadclasses with the minimum utilization
        std::shared_ptr<threadclass> min_tg1 = nullptr;
        std::shared_ptr<threadclass> min_tg2 = nullptr;
        double min_util1 = 1;
        double min_util2 = 1;
        // also keep track of the tg index
        int tg1_index = 0;
        int tg2_index = 0;
        int min_tg1_index = 0;
        int min_tg2_index = 0;

        int num_rt_threadclasses = 0;
        for (auto &tg : threadclasses)
        {
            if (tg->rt_threadclass == true)
            {
                num_rt_threadclasses++;
                double util = tg->get_utilization();
                if (util < min_util1)
                {
                    min_tg1 = tg;
                    min_util1 = util;
                    min_tg1_index = tg1_index;
                }
                else
                {
                    tg1_index++;
                }
            }
        }
        for (auto &tg : threadclasses)
        {
            double util = tg->get_utilization();
            if (tg->rt_threadclass == true)
            {
                if (util < min_util2 && tg != min_tg1)
                {
                    min_tg2 = tg;
                    min_util2 = util;
                    min_tg2_index = tg2_index;
                }
                else
                {
                    tg2_index++;
                }
            }
        }
        if (min_tg1 == nullptr || min_tg2 == nullptr)
        {
            std::cout << "No more RT threadclasses to merge" << std::endl;
            return;
        }
        min_tg1->merge_threadclasss(min_tg1, min_tg2);
        // since we merged two RT threadclasses, we need to merge their matching pairs in the BE threadclasses before erasing the extra RT threadclass
        // each matching pair will be the index of the RT threadclasses + the number of RT threadclasses
        threadclasses.at(num_rt_threadclasses + min_tg1_index)->merge_threadclasss(threadclasses.at(num_rt_threadclasses + min_tg1_index), threadclasses.at(num_rt_threadclasses + min_tg2_index));

        threadclasses.erase(std::remove(threadclasses.begin(), threadclasses.end(), min_tg2), threadclasses.end());
        threadclasses.erase(std::remove(threadclasses.begin(), threadclasses.end(), threadclasses.at(num_rt_threadclasses + min_tg2_index - 1)), threadclasses.end());
        // once the merging is done, reset the merge flag and retry allocation

        merge = false;
    }

    std::vector<std::shared_ptr<threadclass>> tried_threadclasses;
    unsigned int num_rt_tg = 0;
    // count the number of RT capable threadgroups
    for (auto &tg : threadclasses)
    {
        if (tg->rt_threadclass == true)
        {
            num_rt_tg++;
        }
    }
    std::cout << "Number of RT capable threadgroups: " << num_rt_tg << std::endl;

    for (auto &chain : rt_chains)
    {

        std::shared_ptr<threadclass> min_tg = nullptr;
        double min_util = 1;
        // for each threadclass that is RT capable -- find the one with the lowest utilization
        // each threadgroup starts with one thread and 0 utilization
    retry_tg_alloc:
        for (auto &tg : threadclasses)
        {
            // if the threadgroup is inside the tried threadclasses, skip it
            if (tried_threadclasses.size() == num_rt_tg)
            {
                merge = true;
                goto merge_min_tgs;
            }
            else if (std::find(tried_threadclasses.begin(), tried_threadclasses.end(), tg) != tried_threadclasses.end())
            {
                continue;
            }

            // if the threadgroup is RT capable
            if (tg->rt_threadclass == true)
            {
                // get the utilization of the threadgroup
                double util = tg->get_utilization();
                // if the utilization is less than the minimum utilization
                if (util < min_util)
                {
                    // set the minimum utilization to the current utilization
                    min_tg = tg;
                    min_util = util;
                }
            }
        }
        // once we find the minimum utilization threadgroup, lets do assignment of chains to threadgroups
        // first, see if adding the chain to the threadgroup will still let our chainset pass the schedulabilit test
        // if the chain is not schedulable, skip the assignment and try another threadgroup
        auto existing_chains = min_tg->get_chains();
        existing_chains.push_back(chain);
        std::cout << "Trying to add chain to RT threadgroup " << min_tg->id << " with utilization: " << min_tg->get_utilization() << std::endl;
        // check the schedulability of the chainset with the threadgroup
        std::cout << "Computing chain response times" << std::endl;
        auto response_times = pwa_cd(existing_chains, min_tg, min_tg->total_budget / 1000, current_state);
        // debug print response times and deadlines
        for (size_t i = 0; i < response_times.size(); i++)
        {
            std::cout << "Chain " << i << " Response Time: " << response_times[i].tv_sec << "s " << response_times[i].tv_usec << "us" << std::endl;
            std::cout << "Chain " << i << " Deadline: " << chain->getDeadline().tv_sec << "s " << chain->getDeadline().tv_usec << "us" << std::endl;
        }
        // if the chainset is schedulable, assign the chain to the threadgroup
        // if the chainset is not schedulable, remove the chain from the threadgroup and try another threadgroup
        bool schedulable = true;
        for (size_t i = 0; i < response_times.size(); i++)
        {
            if (response_times[i].tv_sec * 1e6 + response_times[i].tv_usec > chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec)
            {
                schedulable = false;
                break;
            }
        }

        if (schedulable)
        {
            double chain_util = compute_chain_utilization(chain, current_state);
            min_tg->add_chain(chain);
            min_tg->set_utilization(min_util + chain_util);
            std::cout << "Chain is schedulable. Adding chain to RT threadgroup: " << min_tg->id << " with new utilization: " << min_tg->get_utilization() << std::endl;
        }
        else
        {
            existing_chains.pop_back();
            tried_threadclasses.push_back(min_tg);
            if (tried_threadclasses.size() == num_rt_tg)
            {
                merge = true;
                goto merge_min_tgs;
            }
            goto retry_tg_alloc;
        }

        // min_tg->add_thread(chain);
        // min_tg->set_utilization(min_util + chain->getUtilization());
    }

    // for each BE chain, perform a worst fit decreasing assignment to BE threadclass based on utilization
    for (auto &chain : be_chains)
    {
        std::shared_ptr<threadclass> min_tg = nullptr;
        std::vector<std::shared_ptr<threadclass>> tried_threadclasses;
        double min_util = 1;
        int num_be_tg = threadclasses.size() - num_rt_tg;

        // for each threadclass that is BE capable -- find the one with the lowest utilization
    retry_be_alloc:
        for (auto &tg : threadclasses)
        {
            // if the threadgroup is RT capable, skip it
            if (tg->rt_threadclass == true)
            {
                continue;
            }
            // also if the threadgroup is inside the tried threadclasses, skip it
            if (std::find(tried_threadclasses.begin(), tried_threadclasses.end(), tg) != tried_threadclasses.end())
            {
                continue;
            }

            // get the utilization of the threadgroup
            double util = tg->get_utilization();
            // if the utilization is less than the minimum utilization
            if (util < min_util)
            {
                // set the minimum utilization to the current utilization
                min_tg = tg;
                min_util = util;
            }
        }
        // once we find the minimum utilization threadgroup, lets do assignment of chains to threadgroups
        // first, see if adding the chain to the threadgroup will still let our chainset pass the schedulabilit test
        // if the chain is not schedulable, skip the assignment and try another threadgroup
        auto existing_chains = min_tg->get_chains();
        existing_chains.push_back(chain);
        std::cout << "Trying to add chain to BE threadgroup " << min_tg->id << " with utilization: " << min_tg->get_utilization() << std::endl;
        // since the budget is 0, the response time analysis will result in infinite loop.
        // we can skip the response time analysis for now
        min_tg->add_chain(chain);
        min_tg->set_utilization(min_util + compute_chain_utilization(chain, current_state));

        std::cout << "Chain added to BE threadgroup: " << min_tg->id << " with new utilization: " << min_tg->get_utilization() << std::endl;
    }
}

void MPCController::reduce_rt_budget(std::shared_ptr<threadclass> tc, const State &current_state)
{
    // State current_state = exec->get_state();
    //  basically we need to minimize the budget of the RT threadclass such that all the RT chains are still schedulable
    //  we can do this by performing a binary search on the budget and evaluating the schedulability of the chainset
    //  if the chainset is schedulable, we can reduce the budget further
    //  if the chainset is not schedulable, we can increase the budget
    //  we can start with the current budget of the threadclass
    int min_budget = 1024, max_budget = THREAD_PERIOD, mid_budget = 0;
    int current_budget = tc->total_budget;
    int best_budget = 1024;
    std::vector<std::shared_ptr<Chain>> chainset = tc->get_chains();
    std::vector<struct timeval> response_times;
    std::vector<struct timeval> deadlines;
    if (chainset.empty())
    {
        std::cout << "No chains assigned to threadclass" << std::endl;
        goto empty_rt_chain;
        // return;
    }

    for (auto &chain : chainset)
    {
        deadlines.push_back(chain->getDeadline());
    }
    while (min_budget < max_budget)
    {
        mid_budget = (min_budget + max_budget) / 2;
        // tc->apply_budgets(mid_budget);
        std::cout << "Trying to reduce budget to: " << mid_budget << std::endl;
        response_times = pwa_cd(chainset, tc, mid_budget / 1000, current_state);
        int i = 0;
        for (auto &chain : chainset)
        {
            auto chain_response_time = 0;
            for (auto &callback : chain->getCallbacks())
            {
                chain_response_time += callback->getExecutionTime(current_state).tv_sec * 1e6 + callback->getExecutionTime(current_state).tv_usec;
            }

            std::cout << "Chain Measured Response Time: " << chain_response_time << std::endl;
            std::cout << "Chain Deadline: " << deadlines[i].tv_sec * 1e6 + deadlines[i].tv_usec << std::endl;
            std::cout << "Chain Predicted Response Time: " << response_times[i].tv_sec * 1e6 + response_times[i].tv_usec << std::endl;
            i++;
        }

        // std::cout << "Response times: " << std::endl;
        //  for (size_t i = 0; i < response_times.size(); i++)
        //  {
        //      auto actual_response_time = 0;
        //      for(auto &callback : chain->getCallbacks()){
        //          actual_response_time += callback->getExecutionTime(current_state).tv_sec * 1e6 + callback->getExecutionTime(current_state).tv_usec;
        //      }
        //      std::cout << "Chain " << i << " Predicted Response Time: " << response_times[i].tv_sec << "s " << response_times[i].tv_usec << "us" << std::endl;
        //      std::cout << "Chain" << i << "Actual Response Time: " << actual_response_time << "us" << std::endl;
        //      std::cout << "Chain " << i << " Deadline: " << deadlines[i].tv_sec << "s " << deadlines[i].tv_usec << "us" << std::endl;
        //  }
        bool schedulable = true;
        for (size_t i = 0; i < response_times.size(); i++)
        {
            if (response_times[i].tv_sec * 1e6 + response_times[i].tv_usec > deadlines[i].tv_sec * 1e6 + deadlines[i].tv_usec || response_times[i].tv_usec == -1)
            {
                schedulable = false;
                break;
            }
        }
        if (schedulable)
        {
            max_budget = mid_budget;
        }
        else
        {
            min_budget = mid_budget + 1;
        }
    }
    best_budget = std::max(min_budget, std::min(mid_budget, max_budget));

empty_rt_chain:
    // now assign the budget to the threadclass
    tc->apply_budgets(best_budget);
    // now we need to set the budget for each thread in the threadclass
    if (tc->rt_threadclass)
    {
        std::cout << "Setting budget for RT threadclass" << std::endl;
    }
    else
    {
        std::cout << "Setting budget for BE threadclass" << std::endl;
    }
    for (auto &thread : tc->threads)
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        attr.sched_policy = SCHED_DEADLINE;
        attr.sched_runtime = best_budget;
        attr.sched_period = THREAD_PERIOD;
        attr.sched_deadline = THREAD_PERIOD;
        attr.sched_flags = 0 | SCHED_FLAG_RECLAIM;
        attr.sched_nice = 0;
        attr.sched_priority = 0;
        attr.sched_util_min = 0;
        attr.sched_util_max = 1024;
        thread->set_sched_deadline(attr, 0);
    }
    bool fifo = false;
    int remaining_budget = THREAD_PERIOD - best_budget;
    if (double(remaining_budget) < double(THREAD_PERIOD) * 0.05)
    {
        std::cerr << "Remaining budget is less than 5% of the period. " << std::endl;
        // switching to sched_fifo
        fifo = true;
    }
    // now we need to distribute remaining budget to the complementary BE threadclass
    // first we have to identify the BE threadclass (it will be assigned to the same cpu core)
    std::shared_ptr<threadclass> be_tc = nullptr;
    for (auto &threadclass : threadclasses)
    {
        if (threadclass->rt_threadclass == false && CPU_EQUAL(threadclass->threads[0]->get_cpuSet(), tc->threads[0]->get_cpuSet()))
        {
            be_tc = threadclass;
            break;
        }
    }
    if (be_tc == nullptr)
    {
        std::cerr << "Error: BE threadclass not found for RT threadclass" << std::endl;
        return;
    }
    else
    {
        if (!fifo)
        {
            be_tc->apply_budgets(remaining_budget);
            for (auto &thread : be_tc->threads)
            {
                struct sched_attr attr;
                attr.size = sizeof(attr);
                attr.sched_policy = SCHED_DEADLINE;
                attr.sched_runtime = remaining_budget;
                attr.sched_period = THREAD_PERIOD;
                attr.sched_deadline = THREAD_PERIOD;
                attr.sched_flags = 0 | SCHED_FLAG_RECLAIM;
                attr.sched_nice = 0;
                attr.sched_priority = 0;
                attr.sched_util_min = 0;
                attr.sched_util_max = 1024;
                thread->set_sched_deadline(attr, 0);
            }
        }
        else
        {
            be_tc->apply_budgets(remaining_budget);
            for (auto &thread : be_tc->threads)
            {
                thread->set_policy(SCHED_FIFO);
                thread->set_priority(0);
            }
        }
    }
    // now we can compute whether the BE chain is schedulable, or if not, if the BE chain will be starvation free within one controller period
    // if the BE chain is not starvation free, we can make a log of BE performance degradation
    // if the BE chain is schedulable, we can proceed with the next chain, and log that it is schedulable
    // for(auto &threadclass : threadclasses){
    // only if the threadclass is a BE threadclass
    // if(!->rt_threadclass){
    if (be_tc->get_utilization() > (double)(be_tc->num_threads) * ((double)be_tc->total_budget) / (double)THREAD_PERIOD)
    {
        std::cerr << "BE Threadclass " << be_tc->id << " is overloaded, expect degraded BE performance" << std::endl;
    }
    else
    {
        std::cout << "BE Threadclass " << be_tc->id << " is not overloaded, expect bounded performance" << std::endl;
    }
    auto be_response_times = pwa_cd(be_tc->get_chains(), be_tc, be_tc->total_budget / 1000, current_state);
    std::cout << "BE Threadclass " << be_tc->id << " response times: " << std::endl;
    for (size_t i = 0; i < be_response_times.size(); i++)
    {
        if (be_response_times[i].tv_sec * 1e6 + be_response_times[i].tv_usec >= 20e6)
        {
            std::cerr << "BE Chain " << i << " is not starvation free" << std::endl;
        }
        else
        {
            std::cout << "BE Chain " << i << " is starvation free" << std::endl;
            std::cout << "Estimated Response Time for Chain " << i << ": " << be_response_times[i].tv_sec << "s " << be_response_times[i].tv_usec << "us" << std::endl;
        }
    }
    //}
    //}
}

void MPCController::optimize_sched_runtime(State &current_state, State &next_state)
{
    // for each chain
    for (size_t i = 0; i < current_state.chains.size(); ++i)
    {
        // get current sched deadline budget
        uint64_t current_budget = current_state.sched_deadline_budget[i];
        timeval current_response_time;                                   // Actual response time
        timeval latency_target = current_state.chain_latency_targets[i]; // Latency target for chain i
                                                                         // get the 95th percentile of the measuerd response times for the chain
        std::deque<struct timeval> response_times = current_state.chains[i]->get_branch_response_time_history(current_state, 0);

        if (latency_target.tv_sec == 0 && latency_target.tv_usec == 0)
        {
            std::cerr << "Error: Latency target is 0. Skipping optimization for chain " << i << std::endl;
            continue;
        }
        // Check if the response times vector is not empty
        if (!response_times.empty())
        {
            // Sort the response times based on their total microseconds
            std::sort(response_times.begin(), response_times.end(), [](const struct timeval &a, const struct timeval &b)
                      { return (a.tv_sec * 1000000 + a.tv_usec) < (b.tv_sec * 1000000 + b.tv_usec); });

            // Calculate the index for the 95th percentile
            size_t index = static_cast<size_t>(response_times.size() * 0.95);

            // Ensure the index does not exceed the size of the vector
            if (index >= response_times.size())
            {
                index = response_times.size() - 1; // Set to the last element if index is out of bounds
            }

            // Get the 95th percentile response time
            timeval percentile_response_time = response_times[index];
            current_response_time = percentile_response_time;
            // Now you can use percentile_response_time for further processing
            std::cout << "95th percentile response time: " << percentile_response_time.tv_sec << "s "
                      << percentile_response_time.tv_usec << "us" << std::endl;
        }
        else
        {
            // Handle the case where response_times is empty
            std::cerr << "Error: No response times available for chain." << std::endl;
            continue;
            current_response_time = {std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max()};
        }
        if (current_response_time.tv_sec == 0 && current_response_time.tv_usec == 0)
        {
            std::cerr << "Error: Response time is 0. Skipping optimization for chain " << i << std::endl;
            continue;
        }
        else if (current_response_time.tv_sec * 1e6 + current_response_time.tv_usec == latency_target.tv_sec * 1e6 + latency_target.tv_usec)
        {
            std::cout << "Response time is equal to latency target. Skipping optimization for chain " << i << std::endl;
            continue;
        }
        // Log current state information
        std::cout << "Optimizing thread " << i << " with current budget: "
                  << current_budget << " and current response time: "
                  << current_response_time.tv_sec << "s "
                  << current_response_time.tv_usec << "us" << std::endl;

        // If the budget is 0, assign a budget based on the current response time
        if (current_budget == 0)
        {
            double response_time_usec = current_response_time.tv_sec * 1e6 + current_response_time.tv_usec;
            double target_time_usec = latency_target.tv_sec * 1e6 + latency_target.tv_usec;

            // Calculate the necessary budget as a ratio of response time to latency target
            current_budget = static_cast<uint64_t>(response_time_usec / target_time_usec * 1e6);
            std::cout << "Assigning new budget based on response time: " << current_budget << " us" << std::endl;
            // bound check
            if (current_budget < 5000)
            {
                current_budget = 5000;
            }
            if (current_budget > 1e6)
            {
                current_budget = 1e6;
            }
            next_state.sched_deadline_budget[i] = current_budget;

            // Apply the new budget to the system
            // exec->set_thread_budget(i, current_budget);
            // exec->set_thread_policy(i, SCHED_DEADLINE);
            // exec->set_thread_affinity(exec->get_thread(i)->get_threadID(), current_state.thread_cpu_set[i]);
            continue;
        }

        // Try different budgets and predict response times
        uint64_t best_budget = current_budget <= 1e6 ? current_budget : 1e6; // Initialize with the current budget
        timeval best_predicted_time = current_response_time;
        double min_difference = std::numeric_limits<double>::max(); // Initialize with a large number
        double step_size = 0.01 * current_budget;                   // Adaptive step size (10% of current budget)
        uint64_t min_budget = 5000;                                 // Minimum budget, e.g., 10ms
        uint64_t max_budget = 1e6;                                  // 2 * current_budget < 1e6 ? 2 * current_budget : 1e6;                   // Max budget is twice the current budget
        uint64_t new_budget = current_budget;
        double current_response_time_usec = current_response_time.tv_sec * 1e6 + current_response_time.tv_usec;
        double latency_target_usec = latency_target.tv_sec * 1e6 + latency_target.tv_usec;
        // Start with the current budget and adjust iteratively
        if (current_response_time_usec > latency_target_usec)
        {
            min_budget = current_budget;
        }
        else
        {
            max_budget = current_budget;
        }

        while (max_budget > min_budget)
        {
            // Calculate the mid-point between min and max budgets
            uint64_t new_budget = (max_budget + min_budget) / 2;
            std::cout << "New budget: " << new_budget << std::endl;
            std::cout << "Current budget: " << current_budget << std::endl;
            std::cout << "Min budget: " << min_budget << std::endl;
            std::cout << "Max budget: " << max_budget << std::endl;
            std::cout << "Current response time: " << current_response_time_usec << std::endl;
            // uint64_t new_budget = current_budget;
            timeval predicted_time = predict_response_time(new_budget, current_response_time, current_budget);

            double predicted_time_usec = predicted_time.tv_sec * 1e6 + predicted_time.tv_usec;
            // Calculate the difference between the predicted response time and the latency target
            double difference = std::abs(predicted_time_usec - latency_target_usec);

            // Log the prediction
            std::cout << "Budget: " << new_budget << " us, Predicted time: "
                      << predicted_time.tv_sec << "s "
                      << predicted_time.tv_usec << "us, Difference: " << difference << " us" << std::endl;

            // Update the best option if this budget results in a smaller difference
            if (difference < min_difference)
            {
                min_difference = difference;
                best_predicted_time = predicted_time;
                best_budget = new_budget;
            }

            if (predicted_time_usec > latency_target_usec)
            {
                std::cout << "Predicted time is greater than latency target" << std::endl;
                min_budget = new_budget + 1;
            }
            else
            {
                std::cout << "Predicted time is later than latency target" << std::endl;
                max_budget = new_budget - 1;
            }
            // Early exit if we are within a certain threshold of the target latency
            if (difference < 20 && predicted_time_usec <= latency_target_usec) // Allow a 1000 us margin for optimization
            {
                std::cout << "Reached optimal budget with acceptable latency difference of " << difference << " us" << std::endl;
                break;
            }
        }

        // Log the final decision
        std::cout << "Selected budget for thread " << i << ": "
                  << best_budget << " us, predicted response time: "
                  << best_predicted_time.tv_sec << "s "
                  << best_predicted_time.tv_usec << "us" << std::endl;

        best_budget = std::max(min_budget, std::min(best_budget, max_budget));

        // Log if the budget was adjusted to the bounds
        if (best_budget == min_budget)
        {
            std::cout << "Setting budget to minimum: " << min_budget << std::endl;
        }
        else if (best_budget == max_budget)
        {
            std::cout << "Setting budget to maximum: " << max_budget << std::endl;
        }

        // Update the next state with the chosen best budget
        next_state.sched_deadline_budget[i] = best_budget;
    }
}

// Function to apply the optimized state
void MPCController::apply_next_state(const State &next_state)
{
}
// Set thread budgets, priorities, and policies based on the next state
//     for (size_t i = 0; i < next_state.sched_deadline_budget.size(); ++i)
//     {
//         std::cout << "Applying new budget to thread " << i
//                   << ": " << next_state.sched_deadline_budget[i] << std::endl;

//         exec->set_thread_budget(i, next_state.sched_deadline_budget[i]);
//         exec->set_thread_policy(i, SCHED_DEADLINE);
//         cpu_set_t cpu_set;
//         CPU_ZERO(&cpu_set);
//         CPU_SET(6, &cpu_set);
//         exec->set_thread_affinity(i, cpu_set);
//         struct sched_attr attr;
//         attr.size = sizeof(attr);
//         attr.sched_policy = SCHED_DEADLINE;
//         attr.sched_runtime = next_state.sched_deadline_budget[i];
//         attr.sched_period = 1 * 1000 * 1000;   // 200ms period
//         attr.sched_deadline = 1 * 1000 * 1000; // 200ms deadline
//         attr.sched_flags = 0;
//         attr.sched_nice = 0;
//         attr.sched_priority = 0;
//         attr.sched_util_min = 0;
//         attr.sched_util_max = 1024;
//         std::cout << exec->get_thread(i)->set_sched_deadline(attr, 0) << std::endl;

//         // Log that the budget has been applied
//         std::cout << "Budget applied for thread " << i << std::endl;
//     }
// }

#endif // MPC_CONTROLLER_CPP
