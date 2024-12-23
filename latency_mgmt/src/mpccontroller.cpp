#ifndef MPC_CONTROLLER_CPP
#define MPC_CONTROLLER_CPP

#include <mpccontroller.hpp>
#define THREAD_PERIOD 10000000 // 10ms
#define THREAD_PERIOD_US 10000 // 10ms
#define NS_IN_US 1000
MPCController::MPCController() {}
MPCController::~MPCController() {}
static inline void timespec_to_timeval(struct timespec *ts, struct timeval *tv)
{
    tv->tv_sec = ts->tv_sec;
    tv->tv_usec = ts->tv_nsec / 1000;
}

void MPCController::assign_executor(executor *exec)
{
    this->exec = exec;
}

void MPCController::chain_test()
{
    // allocate chain 0 to threadgroup 0
    // allocate chain 1 to threadgroup 1

    for (auto &tg : threadclasses)
    {
        tg->chains.clear();
    }
    auto chain0 = exec->get_chains()->at(0);
    auto chain1 = exec->get_chains()->at(1);
    threadclasses[0]->add_chain(chain0);
    threadclasses[1]->add_chain(chain1);
    threadclasses[0]->set_utilization(compute_chain_utilization(chain0, current_state));
    threadclasses[1]->set_utilization(compute_chain_utilization(chain1, current_state));
    exec->enable_callback_priority();
        exec->start();

    exec->apply_callback_to_thread_assignment();
}

// void MPCController::reallocate_be_chains(){
//  // for each BE chain, perform a worst fit decreasing assignment to BE threadclass based on utilization
//      std::cout << "Parsing and sorting chains" << std::endl;
//     std::vector<std::vector<std::shared_ptr<Chain>>> split_chainsets = exec->parse_and_sort_chains(exec->get_chains());
//     std::vector<std::shared_ptr<Chain>> be_chains = split_chainsets[1];
//     for(auto &tg: threadclasses){
//         if(tg->rt_threadclass){
//             continue;
//         }
//         tg->chains.clear();
//     }
//     for (auto &chain : be_chains)
//     {
//         std::shared_ptr<threadclass> min_tg = nullptr;
//         std::vector<std::shared_ptr<threadclass>> tried_threadclasses;
//         double min_util = 1;
//         int num_be_tg = threadclasses.size() - num_rt_tg;

//         // for each threadclass that is BE capable -- find the one with the lowest utilization
//         for (auto &tg : threadclasses)
//         {
//             // if the threadgroup is RT capable, skip it
//             if (tg->rt_threadclass == true)
//             {
//                 continue;
//             }
//             // also if the threadgroup is inside the tried threadclasses, skip it
//             if (std::find(tried_threadclasses.begin(), tried_threadclasses.end(), tg) != tried_threadclasses.end())
//             {
//                 continue;
//             }

//             // get the utilization of the threadgroup
//             double util = tg->get_utilization();
//             // if the utilization is less than the minimum utilization
//             if (util < min_util)
//             {
//                 // set the minimum utilization to the current utilization
//                 min_tg = tg;
//                 min_util = util;
//             }
//         }
//         // once we find the minimum utilization threadgroup, lets do assignment of chains to threadgroups
//         // first, see if adding the chain to the threadgroup will still let our chainset pass the schedulabilit test
//         // if the chain is not schedulable, skip the assignment and try another threadgroup
//         auto existing_chains = min_tg->get_chains();
//         existing_chains.push_back(chain);
//         std::cout << "Trying to add chain " << chain->getChainID() << " to BE threadgroup " << min_tg->id << " with utilization: " << min_tg->get_utilization() << std::endl;
//         // since the budget is 0, the response time analysis will result in infinite loop.
//         // we can skip the response time analysis for now
//         min_tg->add_chain(chain);
//         min_tg->set_utilization(min_util + compute_chain_utilization(chain, current_state));

//         std::cout << "BE Chain " << chain->getChainID() << " added to BE threadgroup: " << min_tg->id << " with new utilization: " << min_tg->get_utilization() << std::endl;
//     }
// }

void MPCController::verify_starvation_freedom(std::shared_ptr<threadclass> tc, State current_state)
{
    std::cout << "Verifying starvation freedom for BE threadclass " << tc->id << std::endl;
    // the threadclass is assumed to be BE, check the response times of the chains and see if theyre bounded
    int i = 0;
    static int tries = 0;
    auto chain_response_times = pwa_cd(tc->chains, tc, tc->total_budget / NS_IN_US, current_state);
    for (auto &rt : chain_response_times)
    {
        auto temp_rt = rt.tv_sec * 1e6 + rt.tv_usec;
        std::cout << "Chain: " << i++ << " Estimated response time: " << temp_rt << std::endl;
        if (temp_rt >= 8e6 - 100 || temp_rt < 0)
        {
            std::cout << "Starvation detected in BE threadclass " << tc->id << std::endl;
            // if(tries == 0){
            // reallocate_be_chains();
            // tries++;
            // }
        }
    }
}

void MPCController::run()
{
    struct timespec start, end;
    struct timeval start_time, end_time;
    // gettimeofday(&start_time, NULL);
    this->create_threadclass();
    // gettimeofday(&end_time, NULL);
    std::cout << "Threadclass creation time: " << (end_time.tv_sec - start_time.tv_sec) * 1e6 + (end_time.tv_usec - start_time.tv_usec) << std::endl;
    bool first_run = true;
    do
    {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
        bool realloc = false;
        // gettimeofday(&start_time, NULL);
        //  Get the current state from the executor
        State current_state = exec->get_state();

        // Create a new state for the next prediction
        State next_state = current_state;
        unsigned int analysis_count = 0;
        // Check timing violations and adjust the budget
        for (auto &tc : threadclasses)
        {
            if (tc->rt_threadclass)
            {
                if (tc->chains.size() == 0 && first_run)
                {
                    // realloc = reduce_rt_budget(tc, current_state, &analysis_count) ? true : false;
                }
                for (auto &chain : tc->chains)
                {
                    // std::cout << "Controller evaluating chain " << chain->getChainID() << std::endl;

                    uint64_t chain_response_time = 0;
                    // for (auto &callback : chain->getCallbacks())
                    // {
                    //     auto temp_rt = callback->getExecutionTime(current_state).tv_sec * 1e6 + callback->getExecutionTime(current_state).tv_usec;
                    //     chain_response_time += temp_rt;
                    //     std::cout << "Callback " << callback->getName() << " response time: " << temp_rt << std::endl;
                    // }
                    chain_response_time = chain->getFirstCallback()->getChain()->getChainResponseTime(0);
                    // std::cout << "Chain " << chain->getChainID() << " response time: " << chain_response_time << std::endl;
                    // std::cout << "Chain " << chain->getChainID() << " deadline: " << chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec << std::endl;
                    if (chain_response_time > chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec && chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec != 0)
                    {
                        std::cout << "Timing violation detected in RT chain " << chain->getChainID() << std::endl;
                        std::cout << "Response time: " << chain_response_time << " Deadline: " << chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec << std::endl;
                        // realloc = reduce_rt_budget(tc, current_state, &analysis_count) ? true : false;
                    }
                    else if (first_run)
                    {
                        realloc = true;
                        // realloc = reduce_rt_budget(tc, current_state, &analysis_count) ? true : false;
                    }
                }
            }
            else
            {
                analysis_count++;
                // update_tc_utilization(tc, current_state);
                // verify_starvation_freedom(tc, current_state);
            }
        }
        // gettimeofday(&end_time, NULL);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
        timespec_to_timeval(&start, &start_time);
        timespec_to_timeval(&end, &end_time);

        std::cout << "Controller activation time: " << (end_time.tv_sec - start_time.tv_sec) * 1e6 + (end_time.tv_usec - start_time.tv_usec) << " Number of times analysis was performed: " << analysis_count << std::endl;
        // for (auto &tc : threadclasses)
        // {
        //     if (tc->rt_threadclass)
        //     {
        //         reduce_rt_budget(tc, current_state);
        //     }
        // }
        // Optimize the sched_deadline_budget
        // optimize_sched_runtime(current_state, next_state);

        // Apply the next state
        // if (current_state != next_state)
        // {
        //     apply_next_state(next_state);
        // }

        // IMPORTANT: Apply callback-to-thread assignment to rclcpp
        if (first_run)
        {
            exec->enable_callback_priority();
            exec->start();
            //exec->apply_callback_to_thread_assignment();
            first_run = false;
        }
        else if (realloc)
        {
            std::cout << "Reallocation occured, applying thread affinity" << std::endl;
            exec->apply_callback_to_thread_assignment();
        }
        std::this_thread::sleep_for(period);

    } while (exec->is_running());
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

void MPCController::update_tc_utilization(std::shared_ptr<threadclass> tc, State current_state)
{
    double total_utilization = 0;
    for (auto &chain : tc->chains)
    {
        total_utilization += compute_chain_utilization(chain, current_state);
    }
    tc->utilization = total_utilization / (tc->total_budget / THREAD_PERIOD * tc->threads.size());
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
            response_times[i] = 20e7;
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
            else if (delta > (double)(1000000)) // 10 seconds
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
                    // delta = delta_prev + 1;
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
        // if (response_times[i] == -1)
        if (response_times[i] > 20e6 - 100) // don't compare with 20e7; double is inaccurate
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

void MPCController::create_threadclass(void)
{
    unsigned int analysis_count = 0;
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
            rt_tg->apply_budgets(THREAD_PERIOD - 10240);
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
            be_tg->apply_budgets(10240);
            threadclasses.push_back(be_tg);
        }
    }
    std::cout << "Threadclasses created" << std::endl;
    // this->reallocate_chains(&analysis_count);
    std::cout << "Parsing and sorting chains" << std::endl;
    std::vector<std::vector<std::shared_ptr<Chain>>> split_chainsets = exec->parse_and_sort_chains(exec->get_chains());
    std::vector<std::shared_ptr<Chain>> rt_chains = split_chainsets[0];
    unsigned int num_rt_chains = rt_chains.size();
    // count the number of RT capable threadgroups
    for (auto &tg : threadclasses)
    {
        if (tg->rt_threadclass == true)
        {
            tg->add_chain(rt_chains.at(num_rt_chains - 1));
            num_rt_chains--;
            if (num_rt_chains < 0)
            {
                return;
            }
        }
    }
}

void MPCController::reallocate_chains(unsigned int *analysis_count)
{
    // for each threadclass
    for (auto &tg : this->threadclasses)
    {
        tg->chains.clear();
        tg->set_utilization(0);
    }
    // start reallocation
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
        std::cout << "Trying to add chain " << chain->getChainID() << " to RT threadgroup " << min_tg->id << " with utilization: " << min_tg->get_utilization() << std::endl;
        // check the schedulability of the chainset with the threadgroup
        std::cout << "Computing chain response times" << std::endl;
        *analysis_count += 1;
        auto response_times = pwa_cd(existing_chains, min_tg, min_tg->total_budget / NS_IN_US, current_state);
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
            std::cout << "RT Chain " << chain->getChainID() << " is schedulable. Adding chain to RT threadgroup: " << min_tg->id << " with new utilization: " << min_tg->get_utilization() << std::endl;
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
        std::cout << "Trying to add chain " << chain->getChainID() << " to BE threadgroup " << min_tg->id << " with utilization: " << min_tg->get_utilization() << std::endl;
        // since the budget is 0, the response time analysis will result in infinite loop.
        // we can skip the response time analysis for now
        min_tg->add_chain(chain);
        min_tg->set_utilization(min_util + compute_chain_utilization(chain, current_state));

        std::cout << "BE Chain " << chain->getChainID() << " added to BE threadgroup: " << min_tg->id << " with new utilization: " << min_tg->get_utilization() << std::endl;
    }
}

bool MPCController::reduce_rt_budget(std::shared_ptr<threadclass> tc, const State &current_state, unsigned int *analysis_count)
{
    // State current_state = exec->get_state();
    //  basically we need to minimize the budget of the RT threadclass such that all the RT chains are still schedulable
    //  we can do this by performing a binary search on the budget and evaluating the schedulability of the chainset
    //  if the chainset is schedulable, we can reduce the budget further
    //  if the chainset is not schedulable, we can increase the budget
    //  we can start with the current budget of the threadclass

    // the calculations are with reference to THREAD_PERIOD whose units are nanoseconds
    // to scale the budget given to the analysis, we divide by 10000 to change our units to us from 10ms
    int min_budget = 1024 * 10, max_budget = THREAD_PERIOD, mid_budget = 0;
    volatile int computed_budget = 0;
    int current_budget = tc->total_budget;
    int best_budget = 1024 * 10;
    std::vector<std::shared_ptr<Chain>> chainset = tc->get_chains();
    std::vector<struct timeval> response_times;
    std::vector<struct timeval> deadlines;
    std::vector<struct timeval> best_response_times;
    bool realloc = false;
    bool schedulable = true;

    if (chainset.size() == 0)
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
        // std::cout << "Trying to reduce budget to: " << mid_budget << std::endl;
        *analysis_count += 1;
        // mid budget is in nanoseconds
        // we hvave to convert it to a proportion of thread period, but in microseconds
        response_times = pwa_cd(chainset, tc, mid_budget / NS_IN_US, current_state);

        // int i = 0;
        // for (auto &chain : chainset)
        // {
        //     auto chain_response_time = 0;
        //     // for (auto &callback : chain->getCallbacks())
        //     // {
        //     //     chain_response_time += callback->getExecutionTime(current_state).tv_sec * 1e6 + callback->getExecutionTime(current_state).tv_usec;
        //     // }
        //     auto temp_chain = chain->getFirstCallback()->getChain();
        //     chain_response_time = temp_chain->getChainResponseTime(0);
        //     /* FIXME: map entry cannot be found by current_state...
        //     std::deque<struct timeval> dq = chain->get_branch_response_time_history(current_state, 0); // all chains linear at this point
        //     for (auto& val : dq)
        //         chain_response_time += val.tv_sec * 1e6 + val.tv_usec;
        //     if (dq.size()) chain_response_time /= dq.size();
        //     */

        //     // std::cout << "Chain Measured Response Time: " << chain_response_time << std::endl;
        //     // std::cout << "Chain Deadline: " << deadlines[i].tv_sec * 1e6 + deadlines[i].tv_usec << std::endl;
        //     // std::cout << "Chain Predicted Response Time: " << response_times[i].tv_sec * 1e6 + response_times[i].tv_usec << std::endl;
        //     i++;
        // }

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
        schedulable = true;
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
            best_response_times.clear();
            for (auto &rt : response_times)
            {
                best_response_times.push_back(rt);
            }
            // best_budget = mid_budget;
        }
        else
        {
            // min_budget = mid_budget + 1000; // increment by 1us
            min_budget = mid_budget + 100000; // increment by 200us
            if (mid_budget > max_budget)
            {
                best_budget = mid_budget;
            }
        }
    }
    best_budget = std::max(min_budget, std::min(mid_budget, max_budget));
    // if(best_budget != computed_budget){
    //     std::cerr << "Computed budget: " << computed_budget << " Best budget: " << best_budget << std::endl;
    //     schedulable = false;
    // }
    // best_budget*=1000;
    // we only care if the best budget is schedulable
    // basically if the budget is < max, it is schedulable
    if (best_budget <= THREAD_PERIOD)
    {
        schedulable = true;
        int i = 0;
        for (auto &chain : chainset)
        {
            std::cout << "Chain: " << chain->getChainID() << " Prio: " << chain->getBranchPriority(0) << " Estimated WCRT: " << best_response_times[i].tv_sec * 1e6 + best_response_times[i].tv_usec << " us" << " With budget: " << best_budget << " On RT threadclass: " << tc->id << std::endl;
        }
    }
    else
    {
        schedulable = false;
    }

    static int tries = 0;
    if (schedulable == false)
    {
        std::cout << "Chainset for threadclass " << tc->id << " is not schedulable with budget " << best_budget << std::endl;
        tries++;
        if (tries == 2)
        {
            std::cerr << "Chainset for threadclass " << tc->id << " is not schedulable after reallocation. Demoting chain to BE." << std::endl;
            tries = 0;
            // demote the chain to BE
            // remove the chain from the rt list and add it to the be list and try again
            // Find the least critical chain and demote it to BE
            std::shared_ptr<Chain> least_critical_chain = nullptr;
            for (auto &chain : chainset)
            {
                if (least_critical_chain == nullptr)
                {
                    least_critical_chain = chain;
                }
                else if (chain->getBranchPriority(0) < least_critical_chain->getBranchPriority(0))
                {
                    least_critical_chain = chain;
                }
                else if (least_critical_chain->getBranchPriority(0) == 0 && chain->getBranchPriority(0) != 0)
                {
                    least_critical_chain = chain;
                }
            }
            tc->remove_chain(least_critical_chain);

            // find the be threadclass
            std::shared_ptr<threadclass> be_tc = nullptr;
            double min_util = 1.0;
            for (auto &threadclass : threadclasses)
            {
                if (threadclass->get_utilization() < min_util && threadclass->rt_threadclass == false)
                {
                    be_tc = threadclass;
                    min_util = threadclass->get_utilization();
                }
            }
            be_tc->add_chain(least_critical_chain);
            be_tc->set_utilization(be_tc->get_utilization() + compute_chain_utilization(least_critical_chain, current_state));
            std::cout << "Demoted chain " << least_critical_chain->getChainID() << " to BE threadclass " << be_tc->id << " with utilization: " << be_tc->get_utilization() << std::endl;
            realloc = true;
        }
        else
        {
            std::cerr << "Chainset for threadclass " << tc->id << " is not schedulable after reallocation. Trying to reallocate chains." << std::endl;
            realloc = true;
            reallocate_chains(analysis_count);
            return realloc;
        }
    }
    // now assign the budget to the threadclass

empty_rt_chain:
    tc->apply_budgets(best_budget);
    // now we need to set the budget for each thread in the threadclass
    if (tc->rt_threadclass)
    {
        std::cout << "Setting budget for RT threadclass " << tc->id << " (" << tc->threads.size() << " threads)" << std::endl;
    }
    else
    {
        std::cout << "Setting budget for BE threadclass " << tc->id << " (" << tc->threads.size() << " threads)" << std::endl;
    }
    for (auto &thread : tc->threads)
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        attr.sched_policy = SCHED_DEADLINE;
        attr.sched_runtime = best_budget;
        attr.sched_period = THREAD_PERIOD;
        attr.sched_deadline = THREAD_PERIOD;
        if (tc->rt_threadclass)
        {
            attr.sched_flags = 0;
        }
        else
        {
            attr.sched_flags = 0 | SCHED_FLAG_RECLAIM;
        }
        // attr.sched_flags = 0;
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
        return false;
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
    *analysis_count += 1;
    auto be_response_times = pwa_cd(be_tc->get_chains(), be_tc, be_tc->total_budget / NS_IN_US, current_state);
    std::cout << "BE Threadclass " << be_tc->id << " response times: " << std::endl;
    for (size_t i = 0; i < be_response_times.size(); i++)
    {
        if (be_response_times[i].tv_sec * 1e6 + be_response_times[i].tv_usec > 20e7 - 100) // don't compare with 20e7; double is inaccurate
        {
            std::cerr << "BE Chain " << i << " is not starvation free" << std::endl;
        }
        else
        {
            std::cout << "BE Chain " << i << " is starvation free" << std::endl;
            std::cout << "Estimated Response Time for Chain " << i << ": " << be_response_times[i].tv_sec << "s " << be_response_times[i].tv_usec << "us" << std::endl;
        }
    }
    return realloc;
    //}
    //}
}

#endif // MPC_CONTROLLER_CPP
