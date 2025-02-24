#ifndef MPC_CONTROLLER_CPP
#define MPC_CONTROLLER_CPP

#include <mpccontroller.hpp>
// #define THREAD_PERIOD 10000000 // 10ms
// #define THREAD_PERIOD_US 10000 // 10ms
#define THREAD_PERIOD 5000000 // 5ms
#define THREAD_PERIOD_US 5000 // 5ms
#define US_OFFSET 5120
#define NS_IN_US 1000
// #define THREAD_PERIOD_US 1000
// #define THREAD_PERIOD 1000000 // 1ms
// #define US_OFFSET 10240

MPCController::MPCController()
{
    int threadpool_size = 4;

    for (int i = 0; i < threadpool_size; i++)
    {
        // Allocate a new atomic bool on the heap
        thread_complete.push_back(std::make_unique<std::atomic<bool>>(true));

        // Pass a reference to the worker thread
        analysis_threadpool.push_back(std::make_shared<std::thread>(
            [this, i](std::atomic<bool> &complete)
            {
                // Set CPU affinity from within the thread
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(4, &cpuset);
                CPU_SET(5, &cpuset);
                CPU_SET(6, &cpuset);
                CPU_SET(7, &cpuset);

                if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0)
                {
                    std::cerr << "Failed to set CPU affinity: " << strerror(errno) << std::endl;
                    return;
                }

                // Now run the actual worker thread function
                this->worker_thread_run(complete);
            },
            std::ref(*thread_complete[i])));
    }
}

MPCController::~MPCController() {}
static inline void timespec_to_timeval(struct timespec *ts, struct timeval *tv)
{
    tv->tv_sec = ts->tv_sec;
    tv->tv_usec = ts->tv_nsec / 1000;
}

void MPCController::assign_executor(executor *exec)
{
    this->exec = exec;
    this->cv_ptr = std::make_shared<std::condition_variable>();
    this->mtx_ptr = std::make_shared<std::mutex>();
    this->exec->assign_cv(cv_ptr, mtx_ptr);
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
    threadclasses[0]->set_utilization(compute_chain_utilization(chain0));
    threadclasses[1]->set_utilization(compute_chain_utilization(chain1));
    exec->enable_callback_priority();
    exec->start();

    exec->apply_callback_to_thread_assignment();
}
double arbitrary_interference(double delta, double alpha, double T, double C)
{

    return std::ceil((delta + alpha) / T) * C;
    // if ((delta - std::floor(double((delta + alpha)) / T) * T) < 0)
    // {
    //     return std::floor(double((delta + alpha)) / T) * C + C;
    // }
    // else
    // {
    //     return std::floor(double((delta + alpha)) / T) * C + std::min(C, delta - std::floor(double((delta + alpha)) / T) * T);
    // }
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

void MPCController::verify_starvation_freedom(std::shared_ptr<threadclass> be_tc)
{
    std::cout << "Verifying starvation freedom for BE threadclass " << be_tc->id << std::endl;

    // the threadclass is assumed to be BE, check the response times of the chains and see if theyre bounded
    int i = 0;
    static int tries = 0;
    auto prev_response_times = be_tc->chain_response_times;
    bool use_ad_analysis = false;
    for (unsigned long int j = 0; j < prev_response_times.size(); j++)
    {
        struct timeval test_tv = be_tc->get_chains()[j]->getDeadline();
        if (timercmp(&prev_response_times[j], &test_tv, >) || (prev_response_times[j].tv_sec < 0 || prev_response_times[j].tv_usec < 0))
        {
            use_ad_analysis = true;
            break;
        }
    }
    std::vector<timeval> be_response_times;

    if (use_ad_analysis)
    {
        be_response_times = pwa_ad(be_tc->chains, be_tc, be_tc->total_budget / NS_IN_US);
    }
    else
    {
        be_response_times = pwa_cd(be_tc->chains, be_tc, be_tc->total_budget / NS_IN_US);
    }
    // auto be_response_times = pwa_ad(be_tc->chains, be_tc, be_tc->total_budget / NS_IN_US);
    // be_tc->chain_response_times = be_response_times;

    std::cout << "BE Threadclass " << be_tc->id << " response times: " << std::endl;
    for (size_t i = 0; i < be_response_times.size(); i++)
    {
        if (be_response_times[i].tv_sec * 1e6 + be_response_times[i].tv_usec > 20e7 - 100 || be_response_times[i].tv_sec * 1e6 + be_response_times[i].tv_usec <= 0) // don't compare with 20e7; double is inaccurate
        {
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " on threadclass: " << be_tc->id << " is not starvation free" << std::endl;
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " WCRT: " << be_response_times[i].tv_sec * 1000000 + be_response_times[i].tv_usec << std::endl;

            // std::cout << "BE Chain " << i << " is not starvation free" << std::endl;
        }
        else
        {
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " on threadclass: " << be_tc->id << " is starvation free" << std::endl;
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " WCRT: " << be_response_times[i].tv_sec * 1000000 + be_response_times[i].tv_usec << std::endl;
            // std::cout << "BE Chain " << i << " is starvation free" << std::endl;
            // std::cout << "Estimated Response Time for Chain " << i << ": " << be_response_times[i].tv_sec << "s " << be_response_times[i].tv_usec << "us" << std::endl;
        }
    }
    // for (auto &rt : chain_response_times)
    // {
    //     auto temp_rt = rt.tv_sec * 1e6 + rt.tv_usec;
    //     std::cout << "Chain: " << i++ << " Estimated response time: " << temp_rt << std::endl;
    //     if (temp_rt >= 20e6 - 100 || temp_rt <= 0)
    //     {
    //         std::cout << "Starvation detected in BE threadclass " << tc->id << std::endl;
    //         // if(tries == 0){
    //         // reallocate_be_chains();
    //         // tries++;
    //         // }
    //     }
    // }
}

void MPCController::worker_thread_run(std::atomic<bool> &complete)
{
    std::shared_ptr<threadclass> tc;
    unsigned int analysis_counter = 0;

    while (exec->is_running())
    {
        {
            std::unique_lock<std::mutex> lock(work_mtx);
            work_cv.wait(lock, [this]
                         { return !work_queue.empty() || !exec->is_running(); });

            if (!work_queue.empty())
            {
                tc = work_queue.front();
                work_queue.pop_front();
                complete.store(false, std::memory_order_release);
                work_cv.notify_all();
            }
            else
            {
                continue; // No work to do, go back to waiting
            }
        } // Release lock before processing

        // Process single task outside critical section
        std::cout << "Worker thread " << syscall(SYS_gettid)
                  << " processing threadclass " << tc->id << std::endl;

        if (tc->rt_threadclass)
        {
            reduce_rt_budget(tc, &analysis_counter, 0);
        }
        else
        {
            update_tc_utilization(tc);
            verify_starvation_freedom(tc);
        }

        complete.store(true, std::memory_order_release);
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
    // this->exec->print_chains_and_callbacks();
    bool first_run = true;
    do
    {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
        bool realloc = false;
        // gettimeofday(&start_time, NULL);
        //  Get the current state from the executor
        // Create a new state for the next prediction
        unsigned int analysis_count = 0;
        // Check timing violations and adjust the budget
        for (auto &tc : threadclasses)
        {
            if (tc->rt_threadclass)
            {
                if (tc->chains.size() == 0 && first_run)
                {
                    std::lock_guard<std::mutex> lock(work_mtx);
                    work_queue.push_back(tc);
                    work_cv.notify_all();
                    // realloc = reduce_rt_budget(tc, &analysis_count, 0) ? true : false;
                }
                for (auto &chain : tc->chains)
                {
                    std::cout << "Controller evaluating chain " << chain->getChainID() << std::endl;

                    uint64_t chain_response_time = 0;
                    // for (auto &callback : chain->getCallbacks())
                    // {
                    //     auto temp_rt = callback->getExecutionTime(current_state).tv_sec * 1e6 + callback->getExecutionTime(current_state).tv_usec;
                    //     chain_response_time += temp_rt;
                    //     std::cout << "Callback " << callback->getName() << " response time: " << temp_rt << std::endl;
                    // }
                    chain_response_time = chain->getFirstCallback()->getChain()->getChainResponseTime(0);
                    std::cout << "Chain " << chain->getChainID() << " response time: " << chain_response_time << std::endl;
                    std::cout << "Chain " << chain->getChainID() << " deadline: " << chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec << std::endl;
                    if (chain_response_time > chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec && chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec != 0)
                    {
                        std::cout << "Timing violation detected in RT chain " << chain->getChainID() << std::endl;
                        std::cout << "Response time: " << chain_response_time << " Deadline: " << chain->getDeadline().tv_sec * 1e6 + chain->getDeadline().tv_usec << std::endl;
                        std::lock_guard<std::mutex> lock(work_mtx);
                        work_queue.push_back(tc);
                        work_cv.notify_all();
                        // realloc = reduce_rt_budget(tc, &analysis_count, tc->total_budget) ? true : false;
                    }
                    else if (first_run)
                    {
                        // realloc = true;
                        std::lock_guard<std::mutex> lock(work_mtx);
                        work_queue.push_back(tc);
                        work_cv.notify_all();
                        // realloc = reduce_rt_budget(tc, &analysis_count, 0) ? true : false;
                    }
                }
            }
            else
            {
                analysis_count++;
                std::lock_guard<std::mutex> lock(work_mtx);
                work_queue.push_back(tc);
                work_cv.notify_all();
                // update_tc_utilization(tc);
                // verify_starvation_freedom(tc);
            }
        }
        while (!work_queue.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        for (auto &complete_ptr : thread_complete)
        {
            while (!complete_ptr->load(std::memory_order_acquire))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // gettimeofday(&end_time, NULL);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
        timespec_to_timeval(&start, &start_time);
        timespec_to_timeval(&end, &end_time);
        timersub(&end_time, &start_time, &end_time);

        std::cout << "Controller activation time: " << end_time.tv_sec * 1e6 + end_time.tv_usec << " us. Number of times analysis was performed: " << analysis_count << std::endl;

        // IMPORTANT: Apply callback-to-thread assignment to rclcpp
        if (first_run)
        {
            exec->enable_callback_priority();
            exec->start();
            // exec->apply_callback_to_thread_assignment();
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
    std::cerr << "Invalid x value" << std::endl;
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

double MPCController::MLP(double M, std::vector<std::shared_ptr<Chain>> chainset, int self_priority, double delta)
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

                double b = std::min((double)(chainset[k]->getCallbacks()[j]->getExecutionTime().tv_sec * 1e6 + chainset[k]->getCallbacks()[j]->getExecutionTime().tv_usec - 1), delta);
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

void MPCController::update_tc_utilization(std::shared_ptr<threadclass> tc)
{
    double total_utilization = 0;
    for (auto &chain : tc->chains)
    {
        total_utilization += compute_chain_utilization(chain);
    }
    tc->utilization = total_utilization / (tc->total_budget / THREAD_PERIOD * tc->threads.size());
}

// create template to convert all types (int, double, long, etc) into timevals
template <typename T>
struct timeval convert_to_timeval(T value)
{
    struct timeval tv;
    tv.tv_sec = static_cast<int>(value) / 1000000;
    tv.tv_usec = static_cast<int>(value) % 1000000;
    return tv;
}

double timeval_to_double(struct timeval tv)
{
    return tv.tv_sec * 1e6 + tv.tv_usec;
}

struct timeval MPCController::do_partial_ad_analysis(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget, std::shared_ptr<Chain> root_chain, std::shared_ptr<Callback> nl_cb_ptr)
{
    double M = tg->threads.size();
    auto k = root_chain->getChainID(); // we want to do a partial analysis on the root chain
    double delta = 1;
    auto E_k = 1;
    auto MSG_DELAY = 200, QUEUE_DELAY = 0;
    auto response_time = 0.0;
    bool nonlinear_cb = true;
    for (auto &callback : root_chain->getCallbacks())
    {
        if (callback->getPlaceInChain() != root_chain->getNumCallbacks() - 1) // this assumes linear chains only, no branches
        {
            auto ex_time = callback->getExecutionTime();
            E_k += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
        }
    }
    auto chain = root_chain;
    while (true && this->exec->is_running())
    {
        double W = 0.0;

        // initialize W, intf
        double intf = 0.0;
        // interference from all interfering chains
        // for each interfering chain in the same chainset
        for (auto &interf_chain : chainset)
        {
            {
                if (!this->exec->is_running())
                {
                    break;
                }
                // calculate the interference
                // T is the period of the interfering chain
                auto T = interf_chain->getPeriod().tv_sec * 1e6 + interf_chain->getPeriod().tv_usec;
                // D is the deadline of the interfering chain
                auto D = 2 * T; // interf_chain->getDeadline().tv_sec * 1e6 + interf_chain->getDeadline().tv_usec;
                // C is the total execution time of all callbacks in the interfering chain
                auto C = 0;
                // for all callbacks in the interfering chain
                double Rj = timeval_to_double(interf_chain->get_Rj());
                double alpha = 0;

                for (auto &callback : interf_chain->getCallbacks())
                {
                    auto ex_time = callback->getExecutionTime();
                    C += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
                }
                if (abs(Rj) < 1e-6 || Rj < C)
                {
                    alpha = D - C;
                }
                else
                {
                    alpha = Rj - C;
                }
                // if the interfering chain is a RT chain
                if (tg->rt_threadclass)
                {
                    // assuming linear chains only: if the interfering chain has a higher priority than the current chain
                    if (interf_chain->getBranchPriority(0) > chain->getBranchPriority(0))
                    {
                        // calculate the interference
                        if (alpha <= 0)
                        {
                            std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                        }
                        intf += arbitrary_interference(delta, alpha, T, C);
                    }
                }
                else // if the interfering chain is a BE chain
                {
                    if (alpha <= 0)
                    {
                        std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                    }
                    // calculate the interference
                    intf += arbitrary_interference(delta, alpha, T, C);
                }
            }
        }
        // calculate the response time
        // if the threadclass is a BE threadclass
        double E_C = 0;
        for (auto &callback : chain->getCallbacks())
        {
            auto ex_time = callback->getExecutionTime();
            E_C += ex_time.tv_sec * 1e6 + ex_time.tv_usec;
        }
        if (!tg->rt_threadclass)
        {

            W = M * double(E_k) + intf - E_C;
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
            double B = MLP(M, exclusive_chainset, chain->getBranchPriority(0), delta);
            // W = M * E_k + intf + B
            W = M * double(E_k) + intf + B - E_C;
        }
        auto sbfd = std::ceil(sbf(delta, budget, THREAD_PERIOD_US));
        // if W is negative, increment delta
        if (W < 0)
        {
            delta += 1000;
        }
        // else if (W < M * delta)
        else if (W < M * sbfd) // change for period and budget
        {
            // if W is less than M * delta, set the response time and delta
            // x is the execution time of the last callback in the chain - 1
            auto x = (double)chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_sec * 1e6 + chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_usec - 1 + MSG_DELAY + QUEUE_DELAY;
            // the response times are calculated as the sum of the execution time of the last callback in the chain and the pseudo-inverse of the SBF function
            response_time = delta + pseudo_inv_sbf(x, budget, THREAD_PERIOD_US); // units are in microseconds, not nanoseconds
            chain->set_Rj(convert_to_timeval(response_time));
            // if the chain has a callback that is a branch root, then let's store the upper bound on RT for the root chain on that root callback
            // std::cout << "Setting Chain: " << chain->getChainID() << " Response Time: " << response_time << std::endl;
            // if (nonlinear_cb)
            // {
            //     std::cout << "Setting Root time for callback: " << nl_cb_ptr->getName() << " to: " << response_time << std::endl;
            //     std::cout << "Full chain response time: " << response_time << std::endl;
            //     //std::cout << "Trunk of chain execution time: " << trunk_ex_time << std::endl;
            //     std::cout << "Callback Place in chain: " << nl_cb_ptr->getPlaceInChain() << std::endl;
            // }
            if (nonlinear_cb) // k is chain id, nonlinear_id is a callback id
            {
                // the root time will be the response time minus the execution time of the callbacks after the branch root
                auto root_time = convert_to_timeval(response_time);
                nl_cb_ptr->root_rt = root_time;
                // nl_cb_ptr->root_rt.tv_sec = root_time / 1e6;
                // nl_cb_ptr->root_rt.tv_usec = root_time - nl_cb_ptr->root_rt.tv_sec * 1e6;
            }
            // update delta value
            // delta_values[k] = delta;
            k++;
            break;
        }
        else if (delta > (double)(20000000))
        { // higher limit
            // not schedulable
            response_time = 20e6;
            // delta_values[k] = delta;

            if (nonlinear_cb)
            {
                // the root time will be the response time minus the execution time of the callbacks after the branch root
                struct timeval root_time = convert_to_timeval(response_time);
                nl_cb_ptr->root_rt = root_time;
                // nl_cb_ptr->root_rt.tv_sec = root_time / 1e6;
                // nl_cb_ptr->root_rt.tv_usec = root_time - nl_cb_ptr->root_rt.tv_sec * 1e6;
            }
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
                delta = delta_prev + 200;
                // delta += std::floor(W/M);
            }
        }
    }
    return convert_to_timeval(response_time);
}

struct timeval MPCController::do_partial_cd_analysis(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget, std::shared_ptr<Chain> root_chain, std::shared_ptr<Callback> nl_cb_ptr)
{
    double M = tg->threads.size();
    auto k = root_chain->getChainID(); // we want to do a partial analysis on the root chain
    double delta = 1;
    auto E_k = 1;
    auto MSG_DELAY = 200, QUEUE_DELAY = 0;
    auto response_time = 0.0;
    bool nonlinear_cb = true;
    for (auto &callback : root_chain->getCallbacks())
    {
        if (callback->getPlaceInChain() != root_chain->getNumCallbacks() - 1) // this assumes linear chains only, no branches
        {
            auto ex_time = callback->getExecutionTime();
            E_k += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
        }
    }
    auto chain = root_chain;
    while (true && this->exec->is_running())
    {
        double W = 0.0;

        // initialize W, intf
        double intf = 0.0;
        // interference from all interfering chains
        // for each interfering chain in the same chainset
        for (auto &interf_chain : chainset)
        {
            if (!this->exec->is_running())
            {
                break;
            }
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
                double Rj = timeval_to_double(interf_chain->get_Rj());
                double alpha = 0;

                for (auto &callback : interf_chain->getCallbacks())
                {
                    auto ex_time = callback->getExecutionTime();
                    C += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
                }
                if (abs(Rj) < 1e-6)
                {
                    alpha = D - C;
                }
                else
                {
                    alpha = Rj - C;
                }
                // if the interfering chain is a RT chain
                if (tg->rt_threadclass)
                {
                    // assuming linear chains only: if the interfering chain has a higher priority than the current chain
                    if (interf_chain->getBranchPriority(0) > chain->getBranchPriority(0))
                    {
                        // calculate the interference
                        if (alpha <= 0)
                        {
                            std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                        }
                        intf += interference(delta, alpha, T, C);
                    }
                }
                else // if the interfering chain is a BE chain
                {
                    if (alpha <= 0)
                    {
                        std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                    }
                    // calculate the interference
                    intf += interference(delta, alpha, T, C);
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
            double B = MLP(M, exclusive_chainset, chain->getBranchPriority(0), delta);
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
            auto x = (double)chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_sec * 1e6 + chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_usec - 1 + MSG_DELAY + QUEUE_DELAY;
            // the response times are calculated as the sum of the execution time of the last callback in the chain and the pseudo-inverse of the SBF function
            response_time = delta + pseudo_inv_sbf(x, budget, THREAD_PERIOD_US); // units are in microseconds, not nanoseconds
            chain->set_Rj(convert_to_timeval(response_time));
            // if the chain has a callback that is a branch root, then let's store the upper bound on RT for the root chain on that root callback
            // std::cout << "Setting Chain: " << chain->getChainID() << " Response Time: " << response_time << std::endl;
            // if (nonlinear_cb)
            // {
            //     std::cout << "Setting Root time for callback: " << nl_cb_ptr->getName() << " to: " << response_time << std::endl;
            //     std::cout << "Full chain response time: " << response_time << std::endl;
            //     //std::cout << "Trunk of chain execution time: " << trunk_ex_time << std::endl;
            //     std::cout << "Callback Place in chain: " << nl_cb_ptr->getPlaceInChain() << std::endl;
            // }
            if (nonlinear_cb) // k is chain id, nonlinear_id is a callback id
            {
                // the root time will be the response time minus the execution time of the callbacks after the branch root
                auto root_time = convert_to_timeval(response_time);
                nl_cb_ptr->root_rt = root_time;
                // nl_cb_ptr->root_rt.tv_sec = root_time / 1e6;
                // nl_cb_ptr->root_rt.tv_usec = root_time - nl_cb_ptr->root_rt.tv_sec * 1e6;
            }
            // update delta value
            // delta_values[k] = delta;
            k++;
            break;
        }
        else if (delta > (double)(20000000))
        { // higher limit
            // not schedulable
            response_time = 20e6;
            // delta_values[k] = delta;

            if (nonlinear_cb)
            {
                // the root time will be the response time minus the execution time of the callbacks after the branch root
                struct timeval root_time = convert_to_timeval(response_time);
                nl_cb_ptr->root_rt = root_time;
                // nl_cb_ptr->root_rt.tv_sec = root_time / 1e6;
                // nl_cb_ptr->root_rt.tv_usec = root_time - nl_cb_ptr->root_rt.tv_sec * 1e6;
            }
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
                delta = delta_prev + 200;
                // delta += std::floor(W/M);
            }
        }
    }
    return convert_to_timeval(response_time);
}
std::vector<struct timeval> MPCController::pwa_cd(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget)
{
    // M is the number of threads in the threadclass * the total budget (per thread)
    // double M = tg->threads.size() * (double)tg->total_budget / 10000;

    auto MSG_DELAY = 200;
    auto QUEUE_DELAY = 0;
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
    if (tg->rt_threadclass)
    {
        // MSG_DELAY = 600; // us -- profiling value
        // QUEUE_DELAY = 500; //us
    }
    // for each chain assigned to the chainset
    for (auto &chain : chainset)
    {
        bool nonlinear_cb = false;
        std::shared_ptr<Callback> nl_cb_ptr = nullptr; // chain->getFirstCallback();
        int nonlinear_id = 0;
        auto trunk_ex_time = 0;
        auto e_top = 0;
        // Initialize Delta and E_k
        double delta = 1;
        auto E_k = 1;

        // check to see whether the chain has a nonlinear callback this should be checked for each chain
        for (auto &callback : chain->getCallbacks())
        {
            if (callback->getPlaceInChain() != chain->getNumCallbacks() - 1) // this assumes linear chains only, no branches -- fixed/////**** */
            {
                // std::cout << "Callback: " << callback->getName() << " Is nonLinear: " << callback->is_non_linear() << std::endl;
                //  if this is a branch root callback

                // the state-wise WCET time of the callback
                // auto ex_time = callback->getExecutionTime(exec->get_state());
                auto ex_time = callback->getExecutionTime();
                // Chain K's execution time is the sum of all callback execution times (except the last callback)
                E_k += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
            }
            if (callback->is_non_linear())
            {
                nonlinear_cb = true;
                nonlinear_id = callback->getPlaceInChain();
                nl_cb_ptr = callback;
                // std::cout << "Nonlinear callback found: " << callback->getName() << std::endl;
                // std::cout << "Nonlinear ID: " << nonlinear_id << std::endl;
            }
        }

        // if we detected a nonlinear callback in the chain, and we are not performing a partial analysis
        if (nonlinear_cb)
        {
            // we need to make a fake chain for this partial analysis and inject it into the chainset
            // we also need to make a fake chainset for the partial analysis
            std::vector<std::shared_ptr<Chain>> partial_chainset;
            for (auto &candidate_chain : chainset)
            {
                if (candidate_chain != chain)
                {
                    partial_chainset.push_back(candidate_chain);
                }
            }
            // now we need to take the current chain, and make a fake chain that only includes the root of the chain, up until the nonlinear callback
            auto partial_chain = std::make_shared<Chain>(chain->getChainID());
            for (auto &callback : chain->getCallbacks())
            {
                if (callback->getPlaceInChain() <= nonlinear_id)
                {
                    partial_chain->addCallback(callback, false);
                }
            }
            partial_chain->setPeriod(chain->getPeriod());
            partial_chain->setDeadline(chain->getDeadline());
            partial_chain->setBranchPriority(chain->getBranchPriority(0), 0);
            partial_chain->setLatencyTarget(chain->get_branch_latency_target(0), 0, chain->get_branch_rt(0)); // this lets it know what analysis to use
            // set branch rt and other things to avoid segfault
            std::vector<int> chain_criticalities;
            chain_criticalities.push_back(0);
            partial_chain->setPriorities(chain_criticalities);
            partial_chainset.push_back(partial_chain);

            // std::cout << "Performing partial analysis on chain: " << chain->getChainID() << " with root callback: " << nl_cb_ptr->getName() << std::endl;
            struct timeval root_time;
            if (partial_chain->get_branch_rt(0) || tg->chain_response_times.size() == 0)
            {
                root_time = do_partial_cd_analysis(partial_chainset, tg, budget, partial_chain, nl_cb_ptr);
            }
            else
            {
                root_time = do_partial_ad_analysis(partial_chainset, tg, budget, partial_chain, nl_cb_ptr);
            }
            // std::cout << "Partial analysis resulted in a partial chain response time of: " << root_time.tv_sec * 1e6 + root_time.tv_usec << std::endl;

            nl_cb_ptr->root_rt = root_time;
            // for (auto &chain : partial_chainset){
            //     chain->printChain();
            //     chain->printCallbacks();
            // }
            // now we need to perform the partial analysis on the chain
            // // if the chain is nonlinear, we need to find the execution time of the trunk
            // for (auto &callback : chain->getCallbacks())
            // {
            //     if (callback->getPlaceInChain() > nonlinear_id)
            //     {
            //         auto ex_time = callback->getExecutionTime();
            //         trunk_ex_time += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
            //     }
            // }
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
                if (!this->exec->is_running())
                {
                    break;
                }
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
                    double Rj = timeval_to_double(interf_chain->get_Rj());
                    double alpha = 0;
                    // for all callbacks in the interfering chain
                    for (auto &callback : interf_chain->getCallbacks())
                    {
                        auto ex_time = callback->getExecutionTime();
                        C += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
                    }
                    if (abs(Rj) < 1e-6 || Rj < C)
                    {
                        alpha = D - C;
                    }
                    else
                    {
                        alpha = Rj - C;
                    }
                    // if the interfering chain is a RT chain
                    if (tg->rt_threadclass)
                    {
                        // assuming linear chains only: if the interfering chain has a higher priority than the current chain
                        if (interf_chain->getBranchPriority(0) > chain->getBranchPriority(0))
                        {
                            // calculate the interference
                            if (alpha <= 0)
                            {
                                std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                            }
                            intf += interference(delta, alpha, T, C);
                        }
                    }
                    else // if the interfering chain is a BE chain
                    {
                        if (alpha <= 0)
                        {
                            std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                        }
                        // calculate the interference
                        intf += interference(delta, alpha, T, C);
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
                double B = MLP(M, exclusive_chainset, chain->getBranchPriority(0), delta);
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
                auto x = (double)chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_sec * 1e6 + chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_usec - 1 + MSG_DELAY + QUEUE_DELAY;
                // the response times are calculated as the sum of the execution time of the last callback in the chain and the pseudo-inverse of the SBF function
                response_times[k] = delta + pseudo_inv_sbf(x, budget, THREAD_PERIOD_US); // units are in microseconds, not nanoseconds
                chain->set_Rj(convert_to_timeval(response_times[k]));

                // if the chain has a callback that is a branch root, then let's store the upper bound on RT for the root chain on that root callback
                // std::cout << "Setting Chain: " << chain->getChainID() << " Response Time: " << response_times[k] << std::endl;
                // if (nonlinear_cb)
                // {
                //     std::cout << "Setting Root time for callback: " << nl_cb_ptr->getName() << " to: " << response_times[k] - trunk_ex_time << std::endl;
                //     std::cout << "Full chain response time: " << response_times[k] << std::endl;
                //     std::cout << "Trunk of chain execution time: " << trunk_ex_time << std::endl;
                //     std::cout << "Callback Place in chain: " << nl_cb_ptr->getPlaceInChain() << " nonlinear cb id: " << nonlinear_id << std::endl;
                // }
                // if (nonlinear_cb) // k is chain id, nonlinear_id is a callback id
                // {
                //     // the root time will be the response time minus the execution time of the callbacks after the branch root
                //     auto root_time = response_times[k] - trunk_ex_time;
                //     /*for(auto &callback : chain->getCallbacks()){
                //         if(callback->is_non_linear()){
                //             callback->root_rt.tv_sec = root_time / 1e6;
                //             callback->root_rt.tv_usec = root_time - callback->root_rt.tv_sec * 1e6;

                //         }
                //     }
                //     */
                //     nl_cb_ptr->root_rt.tv_sec = root_time / 1e6;
                //     nl_cb_ptr->root_rt.tv_usec = root_time - nl_cb_ptr->root_rt.tv_sec * 1e6;
                // }
                // update delta value
                delta_values[k] = delta;
                k++;
                break;
            }
            else if (delta > (double)(20000000))
            { // higher limit
                // not schedulable
                response_times[k] = 20e6;
                // chain->set_Rj(convert_to_timeval(response_time));
                delta_values[k] = delta;

                // if (nonlinear_cb)
                // {
                //     // the root time will be the response time minus the execution time of the callbacks after the branch root
                //     auto root_time = response_times[k] - trunk_ex_time;
                //     nl_cb_ptr->root_rt.tv_sec = root_time / 1e6;
                //     nl_cb_ptr->root_rt.tv_usec = root_time - nl_cb_ptr->root_rt.tv_sec * 1e6;
                //     // for(auto &callback : chain->getCallbacks()){
                //     //     if(callback->is_non_linear()){
                //     //         callback->root_rt.tv_sec = root_time / 1e6;
                //     //         callback->root_rt.tv_usec = root_time - callback->root_rt.tv_sec * 1e6;

                //     //     }
                //     // }
                // }
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
                    delta = delta_prev + 200;
                    // delta += std::floor(W/M);
                }
            }
        }
    }
out:
    // for each chain, we need to see if it is a subchain of a larger chain that is not in the same threadclass
    // if it is, we need to add the response time of the larger chain to the response time of the subchain
    int chain_idx = 0;
    for (auto &chain : chainset)
    {
        struct timeval root_rt = {0, 0};
        bool subchain = false;
        for (auto &cb : chain->getCallbacks())
        {
            if (cb->branch_root_cb != nullptr)
            {
                subchain = true;
                root_rt = cb->branch_root_cb->root_rt;
                break;
            }
        }
        if (subchain)
        {
            // find the index for the chain in the response_times vector
            // int chain_idx = std::distance(chainset.begin(), std::find(chainset.begin(), chainset.end(), chain));
            // update the response time for the new chain with the logged response time of the partial root subchain
            response_times[chain_idx] += root_rt.tv_sec * 1e6 + root_rt.tv_usec;
            chain->set_Rj(convert_to_timeval(response_times[chain_idx]));
        }
        chain_idx++;
    }

    std::vector<struct timeval> return_response_times;
    for (size_t i = 0; i < chainset.size(); i++)
    {
        struct timeval response_time;
        // if (response_times[i] == -1)
        // if (response_times[i] > 20e6 - 100) // don't compare with 1e7; double is inaccurate
        // {
        //     response_time.tv_sec = -1;
        //     response_time.tv_usec = -1;
        //     return_response_times.push_back(response_time);
        //     continue;
        // }
        response_time.tv_sec = static_cast<long>(response_times[i] / 1e6);
        response_time.tv_usec = static_cast<long>(response_times[i] - response_time.tv_sec * 1e6);
        return_response_times.push_back(response_time);
    }
    tg->chain_response_times = return_response_times;
    return return_response_times;
}

// arbitrary deadline analysis for a chainset
std::vector<struct timeval> MPCController::pwa_ad(std::vector<std::shared_ptr<Chain>> chainset, std::shared_ptr<threadclass> tg, int budget)
{

    auto MSG_DELAY = 200;
    auto QUEUE_DELAY = 0;
    double M = (double)tg->threads.size();
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
        goto out_ad;
    }
    if (chainset.size() == 0)
    {
        std::cout << "No chains in chainset" << std::endl;
        for (size_t i = 0; i < response_times.size(); i++)
        {
            response_times[i] = 0;
        }
        goto out_ad;
    }

    for (auto &chain : chainset)
    {
        bool nonlinear_cb = false;
        std::shared_ptr<Callback> nl_cb_ptr = nullptr; // chain->getFirstCallback();
        int nonlinear_id = 0;
        auto trunk_ex_time = 0;
        double delta = 1;
        // initialize E_k to be the sum of all but the last exeution times of the chain + 1
        double E_k = 1;
        for (auto &callback : chain->getCallbacks())
        {
            if (callback->getPlaceInChain() != chain->getNumCallbacks() - 1)
            {
                auto ex_time = callback->getExecutionTime();
                E_k += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY + QUEUE_DELAY;
            }
            if (callback->is_non_linear())
            {
                nonlinear_cb = true;
                nonlinear_id = callback->getPlaceInChain();
                nl_cb_ptr = callback;
                // std::cout << "Nonlinear callback found: " << callback->getName() << std::endl;
                // std::cout << "Nonlinear ID: " << nonlinear_id << std::endl;
            }
        }
        if (nonlinear_cb)
        {
            // we need to make a fake chain for this partial analysis and inject it into the chainset
            // we also need to make a fake chainset for the partial analysis
            std::vector<std::shared_ptr<Chain>> partial_chainset;
            for (auto &candidate_chain : chainset)
            {
                if (candidate_chain != chain)
                {
                    partial_chainset.push_back(candidate_chain);
                }
            }
            // now we need to take the current chain, and make a fake chain that only includes the root of the chain, up until the nonlinear callback
            auto partial_chain = std::make_shared<Chain>(chain->getChainID());
            for (auto &callback : chain->getCallbacks())
            {
                if (callback->getPlaceInChain() <= nonlinear_id)
                {
                    partial_chain->addCallback(callback, false);
                }
            }
            partial_chain->setPeriod(chain->getPeriod());
            partial_chain->setDeadline(chain->getDeadline());
            partial_chain->setBranchPriority(chain->getBranchPriority(0), 0);
            partial_chain->setLatencyTarget(chain->get_branch_latency_target(0), 0, chain->get_branch_rt(0)); // this lets it know what analysis to use
            // set branch rt and other things to avoid segfault
            std::vector<int> chain_criticalities;
            chain_criticalities.push_back(0);
            partial_chain->setPriorities(chain_criticalities);
            partial_chainset.push_back(partial_chain);

            // std::cout << "Performing partial analysis on chain: " << chain->getChainID() << " with root callback: " << nl_cb_ptr->getName() << std::endl;
            struct timeval root_time;
            if (partial_chain->get_branch_rt(0))
            {
                root_time = do_partial_cd_analysis(partial_chainset, tg, budget, partial_chain, nl_cb_ptr);
            }
            else
            {
                root_time = do_partial_ad_analysis(partial_chainset, tg, budget, partial_chain, nl_cb_ptr);
            }

            // std::cout << "Partial analysis resulted in a partial chain response time of: " << root_time.tv_sec * 1e6 + root_time.tv_usec << std::endl;
            nl_cb_ptr->root_rt = root_time;
        }
        while (true && this->exec->is_running())
        {
            double W = 0.0;
            double intf = 0.0;
            for (auto &interf_chain : chainset)
            {
                // if (interf_chain != chain)
                {
                    if (!this->exec->is_running())
                    {
                        break;
                    }
                    auto T = interf_chain->getPeriod().tv_sec * 1e6 + interf_chain->getPeriod().tv_usec;
                    auto D = 2 * T; // interf_chain->getDeadline().tv_sec * 1e6 + interf_chain->getDeadline().tv_usec;
                    auto C = 0;
                    for (auto &callback : interf_chain->getCallbacks())
                    {
                        auto ex_time = callback->getExecutionTime();
                        C += ex_time.tv_sec * 1e6 + ex_time.tv_usec + MSG_DELAY;
                    }
                    double Rj = timeval_to_double(interf_chain->get_Rj());
                    double alpha = 0;
                    if (abs(Rj) < 1e-6 || Rj < C)
                    {
                        alpha = 2 * T - C;
                    }
                    else
                    {
                        alpha = Rj - C;
                    }
                    if (tg->rt_threadclass)
                    {
                        if (interf_chain->getBranchPriority(0) <= chain->getBranchPriority(0))
                        {
                            if (alpha <= 0)
                            {
                                std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                                std::cerr << "Alpha: " << alpha << std::endl;
                                std::cerr << "Rj: " << Rj << std::endl;
                                std::cerr << "C: " << C << std::endl;
                                std::cerr << "T: " << T << std::endl;
                                std::cerr << "D: " << D << std::endl;
                                std::cerr << "Delta: " << delta << std::endl;
                                std::cerr << "Chain ID: " << chain->getChainID() << std::endl;
                                std::cerr << "Interfering Chain ID: " << interf_chain->getChainID() << std::endl;
                                interf_chain->printChain();
                                interf_chain->printCallbacks();
                                exit(EXIT_FAILURE);
                            }
                            intf += arbitrary_interference(delta, alpha, T, C);
                        }
                    }
                    else
                    {
                        if (alpha <= 0)
                        {
                            std::cerr << "Interference calculation error: alpha <= 0" << std::endl;
                            std::cerr << "Alpha: " << alpha << std::endl;
                            std::cerr << "Rj: " << Rj << std::endl;
                            std::cerr << "C: " << C << std::endl;
                            std::cerr << "T: " << T << std::endl;
                            std::cerr << "D: " << D << std::endl;
                            std::cerr << "Delta: " << delta << std::endl;
                            std::cerr << "Chain ID: " << chain->getChainID() << std::endl;
                            std::cerr << "Interfering Chain ID: " << interf_chain->getChainID() << std::endl;
                            interf_chain->printChain();
                            interf_chain->printCallbacks();
                            exit(EXIT_FAILURE);
                        }
                        intf += arbitrary_interference(delta, alpha, T, C);
                    }
                }
            }
            double E_C = 0;
            for (auto &callback : chain->getCallbacks())
            {
                auto ex_time = callback->getExecutionTime();
                E_C += ex_time.tv_sec * 1e6 + ex_time.tv_usec;
            }
            if (!tg->rt_threadclass)
            {

                W = M * double(E_k) + intf - E_C;
            }
            else
            {
                std::vector<std::shared_ptr<Chain>> exclusive_chainset;
                for (auto &candidate_chain : chainset)
                {
                    if (candidate_chain != chain)
                    {
                        exclusive_chainset.push_back(candidate_chain);
                    }
                }
                double B = MLP(M, exclusive_chainset, chain->getBranchPriority(0), delta);
                W = M * double(E_k) + intf + B - E_C;
            }
            auto sbfd = std::ceil(sbf(delta, budget, THREAD_PERIOD_US));
            if (W < 0)
            {
                std::cerr << "Workload Function is negative for chain: " << chain->getChainID() << std::endl;
                delta += 1000;
            }
            else if (W < M * sbfd)
            {
                auto x = (double)chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_sec * 1e6 + chain->getCallbacks()[chain->getNumCallbacks() - 1]->getExecutionTime().tv_usec - 1;
                // auto response_time = delta + pseudo_inv_sbf(x, budget, THREAD_PERIOD_US);
                response_times[k] = delta + pseudo_inv_sbf(x, budget, THREAD_PERIOD_US);
                chain->set_Rj(convert_to_timeval(response_times[k]));
                k++;
                break;
            }
            else if (delta > (double)(20000000))
            {
                response_times[k] = 20e6;
                k++;
                break;
            }
            else
            {
                static int count = 0;
                count++;
                if (count % 1000 == 0)
                {
                    std::cerr << "Chain ID: " << chain->getChainID() << " processed 1000 iterations with bad current delta: " << delta << std::endl;
                    count = 0;
                }
                auto delta_prev = delta;
                delta = 1 + std::floor(W / M);
                if (delta <= delta_prev)
                {
                    delta = delta_prev + 200;
                }
            }
        }
    }

out_ad:
    // for each chain, we need to see if it is a subchain of a larger chain that is not in the same threadclass
    // if it is, we need to add the response time of the larger chain to the response time of the subchain
    int chain_idx = 0;
    for (auto &chain : chainset)
    {
        struct timeval root_rt = {0, 0};
        bool subchain = false;
        for (auto &cb : chain->getCallbacks())
        {
            if (cb->branch_root_cb != nullptr)
            {
                std::cerr << "Chain: " << chain->getChainID() << " is a subchain of a larger chain" << std::endl;
                subchain = true;
                root_rt = cb->branch_root_cb->root_rt;
                std::cerr << "Root RT: " << root_rt.tv_sec * 1e6 + root_rt.tv_usec << std::endl;
                break;
            }
        }
        if (subchain)
        {
            response_times[chain_idx] += root_rt.tv_sec * 1e6 + root_rt.tv_usec;
            chain->set_Rj(convert_to_timeval(response_times[chain_idx]));
        }
        chain_idx++;
    }

    std::vector<struct timeval> return_response_times;
    for (size_t i = 0; i < chainset.size(); i++)
    {
        struct timeval response_time;
        // if (response_times[i] == -1)
        // if (response_times[i] > 20e6 - 100) // don't compare with 1e7; double is inaccurate
        // {
        //     response_time.tv_sec = -1;
        //     response_time.tv_usec = -1;
        //     return_response_times.push_back(response_time);
        //     continue;
        // }
        response_time.tv_sec = static_cast<long>(response_times[i] / 1e6);
        response_time.tv_usec = static_cast<long>(response_times[i] - response_time.tv_sec * 1e6);
        return_response_times.push_back(response_time);
    }
    tg->chain_response_times = return_response_times;
    return return_response_times;
}

// Function to predict the response time based on a new budget

double MPCController::compute_chain_utilization(std::shared_ptr<Chain> chain)
{
    double utilization = 0;
    for (auto &callback : chain->getCallbacks())
    {
        auto ex_time = callback->getExecutionTime();
        utilization += ex_time.tv_sec * 1e6 + ex_time.tv_usec;
    }
    return utilization / (chain->getPeriod().tv_sec * 1e6 + chain->getPeriod().tv_usec);
}

void MPCController::create_threadclass(void)
{
    unsigned int analysis_count = 0;
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
            rt_tg->apply_budgets(THREAD_PERIOD - US_OFFSET);
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
            be_tg->apply_budgets(US_OFFSET);
            threadclasses.push_back(be_tg);
        }
    }
    std::cout << "Threadclasses created" << std::endl;
    this->reallocate_chains(&analysis_count);
    // std::cout << "Parsing and sorting chains" << std::endl;
    // std::vector<std::vector<std::shared_ptr<Chain>>> split_chainsets = exec->parse_and_sort_chains(exec->get_chains());
    // std::vector<std::shared_ptr<Chain>> rt_chains = split_chainsets[0];
    // unsigned int num_rt_chains = rt_chains.size();
    // // count the number of RT capable threadgroups
    // for (auto &tg : threadclasses)
    // {
    //     if (tg->rt_threadclass == true)
    //     {
    //         tg->add_chain(rt_chains.at(num_rt_chains - 1));
    //         num_rt_chains--;
    //         if (num_rt_chains < 0)
    //         {
    //             return;
    //         }
    //     }
    // }
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
        auto response_times = pwa_cd(existing_chains, min_tg, min_tg->total_budget / NS_IN_US);
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
            double chain_util = compute_chain_utilization(chain);
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
        double min_util = std::numeric_limits<double>::max();
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
        min_tg->set_utilization(min_util + compute_chain_utilization(chain));

        std::cout << "BE Chain " << chain->getChainID() << " added to BE threadgroup: " << min_tg->id << " with new utilization: " << min_tg->get_utilization() << std::endl;
    }
    // IF there are leftover RT threadgroups, merge the maximum utilization threadclass with the empty ones
    // merge their complementary BE threadclassesas well

merge_open_tgs:
    // for each threadclass
    int be_zero_id = -1;
    int max_be_id = -1;
    std::shared_ptr<threadclass> max_tg = nullptr;
    std::shared_ptr<threadclass> zero_tg = nullptr;
    for (auto &tg : threadclasses) // go through threadclasses
    {
        if (tg->rt_threadclass == true) // if the threadclass is RT capable
        {
            if (tg->chains.size() == 0) // if the threadclass has no chains
            {
                zero_tg = tg; // set the empty threadclass for merging later
                // find the maximum utilization threadclass
                double max_util = 0;
                for (auto &tg_it : threadclasses) // go through the RT threadclasses again to get max _tg
                {
                    if (tg_it->rt_threadclass == true && tg_it != zero_tg) // if the threadclass is RT capable and not the empty threadclass
                    {
                        double util = tg_it->get_utilization();
                        if (util > max_util)
                        {
                            max_tg = tg_it;
                            max_util = util;
                        }
                    }
                }
                // merge the maximum utilization threadclass with the empty threadclass
                if (max_tg != nullptr && max_tg != zero_tg) // if the max threadclass has been found and it is not the same as the zero_tg
                {

                    if (zero_tg->id < max_tg->id)
                    {
                        std::swap(zero_tg, max_tg);
                    }
                    std::cerr << "Merging RT threadclass " << max_tg->id << " with empty RT threadclass " << zero_tg->id << std::endl;

                    max_be_id = num_rt_tg + max_tg->id;
                    be_zero_id = num_rt_tg + zero_tg->id;

                    std::cerr << "Merging BE threadclass " << max_be_id << " with BE threadclass " << be_zero_id << std::endl;
                    // std::cout << " max_tg_idx: " << max_tg_id << " empty_tg_idx: " << tg_id << std::endl;
                    break;
                }
            }
        }
    }
    if (be_zero_id != -1 && max_be_id != -1)
    {
        max_tg->merge_threadclasss(max_tg, zero_tg);
        threadclasses.at(max_be_id)->merge_threadclasss(threadclasses.at(max_be_id), threadclasses.at(be_zero_id));
        std::cerr << "Removing BE threadclass:  " << threadclasses.at(be_zero_id)->id << std::endl;
        threadclasses.erase(std::remove(threadclasses.begin(), threadclasses.end(), threadclasses.at(be_zero_id)), threadclasses.end());
        std::cerr << "Removing RT threadclass:  " << threadclasses.at(zero_tg->id)->id << std::endl;
        threadclasses.erase(std::remove(threadclasses.begin(), threadclasses.end(), threadclasses.at(zero_tg->id)), threadclasses.end());
        for (auto &tg : threadclasses)
        {
            if (tg->id > zero_tg->id)
            {
                tg->id--;
            }
        }

        num_rt_tg--;
        goto merge_open_tgs;
    }
}

bool MPCController::reduce_rt_budget(std::shared_ptr<threadclass> tc, unsigned int *analysis_count, int min_budget)
{
    // State current_state = exec->get_state();
    //  basically we need to minimize the budget of the RT threadclass such that all the RT chains are still schedulable
    //  we can do this by performing a binary search on the budget and evaluating the schedulability of the chainset
    //  if the chainset is schedulable, we can reduce the budget further
    //  if the chainset is not schedulable, we can increase the budget
    //  we can start with the current budget of the threadclass

    // the calculations are with reference to THREAD_PERIOD whose units are nanoseconds
    // to scale the budget given to the analysis, we divide by 10000 to change our units to us from 10ms
    if (!min_budget)
    {
        min_budget = 1024 * 10;
    }
    // else
    // {
    //     min_budget += 200000; // add 200us
    // }
    int max_budget = THREAD_PERIOD, mid_budget = 0;
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
        response_times = pwa_cd(chainset, tc, mid_budget / NS_IN_US);

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
            min_budget = mid_budget + 125000; // increment by 125us
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
            be_tc->set_utilization(be_tc->get_utilization() + compute_chain_utilization(least_critical_chain));
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
        // if (tc->rt_threadclass)
        // {
        //     attr.sched_flags = 0;
        // }
        // else
        //{
        attr.sched_flags = 0 | SCHED_FLAG_RECLAIM;
        //}
        // attr.sched_flags = 0;
        attr.sched_nice = 0;
        attr.sched_priority = 0;
        attr.sched_util_min = 0;
        attr.sched_util_max = 1024;
        thread->set_sched_deadline(attr, 0);
    }
    bool fifo = false;
    int remaining_budget = THREAD_PERIOD - best_budget - static_cast<int>(0.05 * THREAD_PERIOD); // offset 5% utilization because of imprecision
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
                thread->set_priority(90);
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
        std::cerr << "BE Threadclass " << be_tc->id << " utilization: " << be_tc->get_utilization() << std::endl;
        std::cerr << "BE Threadclass " << be_tc->id << " capacity: " << (double)(be_tc->num_threads) * ((double)be_tc->total_budget) / (double)THREAD_PERIOD << std::endl;
        std::cerr << "BE Threadclass " << be_tc->id << " budget: " << be_tc->total_budget << std::endl;
        std::cerr << "BE Threadclass " << be_tc->id << " period: " << THREAD_PERIOD << std::endl;
        std::cerr << "BE Threadclass " << be_tc->id << " threads: " << be_tc->num_threads << std::endl;
    }
    else
    {
        std::cout << "BE Threadclass " << be_tc->id << " is not overloaded, expect bounded performance" << std::endl;
    }
    *analysis_count += 1;
    // perform constrained deadline analysis first at this stage, then in verify starvation freedom, do a check on whether the deadlines are constrained and choose whether to use CD or AD
    auto be_response_times = pwa_cd(be_tc->get_chains(), be_tc, be_tc->total_budget / NS_IN_US);
    std::cout << "BE Threadclass " << be_tc->id << " response times: " << std::endl;
    for (size_t i = 0; i < be_response_times.size(); i++)
    {
        if (be_response_times[i].tv_sec * 1e6 + be_response_times[i].tv_usec > 20e7 - 100 || be_response_times[i].tv_sec * 1e6 + be_response_times[i].tv_usec <= 0) // don't compare with 20e7; double is inaccurate
        {
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " on threadclass: " << be_tc->id << " is not starvation free" << std::endl;
            // std::cout << "BE Chain " << i << " is not starvation free" << std::endl;
        }
        else
        {
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " on threadclass: " << be_tc->id << " is starvation free" << std::endl;
            std::cout << "BE Chain: " << be_tc->get_chains()[i]->getChainID() << " WCRT: " << be_response_times[i].tv_sec * 1000000 + be_response_times[i].tv_usec << std::endl;
            // std::cout << "BE Chain " << i << " is starvation free" << std::endl;
            // std::cout << "Estimated Response Time for Chain " << i << ": " << be_response_times[i].tv_sec << "s " << be_response_times[i].tv_usec << "us" << std::endl;
        }
    }
    return realloc;
    //}
    //}
}

#endif // MPC_CONTROLLER_CPP
