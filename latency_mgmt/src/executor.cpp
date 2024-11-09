#ifndef EXECUTOR_CPP
#define EXECUTOR_CPP
#include <executor.hpp>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* See feature_test_macros(7) */
#endif

#include <stdio.h>
#define sched_setattr(pid, attr, flags) syscall(__NR_sched_setattr, pid, attr, flags)
#define sched_getattr(pid, attr, size, flags) syscall(__NR_sched_getattr, pid, attr, size, flags)

#define THREAD_PERIOD 1000000 // 1ms
std::vector<executor *> executor::instances;

#define LOGGER(fmt, ...) RCLCPP_INFO(rclcpp::get_logger("picas"), fmt, ##__VA_ARGS__)
//#define LOGGER(fmt, ...) ((void)0)

extern thread_local size_t thread_id;
extern thread_local bool is_rt_thread;

bool CompareCallback::operator()(const std::pair<std::shared_ptr<Callback>, int> &a, const std::pair<std::shared_ptr<Callback>, int> &b) const
{
    // Compare by type: TIMER has higher priority than SUBSCRIPTION
    if (a.first->getPriorityScheduling())
    {
        // std::cout << "Priority Scheduling Enabled" << std::endl;
        return a.first->getPriority() > b.first->getPriority();
    }
    else if (a.first->getType() != b.first->getType())
    {
        // std::cout << "Timer Vs Subscription" << std::endl;
        return a.first->getType() > b.first->getType(); // Timer < Subscription (since TIMER has higher priority)
    }
    else
    {
        // std::cout << "Sequence Number" << std::endl;
        return a.second > b.second; // if both timer or both subscription, compare by sequence number (std::prio_queue does not enforce FIFO)
    }
}

void executor::add_chain_to_thread(std::shared_ptr<Chain> chain, std::shared_ptr<executor_thread> thread)
{
    for (auto &callback : chain->getCallbacks())
    {
        thread->add_callback(callback);
    }
}

std::vector<std::vector<std::shared_ptr<Chain>>> executor::parse_and_sort_chains(std::vector<std::shared_ptr<Chain>> *in_chains)
{
    auto chains = *in_chains;
    std::vector<std::shared_ptr<Chain>> sorted_rt_chains;
    std::vector<std::shared_ptr<Chain>> sorted_be_chains;
    std::shared_ptr<Callback> previous_callback = nullptr;
    int rt_chain_id = 0, be_chain_id = 0;

    for (auto &chain : chains)
    {
        if (chain->get_num_branches() == 0)
        { // if linear
            if (chain->getBranchPriority(0) == 0)
            { // if BE
                std::shared_ptr<Chain> be_chain = std::make_shared<Chain>(be_chain_id);
                for (auto &callback : chain->getCallbacks())
                {
                    be_chain->addCallback(callback, false);
                    callback->setChain(chain);
                }
                be_chain->setPeriod(chain->getPeriod());
                be_chain->setDeadline(chain->getPeriod());
                be_chain->setBranchPriority(0, 0);
                be_chain->setLatencyTarget({0, 0}, 0, false);
                sorted_be_chains.push_back(be_chain);
                be_chain_id++;
            }
            else
            { // if linear and RT
                std::shared_ptr<Chain> rt_chain = std::make_shared<Chain>(rt_chain_id);
                for (auto &callback : chain->getCallbacks())
                {
                    rt_chain->addCallback(callback, false);
                    callback->setChain(chain);
                }
                rt_chain->setPeriod(chain->getPeriod());
                rt_chain->setDeadline(chain->getLatencyTargets()->at(0));
                rt_chain->setBranchPriority(chain->getBranchPriority(0), 0);
                rt_chain->setLatencyTarget(chain->getLatencyTargets()->at(0), 0, true);
                sorted_rt_chains.push_back(rt_chain);
                rt_chain_id++;
            }
        }
        else
        { // if nonlinear, then create a new chain for each branch and add only the branched callbacks to their own chains, change
            for (int branch_id = 0; branch_id <= chain->get_num_branches(); branch_id++)
            {
                if (chain->getBranchPriority(branch_id) == 0)
                {
                    std::shared_ptr<Chain> be_chain = std::make_shared<Chain>(be_chain_id);
                    int place_in_chain = 0;
                    for (auto &callback : chain->getCallbacks())
                    {
                        if (callback->get_branch_id() == branch_id && place_in_chain == 0)
                        {
                            // Note: Callback is derived from rclcpp::Node which cannot be copied (copy constructor not allowed). 
                            //       So, Keep the original callback instance and add its pointer to the new chain
                            //std::shared_ptr<Callback> new_callback = std::make_shared<Callback>(CallbackType::TIMER, chain->getFirstCallback()->getPeriod(), be_chain_id, place_in_chain++, 0, callback->getName(), callback->getNumCruncherLimit(), callback->getUUID());
                            callback->setPeriod(chain->getFirstCallback()->getPeriod());
                            callback->setChainID(be_chain_id);
                            callback->setPlaceInChain(place_in_chain++);
                            callback->setPriority(0);
                            callback->setExecutionTime(callback->getExecutionTime(this->get_state()), this->get_state());
                            callback->set_branch_id(0);
                            callback->set_nonlinear(false);
                            be_chain->addCallback(callback, false);
                            callback->setChain(be_chain);
                            be_chain->setPeriod(chain->getPeriod());
                            be_chain->setDeadline(chain->getPeriod());
                            be_chain->setBranchPriority(0, 0);
                            be_chain->setLatencyTarget({0, 0}, 0, false);
                        }
                        else if (callback->get_branch_id() == branch_id && place_in_chain > 0)
                        {
                            struct timeval period = {0, 0};
                            //std::shared_ptr<Callback> new_callback = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, period, be_chain_id, place_in_chain++, 0, callback->getName(), callback->getNumCruncherLimit(), callback->getUUID());
                            callback->setPeriod(period);
                            callback->setChainID(be_chain_id);
                            callback->setPlaceInChain(place_in_chain++);
                            callback->setPriority(0);
                            callback->setExecutionTime(callback->getExecutionTime(this->get_state()), this->get_state());
                            callback->set_branch_id(0);
                            callback->set_nonlinear(false);
                            be_chain->addCallback(callback, false);
                            callback->setChain(be_chain);
                            be_chain->setPeriod(chain->getPeriod());
                            be_chain->setDeadline(chain->getPeriod());
                            be_chain->setBranchPriority(0, 0);
                            be_chain->setLatencyTarget({0, 0}, 0, false);
                        }
                    }
                    be_chain_id++;
                    sorted_be_chains.push_back(be_chain);
                }
                else
                {
                    int place_in_chain = 0;

                    std::shared_ptr<Chain> rt_chain = std::make_shared<Chain>(rt_chain_id);
                    for (auto &callback : chain->getCallbacks())
                    {
                        if (callback->get_branch_id() == branch_id && place_in_chain == 0)
                        {
                            //std::shared_ptr<Callback> new_callback = std::make_shared<Callback>(CallbackType::TIMER, chain->getFirstCallback()->getPeriod(), rt_chain_id, place_in_chain++, callback->getPriority(), callback->getName(), callback->getNumCruncherLimit(), callback->getUUID());
                            callback->setPeriod(chain->getFirstCallback()->getPeriod());
                            callback->setChainID(rt_chain_id);
                            callback->setPlaceInChain(place_in_chain++);
                            callback->setExecutionTime(callback->getExecutionTime(this->get_state()), this->get_state());
                            callback->set_branch_id(0);
                            callback->set_nonlinear(false);
                            rt_chain->addCallback(callback, false);
                            callback->setChain(rt_chain);
                            rt_chain->setPeriod(chain->getPeriod());
                            rt_chain->setDeadline(chain->getLatencyTargets()->at(branch_id));
                            rt_chain->setBranchPriority(chain->getBranchPriority(branch_id), 0);
                            rt_chain->setLatencyTarget(chain->getLatencyTargets()->at(branch_id), 0, true);
                        }
                        else if (callback->get_branch_id() == branch_id && place_in_chain > 0)
                        {
                            struct timeval period = {0, 0};
                            //std::shared_ptr<Callback> new_callback = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, period, rt_chain_id, place_in_chain++, 0, callback->getName(), callback->getNumCruncherLimit(), callback->getUUID());
                            callback->setPeriod(period);
                            callback->setChainID(rt_chain_id);
                            callback->setPlaceInChain(place_in_chain++);
                            callback->setExecutionTime(callback->getExecutionTime(this->get_state()), this->get_state());
                            callback->set_branch_id(0);
                            callback->set_nonlinear(false);
                            rt_chain->addCallback(callback, false);
                            callback->setChain(rt_chain);
                            rt_chain->setPeriod(chain->getPeriod());
                            rt_chain->setDeadline(chain->getLatencyTargets()->at(branch_id));
                            rt_chain->setBranchPriority(chain->getBranchPriority(branch_id), 0);
                            rt_chain->setLatencyTarget(chain->getLatencyTargets()->at(branch_id), 0, true);
                        }
                    }
                    rt_chain_id++;
                    sorted_rt_chains.push_back(rt_chain);
                }
            }
        }
    }

    // Now sort the chains by chain priority in ascending order
    std::sort(sorted_rt_chains.begin(), sorted_rt_chains.end(), [](std::shared_ptr<Chain> a, std::shared_ptr<Chain> b)
              { return a->getBranchPriority(0) < b->getBranchPriority(0); });
    std::vector<std::vector<std::shared_ptr<Chain>>> sorted_chains;
    sorted_chains.push_back(sorted_rt_chains);
    sorted_chains.push_back(sorted_be_chains);
    return sorted_chains;
}

void executor::set_callback_priorities()
{
    int priority = 1;
    std::cout << "Current Callbacks: " << std::endl;
    // for (auto &chain : chains)
    // {
    //     chain->printCallbacks();
    // }
    std::vector<std::vector<std::shared_ptr<Chain>>> sorted_chains = parse_and_sort_chains(&chains);
    this->sorted_rt_chains = sorted_chains[0];
    this->sorted_be_chains = sorted_chains[1];
    // Update 'chains' with linearized chains
    chains.resize(0);
    for (auto &chain : sorted_rt_chains) chains.push_back(chain);
    for (auto &chain : sorted_be_chains) chains.push_back(chain);

    // for each RT chain, assign priority to each callback
    for (auto &chain : sorted_rt_chains)
    {
        // since these chains are RT or BE and always linear, we can just assign the priority in order
        for (auto &callback : chain->getCallbacks())
        {
            // We set the priority of the callback in the original chain (not the fake chain), and do the search via UUID which is copied from the original callback
            callback->getChain()->getCallback(callback->getUUID())->setPriority(priority++);
            callback->getChain()->getCallback(callback->getUUID())->setPriorityScheduling(true);
            callback->setPriority(priority - 1);
            // std::cout << "Callback: " << callback->getName() << " Priority: " << priority -1 << std::endl;
        }
        chain->setBranchPriority(priority - 1, 0);
        chain->getFirstCallback()->getChain()->setBranchPriority(priority - 1, 0);
        // std::cout << "Chain: " << chain->getChainID() << " Priority: " << priority -1 << std::endl;
        std::cout << "Printing Sorted RT Chain: " << chain->getChainID() << std::endl;
        chain->printChain();
        chain->printCallbacks();
        std::cout << std::endl;
    }
    for (auto &chain : sorted_be_chains)
    {
        for (auto &callback : chain->getCallbacks())
        {
            callback->getChain()->getCallback(callback->getUUID())->setPriority(0);
            callback->getChain()->getCallback(callback->getUUID())->setPriorityScheduling(false);
            callback->setPriority(0);
        }
        std::cout << "Printing Sorted BE Chain: " << chain->getChainID() << std::endl;
        chain->printChain();
        chain->printCallbacks();
        std::cout << std::endl;
    }
}

void executor::print_chains()
{
    for (auto &chain : chains)
    {
        chain->printChain();
    }
    std::cout << std::endl;
}
void executor::print_callbacks()
{
    for (auto &chain : chains)
    {
        chain->printCallbacks();
    }
    std::cout << std::endl;
}

void executor::print_chains_and_callbacks()
{
    for (auto &chain : this->chains)
    {
        chain->printChain();
        chain->printCallbacks();
    }
}

void executor::handle_sigint(int signum)
{
    (void)signum;
    std::cout << "SIGINT received, stopping all executors..." << std::endl;
    for (auto &instance : instances)
    {
        instance->stop();
    }
}

// Register an instance in the list
void executor::register_instance(executor *instance)
{
    instances.push_back(instance);
}

// Unregister an instance from the list
void executor::unregister_instance(executor *instance)
{
    instances.erase(std::remove(instances.begin(), instances.end(), instance), instances.end());
}

void executor::stop()
{
    running = false;
    std::cout << "Executor stopped." << std::endl;
    {
        std::lock_guard<std::mutex> lock(thread_sync_mutex);
        active_thread_mask = -1;
        thread_sync_cv.notify_all();
    }
    rclcpp::shutdown();
}

void executor::start()
{
    running = true;
    std::cout << "Executor and all timers start." << std::endl;
    // Start all timer callbacks
    for (auto &chain : this->chains)
    {
        for (auto &callback : chain->getCallbacks())
        {
            callback->start_timer(); // starts only for timer callbacks
            //set_callback_data(callback->timer_, callback->get_raw_pointer()); // not needed anymore
            //set_callback_data(callback->subscription_, callback->get_raw_pointer());
        }
    }
    // IMPORTANT: Apply callback-to-thread assignment to rclcpp
    apply_callback_to_thread_assignment();
}

void executor::pause()
{
    std::cout << "Executor paused." << std::endl;
    // Stop all timer callbacks
    for (auto &chain : this->chains)
    {
        for (auto &callback : chain->getCallbacks())
        {
            callback->stop_timer();
        }
    }    
}

bool executor::is_running()
{
    return running;
}

std::vector<std::shared_ptr<executor_thread>> executor::get_threads()
{
    return threads;
}

std::vector<std::shared_ptr<Chain>> *executor::get_chains()
{
    return &chains;
}

void executor::join()
{
    for (auto &thread : threads)
    {
        thread->get_thread()->join();
    }
    std::cout << "Threads Joined" << std::endl;
}
/*
void executor::set_partitioned(bool partitioned)
{
    this->partitioned = partitioned;
}
executor::~executor()
{
    executor::unregister_instance(this);
    std::cout << "All threads finished" << std::endl;
}
*/

void executor::print_threads()
{
    for (auto &thread : threads)
    {
        thread->getusage();
        std::cout << "Thread ID: " << thread->get_threadID() << std::endl;
        std::cout << "Thread Priority: " << thread->get_priority() << std::endl;
        std::cout << "Thread Policy: " << thread->get_policy() << std::endl;
        std::cout << "Thread CPU Set: ";
        for (int i = 0; i < CPU_SETSIZE; i++)
        {
            if (CPU_ISSET(i, thread->get_cpuSet()))
            {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;
        std::cout << "Thread Budget: " << thread->budget << std::endl;
        std::cout << "Thread Usage: " << std::endl;
        std::cout << "User CPU Time: " << thread->usage.ru_utime.tv_sec << "s " << thread->usage.ru_utime.tv_usec << "us" << std::endl;
        std::cout << "System CPU Time: " << thread->usage.ru_stime.tv_sec << "s " << thread->usage.ru_stime.tv_usec << "us" << std::endl;
        std::cout << "Maximum Resident Set Size: " << thread->usage.ru_maxrss << std::endl;
        std::cout << "Integral Shared Memory Size: " << thread->usage.ru_ixrss << std::endl;
        std::cout << "Integral Unshared Data Size: " << thread->usage.ru_idrss << std::endl;
        std::cout << "Integral Stack Size: " << thread->usage.ru_isrss << std::endl;
        std::cout << "Page Reclaims (soft page faults): " << thread->usage.ru_minflt << std::endl;
        std::cout << "Page Faults (hard page faults): " << thread->usage.ru_majflt << std::endl;
        std::cout << "Swaps: " << thread->usage.ru_nswap << std::endl;
        std::cout << "Block Input Operations: " << thread->usage.ru_inblock << std::endl;
        std::cout << "Block Output Operations: " << thread->usage.ru_oublock << std::endl;
        std::cout << "IPC Messages Sent: " << thread->usage.ru_msgsnd << std::endl
                  << std::endl;
    }
}

void executor::make_threads(int num_threads) // equivalent to MultiThreadedExecutor::spin()
{
    number_of_threads_ = num_threads;
    spinning.exchange(true);
    update_active_threads(0); // Force threads to wait until start() is called

    if (num_threads > (int)std::thread::hardware_concurrency())
    {
        std::cerr << "Error: Number of threads exceeds hardware concurrency. Setting to hardware concurrency." << std::endl;
        num_threads = std::thread::hardware_concurrency() - 1;
    }
    int num_rt_threads = num_threads;
    int num_be_threads = num_threads;
    for (int i = 0; i < num_rt_threads + num_be_threads; i++)
    {
        std::cout << "Creating thread " << i << std::endl;
        std::lock_guard wait_lock{wait_mutex_};
        //auto thread = std::make_shared<executor_thread>(&global_waitset_mutex, this);
        auto thread = std::make_shared<executor_thread>(&wait_mutex_, this, i); // i: logical_thread_id
        threads.push_back(thread);
    }
    // For each thread, create a thread and run executor_thread::spin
    for (auto &thread : threads)
    {
        // thread->assign_thread_ptr(std::make_shared<std::thread>(&executor_thread::spin, thread));
        //auto raw_thread = std::make_shared<std::thread>(&executor_thread::spin, thread);
        auto raw_thread = std::make_shared<std::thread>(&executor::run, this, thread);
        raw_threads.push_back(raw_thread);
        thread->assign_thread_ptr(raw_thread);
        while (!thread->get_threadID())
            ;
        std::cout << "Thread ID: " << thread->get_threadID() << "(" << thread->logical_thread_id << ")" << std::endl;
        if (num_rt_threads > 0)
        {
            std::cout << "Creating RT Thread No. " << num_rt_threads << std::endl;
            thread->set_policy(SCHED_DEADLINE);
            // thread->set_priority(0);
            thread->set_budget(THREAD_PERIOD - 1024);
            cpu_set_t cpuSet;
            CPU_ZERO(&cpuSet);
            CPU_SET(num_rt_threads, &cpuSet);
            num_rt_threads--;

            thread->set_affinity(cpuSet, true); 
            struct sched_attr attr;
            attr.size = sizeof(attr);
            attr.sched_policy = SCHED_DEADLINE;
            attr.sched_runtime = THREAD_PERIOD - 1024;
            attr.sched_period = THREAD_PERIOD;   // 200ms period
            attr.sched_deadline = THREAD_PERIOD; // 200ms deadline
            attr.sched_flags = 0;
            attr.sched_nice = 0;
            attr.sched_priority = 0;
            attr.sched_util_min = 0;
            attr.sched_util_max = 1024;
            thread->set_sched_deadline(attr, 0);
            thread->set_rt(true);
        }
        else if (num_be_threads > 0)
        {
            thread->set_policy(SCHED_DEADLINE);
            // thread->set_priority(0);
            thread->set_budget(1024);
            cpu_set_t cpuSet;
            CPU_ZERO(&cpuSet);
            CPU_SET(num_be_threads, &cpuSet);
            num_be_threads--;
            thread->set_affinity(cpuSet, false);
            struct sched_attr attr;
            attr.size = sizeof(attr);
            attr.sched_policy = SCHED_DEADLINE;
            attr.sched_runtime = 1024;
            attr.sched_period = THREAD_PERIOD;   // 10ms period
            attr.sched_deadline = THREAD_PERIOD; // 10ms deadline
            attr.sched_flags = 0;
            attr.sched_nice = 0;
            attr.sched_priority = 0;
            attr.sched_util_min = 0;
            attr.sched_util_max = 1024;
            thread->set_sched_deadline(attr, 0);
            thread->set_rt(false);
        }

        // std::cout << "Thread ID: " << thread->get_threadID() << std::endl;
        //  thread->assign_thread_ptr(std::make_shared<std::thread>(std::bind(&executor_thread::spin, thread)));
    }

    std::cout << "Threads created, waiting for them to finish" << std::endl;
}

void executor::run(std::shared_ptr<executor_thread> t) // equivalent to MultiThreadedExecutor::run()
{
    thread_id = (int)t->logical_thread_id;
    t->threadID = syscall(SYS_gettid);
    CPU_ZERO(&t->cpuSet);
    sched_getaffinity(t->threadID, sizeof(cpu_set_t), &t->cpuSet);
    t->policy = sched_getscheduler(t->threadID);

    // Wait for go sign. Needed regardless of PICAS_THREAD_AFFINITY
    while (!(active_thread_mask & (1 << thread_id))) {
        std::unique_lock<std::mutex> lock(thread_sync_mutex);
        if (!(active_thread_mask & (1 << thread_id)))
            thread_sync_cv.wait(lock);
    }

    // IMPORTANT: is_rt_thread is used by rclcpp:Executor
    // - true: priority-based scheduling & refreshing waitset every time
    // - false: standard ROS scheduling
    // NOTE: t->rt may be set after the thread begins run(). So let's check it within the loop.
    is_rt_thread = t->rt; 

    LOGGER("[run] thread %lu active (is_rt: %d, mask %lx)", thread_id, is_rt_thread, active_thread_mask);
    while (rclcpp::ok(this->context_) && spinning.load()) {
        rclcpp::AnyExecutable any_exec;

#ifdef PICAS_THREAD_AFFINITY
        if (!(active_thread_mask & (1 << thread_id))) {
            std::unique_lock<std::mutex> lock(thread_sync_mutex);
            if (!(active_thread_mask & (1 << thread_id))) {
                //LOGGER("[run] thread %lu - inactive cv wait", thread_id);
                thread_sync_cv.wait(lock);
                //thread_sync_cv.wait_for(lock, std::chrono::milliseconds(10));
                continue;
            }
        }
#endif
        {
            //LOGGER("[run] thread %lu", thread_id);
            std::lock_guard wait_lock{wait_mutex_};
            //LOGGER("[run] thread %lu - lock acquired", thread_id);
            if (!rclcpp::ok(this->context_) || !spinning.load()) {
                return;
            }
            if (!get_next_executable(any_exec, next_exec_timeout_)) {
                continue;
            }
        }
        //LOGGER("[run] thread %lu - get next executable", thread_id);
        if (yield_before_execute_) {
            std::this_thread::yield();
        }

        execute_any_executable(any_exec);
        //execute_and_time(any_exec); 

        // Clear the callback_group to prevent the AnyExecutable destructor from
        // resetting the callback group `can_be_taken_from`
        any_exec.callback_group.reset();
    }
    LOGGER("[run] thread %lu EXIT (rclcpp::ok %d, spinning %d)", thread_id, rclcpp::ok(this->context_), spinning.load());
}


/*
executor::executor(int num_threads, bool partitioned)
{
    executor::register_instance(this);
    signal(SIGINT, executor::handle_sigint);

    this->set_partitioned(partitioned);
    this->make_threads(num_threads);
    return;

    std::cout << "Threads created, waiting for them to finish" << std::endl;
}

void executor::schedule_timer_callback(std::shared_ptr<Callback> callback)
{
    // Increment and assign the sequence number atomically for the global sequence
    callback->increment_sequence_number();                 // Increment the sequence number
    int chain_instance_id = callback->getSequenceNumber(); // Capture the chain instance ID

    // Create a callback_entry and fill in the details
    callback_entry entry;
    entry.raw_ptr = callback->get_raw_pointer();             // Use the raw pointer from the callback
    entry.global_sequence_number = ++global_sequence_number; // Increment and assign sequence number
    entry.chain_instance_id = chain_instance_id;             // Assign the chain instance ID

    callback->add_branch_timestamp(chain_instance_id); // Add the initial timestamp for the chain instance

    // Push the callback_entry directly into the lock-free queue
    global_queue_mutex.lock();
    global_waitset_queue.push_back(entry); // Push the callback_entry into the queue
    global_queue_mutex.unlock();
    // Calculate the next execution time based on the callback's timer period
    auto period = std::chrono::milliseconds(callback->getPeriod().tv_sec * 1000 + callback->getPeriod().tv_usec / 1000);

    // Check if the timer thread already exists
    if (!callback->is_timer_running())
    {
        // Use a background thread to periodically insert callbacks
        std::thread([this, callback, period, chain_instance_id]()
                    {
                        callback->set_timer_running(true); // Indicate the timer is running

                        while (this->running)
                        {
                            std::this_thread::sleep_for(period);

                            // Increment the sequence number for the periodic callback
                            callback->increment_sequence_number(); // Increment sequence number
                            int periodic_instance_id = callback->getSequenceNumber();

                            // Add a new timestamp for this chain instance
                            callback->add_branch_timestamp(periodic_instance_id);

                            // Create a callback_entry for the periodic callback
                            callback_entry periodic_entry;
                            periodic_entry.raw_ptr = callback->get_raw_pointer();
                            periodic_entry.global_sequence_number = ++global_sequence_number; // Use atomic increment
                            periodic_entry.chain_instance_id = periodic_instance_id;          // Track instance ID

                            global_queue_mutex.lock();
                            // Push the periodic callback into the lock-free queue
                            global_waitset_queue.push_back(periodic_entry);
                            global_queue_mutex.unlock();
                            // if (callback->getChainID() == 0)
                            // {

                            //     std::cout << "Periodic Callback: " << callback->getName()
                            //               << " for Chain Instance: " << periodic_instance_id
                            //               << " queued with Sequence Number: " << periodic_entry.global_sequence_number
                            //               << " with timestamp: " << callback->get_last_branch_timestamp(periodic_instance_id).tv_sec << "s "
                            //               << callback->get_last_branch_timestamp(periodic_instance_id).tv_usec << "us"
                            //               << std::endl;
                            // }
                        }

                        callback->set_timer_running(false); // Timer is no longer running
                    })
            .detach(); // Detach the thread to run independently
    }
}
*/

executor::executor(int num_threads)
 : MultiThreadedExecutor(rclcpp::ExecutorOptions(), num_threads)
{
    executor::register_instance(this);
    signal(SIGINT, executor::handle_sigint);
    // Turns on PICAS priority-based callback scheduling in rclcpp
    // Once turned on, any thread with is_rt_thread == true uses priority scheduling.
    // Threads with is_rt_thread == false still follows standard ROS scheduling
    this->enable_callback_priority(); 
    make_threads(num_threads);
}

executor::~executor() {}

/*
executor::executor()
{
    executor::register_instance(this);
    signal(SIGINT, executor::handle_sigint);
    // Create a thread for each core
    make_threads(std::thread::hardware_concurrency());
}

void executor::add_thread(std::shared_ptr<executor_thread> thread)
{
    thread->assign_thread_ptr(std::make_shared<std::thread>(&executor_thread::spin, thread));
    threads.push_back(thread);
}

void executor::remove_thread(int threadID)
{
    threads.erase(threads.begin() + threadID);
}
*/

void executor::set_thread_priority(int threadID, int priority)
{
    threads.at(threadID)->set_priority(priority);
}

void executor::set_thread_policy(int threadID, int policy)
{
    threads.at(threadID)->set_policy(policy);
}

void executor::set_thread_affinity(int threadID, cpu_set_t cpuSet, bool rt)
{
    threads.at(threadID)->set_affinity(cpuSet, rt);
}

void executor::add_callback_to_thread(int threadID, std::shared_ptr<Callback> callback)
{
    threads.at(threadID)->add_callback(callback);
}

void executor::remove_callback_from_thread(int threadID, std::shared_ptr<Callback> callback)
{
    threads.at(threadID)->remove_callback(callback->getUUID());
}

void executor::get_thread_usage(int threadID)
{
    threads.at(threadID)->getusage();
}

void executor::set_thread_budget(int threadID, int budget)
{
    threads.at(threadID)->budget = budget;
}

void executor::add_all_callbacks_to_all_threads()
{
    for (auto &thread : threads)
    {
        if (!thread->rt) continue; // FIXME: I guess BE threads don't have enough budget to run callbacks for profiling
        for (auto &chain : chains)
        {
            for (auto &callback : chain->getCallbacks())
            {
                thread->add_callback(callback);
            }
        }
    }
}

void executor::remove_all_callbacks_from_all_threads()
{
    for (auto &thread : threads)
    {
        thread->callbacks.clear();
    }
}

void executor::remove_all_callbacks_from_thread(int threadID)
{
    threads.at(threadID)->callbacks.clear();
}

// Apply callback-to-thread assignment to rclcpp executor backend
void executor::apply_callback_to_thread_assignment()
{
    std::lock_guard wait_lock{wait_mutex_};
    for (auto &chain : chains) {
        for (auto &callback : chain->getCallbacks()) {
            callback->set_callback_affinity(0);
        }
    }
    uint64_t active_thread_mask = 0;
    LOGGER("[cb to thread] Update rclcpp callback-to-thread assignment");
    for (auto &thread : threads)
    {
        auto callbacks = thread->get_callbacks();
        LOGGER("[cb to thread] thread %d: callbacks.size = %lu, rt = %d", thread->logical_thread_id, callbacks.size(), thread->rt);
        for (auto it = callbacks.begin(); it != callbacks.end(); it++) 
        {
            uint64_t mask = it->second->get_callback_affinity();
            mask |= 1 << thread->logical_thread_id;
            it->second->set_callback_affinity(mask);
            LOGGER("[cb to thread] thread %d: chain %d callback %d (mask %lx)", thread->logical_thread_id, it->second->getChainID(), it->second->getPlaceInChain(), mask);
        }
        if (callbacks.size() > 0) active_thread_mask |= 1 << thread->logical_thread_id;
    }
    // IMPORTANT: notify which threads are active
#ifdef PICAS_THREAD_AFFINITY
    update_active_threads(active_thread_mask);    
#else
    update_active_threads(-1);    
#endif
}

/*
void executor::remove_callback_from_global_waitset_and_queue(std::shared_ptr<Callback> callback)
{
    // Step 1: Remove the callback from the global waitset (priority queue)
    std::deque<std::pair<std::shared_ptr<Callback>, int>> temp; // Temporarily hold non-matching callbacks

    global_waitset_mutex.lock();

    // Remove items from the global waitset (priority queue)
    while (!global_waitset.empty())
    {
        auto current = global_waitset.top(); // Get the top callback_entry
        global_waitset.pop();                // Pop the top entry

        // Only keep callbacks that don't match the UUID of the callback to remove
        if (current.first->getUUID() != callback->getUUID())
        {
            temp.push_back(current); // Store non-matching callbacks
        }
    }

    // Reinsert the remaining callbacks back into the global waitset
    for (auto &item : temp)
    {
        global_waitset.push(item); // Push back the non-matching callbacks
    }

    global_waitset_mutex.unlock();

    // Step 2: Remove the callback from the Boost lock-free queue (global_waitset_queue)
    callback_entry current_item;
    std::deque<callback_entry> remaining_items;
    global_queue_mutex.lock();
    // Pop all items from the lock-free queue and filter out the callback to remove
    while (!global_waitset_queue.empty())
    {
        current_item = global_waitset_queue.front(); // Get the front item
        global_waitset_queue.pop_front();            // Pop the front item
        // Only keep items that don't match the callback's UUID
        if (current_item.raw_ptr->getUUID() != callback->getUUID())
        {
            remaining_items.push_back(current_item); // Store non-matching items
        }
    }

    // Reinsert the remaining items back into the lock-free queue
    for (auto &item : remaining_items)
    {
        global_waitset_queue.push_back(item); // Now pushing the correct callback_entry
    }
    global_queue_mutex.unlock();
}

void executor::remove_chain(std::shared_ptr<Chain> chain)
{
    registration_mutex.lock();
    callback_count -= chain->getNumCallbacks();
    chains.erase(std::remove(chains.begin(), chains.end(), chain), chains.end());
    for (auto &chain : chains)
    {
        chain->setChainID(chain->getChainID() - 1);
    }
    registration_mutex.unlock();
}
*/

static inline void timespec_to_timeval(struct timespec *ts, struct timeval *tv)
{
    tv->tv_sec = ts->tv_sec;
    tv->tv_usec = ts->tv_nsec / 1000;
}

// Deprecated: Execution time is measured inside callback function because there is no way to know chain instance ID here...
/*void executor::execute_and_time(rclcpp::AnyExecutable &any_exec)
{
    struct timespec start, end;
    struct timeval start_tv, end_tv;
    // change timer to clock get time for thread
    // gettimeofday(&start_tv, NULL);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

    execute_any_executable(any_exec);

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
    timespec_to_timeval(&start, &start_tv);
    timespec_to_timeval(&end, &end_tv);
    // gettimeofday(&end_tv, NULL);
    struct timeval execution_time;
    timersub(&end_tv, &start_tv, &execution_time);
    // std::cout << "Callback: " << callback->getName() << " executed in " << execution_time.tv_sec << "s " << execution_time.tv_usec << "us by thread: " << std::this_thread::get_id() << std::endl;
    void *ptr = NULL;
    if (any_exec.timer) { 
        ptr = any_exec.timer->callback_data; 
        //if (ptr) LOGGER("[execute_and_time] timer prio %d, affinity %lx", any_exec.timer->callback_priority, any_exec.timer->callback_affinity); 
    }
    else if (any_exec.subscription) { 
        ptr = any_exec.subscription->callback_data; 
        //if (ptr) LOGGER("[execute_and_time] subscription prio %d, affinity %lx", any_exec.subscription->callback_priority, any_exec.subscription->callback_affinity); 
    }
    else if (any_exec.service) ptr = any_exec.service->callback_data;
    else if (any_exec.client) ptr = any_exec.client->callback_data;
    else if (any_exec.waitable) { 
        ptr = any_exec.waitable->callback_data; 
        //if (ptr) LOGGER("[execute_and_time] waitable prio %d, affinity %lx", any_exec.waitable->callback_priority, any_exec.waitable->callback_affinity); 
    }
    if (ptr) {
        Callback* callback = (Callback*)ptr;
        callback->addExecutionTimeToHistory(get_state(), execution_time);
        if (!callback->publisher_){ // last callback: record response time
        }
    }
}*/

/*
void executor::print_waitset()
{
    std::cout << "Global Waitset: " << std::endl;
    global_waitset_mutex.lock();
    std::priority_queue<std::pair<std::shared_ptr<Callback>, int>, std::vector<std::pair<std::shared_ptr<Callback>, int>>, CompareCallback> temp = global_waitset;
    while (!temp.empty())
    {
        auto current = temp.top();
        temp.pop();
        std::cout << "Callback: " << current.first->getName() << " Sequence Number: " << current.second << std::endl;
    }
    global_waitset_mutex.unlock();
}
void executor::print_waitset_queue()
{
    std::cout << "Global Waitset Queue: " << std::endl;
    global_queue_mutex.lock();
    for (auto &item : global_waitset_queue)
    {
        std::cout << "Callback: " << item.raw_ptr->getName() << " Chain Instance ID: " << item.chain_instance_id << std::endl;
    }
    global_queue_mutex.unlock();
}

void executor::add_next_callback_to_global_waitset_queue(std::shared_ptr<Callback> callback, int chain_instance_id)
{
    // print_waitset();
    // print_waitset_queue();

    partitioned_mutex.lock();

    auto current_chain = chains.at(callback->getChainID());
    auto current_place_in_chain = callback->getPlaceInChain();
    // int chain_instance_id = callback->getSequenceNumber();

    if (callback->is_non_linear())
    {
        for (auto &next_callback : current_chain->getCallbacks())
        {
            if (next_callback->getPlaceInChain() == current_place_in_chain + 1 && next_callback != nullptr)
            {
                // next_callback->setSequenceNumber(chain_instance_id);
                next_callback->add_branch_timestamp(chain_instance_id); // Add the timestamp for the specific instance

                callback_entry entry;
                entry.raw_ptr = next_callback->get_raw_pointer();
                entry.global_sequence_number = ++global_sequence_number;
                entry.chain_instance_id = chain_instance_id;

                global_waitset_queue.push_back(entry);

                // std::cout << "Nonlinear Callback: " << next_callback->getName()
                //           << " queued by callback: " << callback->getName() << std::endl;
            }
        }
    }
    else
    {
        bool callback_found = false;
        for (auto next_callback : current_chain->getCallbacks())
        {
            if (next_callback->getPlaceInChain() == current_place_in_chain + 1 &&
                next_callback->get_branch_id() == callback->get_branch_id() && next_callback != nullptr)
            {

                next_callback->setSequenceNumber(chain_instance_id);
                next_callback->add_branch_timestamp(chain_instance_id); // Add the timestamp for the specific instance

                callback_found = true;

                callback_entry entry;
                entry.raw_ptr = next_callback->get_raw_pointer();
                entry.global_sequence_number = ++global_sequence_number;
                entry.chain_instance_id = chain_instance_id;

                global_waitset_queue.push_back(entry);

                // std::cout << "Callback: " << next_callback->getName()
                //           << " queued by callback: " << callback->getName() << std::endl;
            }
        }

        if (!callback_found)
        {
            callback->add_branch_timestamp(chain_instance_id);

            // std::cout << "Callback: " << callback->getName() << " ending chain with timestamp: "
            //           << callback->get_last_branch_timestamp(chain_instance_id).tv_sec << "s "
            //           << callback->get_last_branch_timestamp(chain_instance_id).tv_usec << "us" << std::endl;
            struct timeval chain_stop_time = callback->get_last_branch_timestamp(chain_instance_id);
            struct timeval chain_base_time = chains[callback->getChainID()]->getFirstCallback()->get_last_branch_timestamp(chain_instance_id);
            struct timeval chain_ex_time;
            timersub(&chain_stop_time, &chain_base_time, &chain_ex_time);
            chains[callback->getChainID()]->add_response_time_to_history(get_state(), callback->get_branch_id(), chain_ex_time);
            // struct timeval chain_ex_time = {chain_stop_time.tv_sec - chain_base_time.tv_sec,
            //                                 chain_stop_time.tv_usec - chain_base_time.tv_usec};

            // callback->addExecutionTimeToHistory(chain_ex_time);

            std::cout << "Chain " << callback->getChainID()
                      << " Branch " << callback->get_branch_id()
                      << " Instance: " << chain_instance_id
                      << " Execution Time: " << chain_ex_time.tv_sec << "s "
                      << chain_ex_time.tv_usec << "us" << std::endl;
        }
    }

    partitioned_mutex.unlock();
}
*/
std::shared_ptr<executor_thread> executor::get_thread(int threadID)
{
    return threads.at(threadID);
}
/*
void executor::update_waitset_partitioned()
{
    int i = 0;
    std::deque<callback_entry> rejected; // To hold callbacks that are rejected
    std::set<callback_entry> callbacks;  // To track unique callbacks

    // Lock the global_waitset_mutex before interacting with global_partitioned_waitset
    this->global_waitset_mutex.lock();

    // If the partitioned waitset is already populated, exit early
    if (!priority_scheduling && !this->global_partitioned_waitset.empty())
    {
        this->global_waitset_mutex.unlock();
        return;
    }
    callback_entry callback;  // Temp variable to hold popped callbacks
    partitioned_mutex.lock(); // Lock the partitioned mutex
    // Pop elements from the lock-free queue and process them
    while (!global_waitset_queue.empty() && global_partitioned_waitset.size() < callback_count)
    {
        // If the callback is already in the partitioned waitset, reject it
        if (global_partitioned_waitset.find(std::make_pair(callback.raw_ptr->getSharedPtr(), callback.chain_instance_id)) != global_partitioned_waitset.end())
        {
            rejected.push_back(callback); // Store rejected callbacks
            continue;
        }
        i++;
        // Otherwise, insert it into the partitioned waitset
        global_partitioned_waitset.insert(std::make_pair(callback.raw_ptr->getSharedPtr(), callback.chain_instance_id));
    }

    // Unlock the mutex after finishing the queue processing
    this->global_waitset_mutex.unlock();

    // // pop all remaining items in queue, then push them back in the order they were popped
    // while (!global_waitset_queue.empty() && !rejected.empty())
    // {
    //     callback = global_waitset_queue.front();
    //     global_waitset_queue.pop_front();
    //     rejected.push_back(callback);
    // }

    // Push the rejected callbacks back into the lock-free queue
    while (!rejected.empty())
    {
        global_waitset_queue.push_front(rejected.back());
        rejected.pop_back();
    }
    partitioned_mutex.unlock(); // Unlock the partitioned mutex
}
*/
State executor::get_state()
{
    State state;
    state.chains = chains;
    state.num_threads = threads.size();
    for (auto &thread : threads)
    {
        state.thread_prio.push_back(thread->get_priority());
        state.thread_policy.push_back(thread->get_policy());
        state.sched_deadline_budget.push_back(thread->budget);
        state.thread_cpu_set.push_back(*thread->get_cpuSet());
    }
    state.prioritized = priority_scheduling;
    state.partitioned = partitioned;
    // makes a list of all the branch targets without branch identifiers ******
    for (auto &chain : chains)
    {
        auto latency_targets = *chain->getLatencyTargets();
        for (auto &target : latency_targets)
        {
            state.chain_latency_targets.push_back(target);
        }
        // state.chain_latency_targets.push_back(*chain->getLatencyTargets());
    }
    return state;
}
/*
void executor::update_waitset()
{
    int i = 0;
    std::deque<callback_entry> rejected; // Rejected callbacks
    std::set<callback_entry> callbacks;  // To track unique callbacks

    // Lock the mutex before working with the global_waitset (priority queue)
    global_waitset_mutex.lock();
    // Repurpose the partitioned lock in order to preserve the ordering in the lock-free queue

    // Check if global_waitset is already populated
    if (!global_waitset.empty() && !priority_scheduling) // if the global waitset is not empty and priority scheduling is not enabled
    {
        global_waitset_mutex.unlock();
        partitioned_mutex.unlock();
        return; // return because we don't need to update the waitset
    }

    callback_entry callback; // Temp variable to hold popped callbacks
    global_queue_mutex.lock();
    // Pop elements from the lock-free queue and process them
    while (!global_waitset_queue.empty() && global_waitset.size() < callback_count)
    {
        callback = global_waitset_queue.front();
        global_waitset_queue.pop_front();
        if (callback.raw_ptr == nullptr && callback.global_sequence_number == 0)
        {
            continue;
        }
        // Check if the callback is already in the set
        if (callbacks.find(callback) != callbacks.end())
        {
            // If it's already in the set, reject it
            rejected.push_back(callback);
            continue;
        }

        // Add the callback to the set and push it to the priority queue (global_waitset)
        callbacks.insert(callback);
        // global_waitset.push(std::make_pair(callback.raw_ptr->getSharedPtr(), callback.global_sequence_number));
        global_waitset.push(std::make_pair(callback.raw_ptr->getSharedPtr(), callback.chain_instance_id));
    }

    // Unlock the mutex after finishing the queue processing
    global_waitset_mutex.unlock();
    while (!global_waitset_queue.empty() && !rejected.empty())
    {
        callback = global_waitset_queue.front();
        global_waitset_queue.pop_front();
        rejected.push_back(callback);
    }
    // Push the rejected callbacks back into the lock-free queue
    while (!rejected.empty())
    {
        global_waitset_queue.push_back(rejected.front());
        rejected.pop_front();
    }
    global_queue_mutex.unlock();
}
*/

void executor::add_chain(std::shared_ptr<Chain> chain)
{
    std::lock_guard wait_lock{wait_mutex_};
    if (callback_priority_enabled)
    {
        for (auto &callback : chain->getCallbacks())
        {
            callback->setPriorityScheduling(true);
            callback->exec = this;
            this->add_node(callback);
        }
    }
    chain->setChainID(chains.size());
    chains.push_back(chain);
    callback_count += chain->getNumCallbacks();
}

/*
void executor::add_callback_to_chain(int chainID, std::shared_ptr<Callback> callback)
{
    std::lock_guard wait_lock{wait_mutex_};
    //registration_mutex.lock();
    chains.at(chainID)->addCallback(callback);
    callback_count++;
    //registration_mutex.unlock();
}

void executor::remove_callback_from_chain(int chainID, boost::uuids::uuid callbackUUID)
{
    std::lock_guard wait_lock{wait_mutex_};
    //registration_mutex.lock();
    chains.at(chainID)->removeCallback(callbackUUID);
    callback_count--;
    //registration_mutex.unlock();
}

void executor::enable_priority_scheduling()
{
    set_callback_priorities();
    priority_scheduling = true;
    for (auto &chain : chains)
    {
        for (auto &callback : chain->getCallbacks())
        {
            callback->setPriorityScheduling(true);
        }
    }
}

void executor::disable_priority_scheduling()
{
    priority_scheduling = false;
    for (auto &chain : chains)
    {
        for (auto &callback : chain->getCallbacks())
        {
            callback->setPriorityScheduling(false);
        }
    }
}

bool executor::is_priority_scheduling_enabled()
{
    return priority_scheduling;
}

bool executor::get_partitioned()
{
    return partitioned;
}
*/

#endif
