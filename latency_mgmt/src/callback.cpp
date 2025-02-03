#ifndef CALLBACK_CPP
#define CALLBACK_CPP

#include <callback.hpp>

#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/msg/test_string.hpp"

template <typename... Args>
std::string string_format(const std::string &format, Args... args)
{
    // First, find the size needed
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
    if (size_s <= 0)
    {
        throw std::runtime_error("Error in string_format while formatting.");
    }

    // Allocate the exact buffer
    auto size = static_cast<size_t>(size_s);
    std::unique_ptr<char[]> buf(new char[size]);

    // Do the actual formatting
    std::snprintf(buf.get(), size, format.c_str(), args...);

    // Convert to std::string (excluding the null terminator)
    return std::string(buf.get(), buf.get() + size - 1);
}



#define USE_INTRA_PROCESS_COMMS false
// #define USE_INTRA_PROCESS_COMMS true // TODO: intra comm doesn't seem to work properly for BE threads (is_rt_thread == false) with PICAS_THREAD_AFFINITY
using std::placeholders::_1;

extern thread_local size_t thread_id;
extern thread_local bool is_rt_thread;

uint64_t number_cruncher(const uint64_t maximum_number)
{
    uint64_t number_of_primes = 0;
    for (uint64_t i = 3; i < maximum_number; ++i)
    {
        uint64_t rootOfI = static_cast<uint64_t>(std::sqrt(i));
        bool is_prime = true;
        for (uint64_t n = 2; n < rootOfI; ++n)
        {
            if (i % n == 0)
            {
                is_prime = false;
                break;
            }
        }

        if (is_prime)
        {
            ++number_of_primes;
        }
    }
    return number_of_primes;
}
void Callback::setPriorityScheduling(bool priority_scheduling)
{
    this->priority_scheduling = priority_scheduling;
}
bool Callback::getPriorityScheduling()
{
    return priority_scheduling;
}

Callback::Callback(CallbackType type, struct timeval period, int chainID, int placeInChain, int priority, std::string name, std::string sub_topic, std::string pub_topic, int num_cruncher_limit, std::shared_ptr<rclcpp::CallbackGroup> &chain_cb_group, bool is_nonlinear)
    : type(type), timerPeriod(period), chainID(chainID), placeInChain(placeInChain), priority(priority), name(name), num_cruncher_limit(num_cruncher_limit), is_nonlinear(is_nonlinear), chain_cb_group_(chain_cb_group),
      Node(name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS))
{
    uuid = boost::uuids::random_generator()(); // Generate a random UUID for the Callback
    setPeriod(period);
    if (sub_topic != "")
    {
        // chain_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // rclcpp::SubscriptionOptions options;
        // options.callback_group = chain_cb_group_;
        //  subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&Callback::execute_sub, this, _1), options);
        //  this->add_callback_group(chain_cb_group_);
        //  chain_cb_group_->add_subscription(subscription_);
        subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&Callback::execute_sub, this, _1));
    }

    if (pub_topic != "")
    {
        publisher_ = this->create_publisher<test_interfaces::msg::TestString>(pub_topic, 1);
    }
    else
    {
        this->end_callback = true;
    }
}

Callback::Callback(CallbackType type, struct timeval period, int chainID, int placeInChain, int priority, int branch_id, std::string name, std::string sub_topic, std::string pub_topic, int num_cruncher_limit, std::shared_ptr<rclcpp::CallbackGroup> &chain_cb_group, bool is_nonlinear)
    : type(type), timerPeriod(period), chainID(chainID), placeInChain(placeInChain), priority(priority), branch_id(branch_id), name(name), num_cruncher_limit(num_cruncher_limit), is_nonlinear(is_nonlinear), chain_cb_group_(chain_cb_group),
      Node(name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS))
{
    uuid = boost::uuids::random_generator()(); // Generate a random UUID for the Callback
    setPeriod(period);
    if (sub_topic != "")
    {
        // chain_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // rclcpp::SubscriptionOptions options;
        // options.callback_group = chain_cb_group_;
        // subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&Callback::execute_sub, this, _1), options);
        //  this->add_callback_group(chain_cb_group_);
        subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&Callback::execute_sub, this, _1));
    }
    if (pub_topic != "")
    {
        publisher_ = this->create_publisher<test_interfaces::msg::TestString>(pub_topic, 1);
    }
    else
    {
        this->end_callback = true;
    }
}

void Callback::setPeriod(struct timeval period)
{
    this->timerPeriod = period;
}

void Callback::stop_timer()
{
    if (timer_)
    {
        timer_->cancel();
        // timer_ = NULL;
    }
}

void Callback::start_timer()
{
    // Branched chains triggered by subscription can have non-zero period values for analysis purposes. Do not start timer for them.
    if (subscription_)
        return;
    RCLCPP_INFO(this->get_logger(), "Starting timer for callback %s", this->name.c_str());
    if (timerPeriod.tv_sec > 0 || timerPeriod.tv_usec > 0)
    {
        if (timer_)
            timer_->cancel();
        auto chrono_period = std::chrono::seconds(timerPeriod.tv_sec) + std::chrono::microseconds(timerPeriod.tv_usec);
        if (!timer_)
        {

            // chain_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // timer_ = this->create_wall_timer(chrono_period, std::bind(&Callback::execute_timer, this), chain_cb_group_);
            //  this->add_callback_group(chain_cb_group_);
            timer_ = this->create_wall_timer(chrono_period, std::bind(&Callback::execute_timer, this));
            exec->set_callback_priority(timer_, priority); // Needed if callback priority was set before the timer object was created
        }
        else
        {
            timer_->reset();
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Timer period is zero for callback %s", this->name.c_str());
    }
}

struct timeval Callback::getPeriod()
{
    return this->timerPeriod;
}

void Callback::setExecutionTime(struct timeval executionTime)
{
    this->addExecutionTimeToHistory(executionTime);
    // // Calculate the 95th percentile of the execution time history for the state
    // std::deque<struct timeval> executionTimes = this->getExecutionTimeHistory(state);
    // std::sort(executionTimes.begin(), executionTimes.end(), [](const struct timeval &a, const struct timeval &b) {
    //     return (a.tv_sec * 1000000 + a.tv_usec) < (b.tv_sec * 1000000 + b.tv_usec);
    // });
    // int percentileIndex = static_cast<int>(0.95 * executionTimes.size());
    // this->executionTime = executionTimes[percentileIndex];

    this->executionTime = executionTime;
}

struct timeval Callback::getExecutionTime()
{
    std::lock_guard mutex{execution_history_mutex_};
    //(void)state;
    // Take the 95th percentile of the execution time history for the state
    // std::deque<struct timeval> executionTimes = this->getExecutionTimeHistory(state);
    // if(executionTimes.empty())
    // {
    //     return {0, 0};
    // }
    // std::sort(executionTimes.begin(), executionTimes.end(), [](const struct timeval &a, const struct timeval &b) {
    //     return (a.tv_sec * 1000000 + a.tv_usec) < (b.tv_sec * 1000000 + b.tv_usec);
    // });
    // int percentileIndex = static_cast<int>(0.95 * executionTimes.size());
    // this->executionTime = executionTimes[percentileIndex];
    // std::sort(non_state_aware_ex_time_history.begin(), non_state_aware_ex_time_history.end(), [](const struct timeval &a, const struct timeval &b)
    //        { return (a.tv_sec * 1000000 + a.tv_usec) < (b.tv_sec * 1000000 + b.tv_usec); });
    // std::cout << "Execution time history size: " << non_state_aware_ex_time_history.size() << std::endl;
    // int percentileIndex = static_cast<int>(0.95 * non_state_aware_ex_time_history.size());
    // if (non_state_aware_ex_time_history.size() > 0)
    //   this->executionTime = non_state_aware_ex_time_history[percentileIndex];
    // else
    //    this->executionTime = {0, 0};
    // std::cout << "Execution time: " << this->executionTime.tv_sec << "s " << this->executionTime.tv_usec << "us" << std::endl;
    return this->executionTime;
}

void Callback::setChainID(int chainID)
{
    this->chainID = chainID;
}

int Callback::getChainID()
{
    return this->chainID;
}

void Callback::setPriority(int priority)
{
    this->priority = priority;
    if (!exec)
    {
        std::cout << "Warning: callback priority can be set only after added to the executor" << std::endl;
        return;
    }
    if (timer_)
        exec->set_callback_priority(timer_, priority); // write needs this function
    if (subscription_)
        exec->set_callback_priority(subscription_, priority);
}

int Callback::getPriority()
{
    return this->priority;
}

boost::uuids::uuid Callback::getUUID()
{
    return this->uuid;
}

void Callback::printCallback()
{
    std::cout << "Callback UUID: " << boost::uuids::to_string(this->uuid) << std::endl;
    std::cout << "Callback Period: " << this->timerPeriod.tv_sec << "s " << this->timerPeriod.tv_usec << "us" << std::endl;
    std::cout << "Callback Execution Time: " << this->executionTime.tv_sec << "s " << this->executionTime.tv_usec << "us" << std::endl;
    std::cout << "Callback Chain ID: " << this->chainID << std::endl;
    std::cout << "Callback Place in Chain: " << this->placeInChain << std::endl;
    std::cout << "Callback Branch ID: " << this->branch_id << std::endl;
    std::cout << "Callback Name: " << this->name << std::endl;
    std::cout << "Callback Num Cruncher Limit: " << this->num_cruncher_limit << std::endl;
    std::cout << "Callback Priority: " << this->priority << std::endl;
    std::cout << "Callback is nonlinear:" << this->is_nonlinear << std::endl;
    if (this->is_nonlinear)
    {
        std::cout << "Nonlinear Callback Stored Root Time: " << this->root_rt.tv_sec * 1e6 + this->root_rt.tv_usec << std::endl;
    }
    // if(this->root_cb){
    //     std::cout << "Root Callback: " << root_cb->getName() << std::endl;
    // }
    if (this->branch_root_cb)
    {
        std::cout << "Part of nonlinear chain, branch root: " << branch_root_cb->getName() << " Time: " << branch_root_cb->root_rt.tv_sec * 1e6 + branch_root_cb->root_rt.tv_usec << std::endl;
    }
    std::cout << std::endl;
}
int Callback::getNumCruncherLimit()
{
    return num_cruncher_limit;
}

void Callback::addExecutionTimeToHistory(const timeval &executionTime)
{
    std::lock_guard mutex{execution_history_mutex_};
    // if (executionTimeHistory.find(state) == executionTimeHistory.end()) {
    //     executionTimeHistory[state] = std::deque<struct timeval>();
    // }
    // executionTimeHistory[state].push_back(executionTime);
    non_state_aware_ex_time_history.push_back(executionTime);
    if (non_state_aware_ex_time_history.size() > 100)
    {
        non_state_aware_ex_time_history.pop_front();
    }
    // compare executionTime to this->executionTime
    // set this->executionTime = executionTime if executionTime > this->executionTime
    if (executionTime.tv_sec > this->executionTime.tv_sec || (executionTime.tv_sec == this->executionTime.tv_sec && executionTime.tv_usec > this->executionTime.tv_usec))
    {
        this->executionTime = executionTime;
    }

    /*std::cout << "Added execution time to history for callback: "
        << "(cb_prio " << priority << ", thread " << thread_id << ", rt " << is_rt_thread << ", seq " << getSequenceNumber() << ") "
        << name << ": " << executionTime.tv_sec << "s " << executionTime.tv_usec << "us" << std::endl;
    */
}

void Callback::setType(CallbackType type)
{
    this->type = type;
}

CallbackType Callback::getType()
{
    return type;
}

void Callback::setName(std::string name)
{
    this->name = name;
}

std::string Callback::getName()
{
    return name;
}
void Callback::setSequenceNumber(int sequence_number)
{
    this->sequence_number = sequence_number;
}

int Callback::getSequenceNumber()
{
    return sequence_number;
}
void Callback::setNumCruncherLimit(int limit) { num_cruncher_limit = limit; }

static inline void timespec_to_timeval(struct timespec *ts, struct timeval *tv)
{
    tv->tv_sec = ts->tv_sec;
    tv->tv_usec = ts->tv_nsec / 1000;
}

void Callback::execute_timer()
{
    struct timespec start, end;
    struct timeval start_tv, end_tv;
    struct timeval rec_start, rec_end;

    int chain_instance_id;
    chain_instance_id = getSequenceNumber(); // Capture the chain instance ID

    // std::string to pass into the scoped range using sprintf
    NvtxScopedRange r(string_format("Callback %s, instance %d", name.c_str(), chain_instance_id).c_str());
    // nvtx3::scoped_range range{string_format("Callback %s, instance %d", name.c_str(), chain_instance_id).c_str()};

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

    // change timer to clock get time for thread
    // gettimeofday(&start_tv, NULL);
    {
        std::lock_guard<std::mutex> lock(timer_mutex_);
        // while(!timer_);
        if (!timer_)
        {
            RCLCPP_ERROR(this->get_logger(), "Timer not set for callback %s", name.c_str());
            return;
        }
        if (timer_->is_canceled())
        {
            RCLCPP_ERROR(this->get_logger(), "Timer is canceled for callback %s", name.c_str());
            return;
        }
        ///////////////////////////////////////////////////////////

        // Increment and assign the sequence number atomically for the global sequence
        increment_sequence_number(); // Increment the sequence number
        // chain_instance_id = getSequenceNumber(); // Capture the chain instance ID
        uint64_t time_until_next_period_usec = std::chrono::duration_cast<std::chrono::microseconds>(timer_->time_until_trigger()).count();
        uint64_t period_usec = timerPeriod.tv_sec * 1e6 + timerPeriod.tv_usec;
        add_branch_timestamp(chain_instance_id, period_usec - time_until_next_period_usec); // Add the initial timestamp for the chain instance

        // std::cout << "Executing callback " << name << "(thread " << thread_id << ")" << std::endl;
        // std::cout << "Executing timer callback " << name << "(thread " << thread_id << ") Instance: " << chain_instance_id << " Elapsed from release(ms): " << (period_usec - time_until_next_period_usec) / 1000. << std::endl;
    }
    volatile uint64_t result = number_cruncher(num_cruncher_limit);
    (void)result;
    // struct timeval current_time;
    // gettimeofday(&current_time, NULL);
    test_interfaces::msg::TestString::UniquePtr message(new test_interfaces::msg::TestString());
    message->data = std::to_string(chain_instance_id);
    // message->stamp.sec = current_time.tv_sec;
    // message->stamp.usec = current_time.tv_usec;
    if (publisher_)
        publisher_->publish(std::move(message));
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Publisher not set for callback %s", name.c_str());
    }

    // auto message = test_interfaces::msg::TestString();
    // message.data = std::to_string(chain_instance_id);
    // if (publisher_)
    //     publisher_->publish(message);
    // else
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Publisher not set for callback %s", name.c_str());
    // }
    // if the recording delay has not been changed from the default value, record the time it takes to log the response time
    struct timespec start_ts, end_ts;

    if (!timerisset(&recording_delay))
    {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start_ts);
        // gettimeofday(&rec_start, NULL);
    }
    ///////////////////////////////////////////////////////////
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
    timespec_to_timeval(&start, &start_tv);
    timespec_to_timeval(&end, &end_tv);
    // gettimeofday(&end_tv, NULL);
    struct timeval execution_time;
    timersub(&end_tv, &start_tv, &execution_time);
    // std::cout << "Callback: " << callback->getName() << " executed in " << execution_time.tv_sec << "s " << execution_time.tv_usec << "us by thread: " << std::this_thread::get_id() << std::endl;
    addExecutionTimeToHistory(execution_time);
    if (!publisher_)
    { // last callback: record response time
        record_response_time(chain_instance_id);
    }
    if (!timerisset(&recording_delay))
    {
        // gettimeofday(&rec_end, NULL);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end_ts);
        timespec_to_timeval(&start_ts, &rec_start);
        timespec_to_timeval(&end_ts, &rec_end);
        struct timeval rec_time;
        timersub(&rec_end, &rec_start, &rec_time);
        recording_delay = rec_time;
    }
}
std::mutex &Callback::get_mutex()
{
    return timer_mutex_;
}
// void Callback::execute_sub(const test_interfaces::msg::TestString::SharedPtr msg)
void Callback::execute_sub(const test_interfaces::msg::TestString::UniquePtr msg)
{

    struct timespec start, end;
    struct timeval start_tv, end_tv;
    struct timeval rec_start, rec_end;
    struct timeval msg_rec, msg_sent;
    struct timeval current_time;
    int chain_instance_id = std::stoi(msg->data); // Capture the chain instance ID

    NvtxScopedRange r(string_format("Callback %s, instance %d", name.c_str(), chain_instance_id).c_str());

    // change timer to clock get time for thread
    // gettimeofday(&start_tv, NULL);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

    // gettimeofday(&msg_rec, NULL); // msg received timestamp

    // capture when the message was sent
    // msg_sent.tv_sec = msg->stamp.sec;
    // msg_sent.tv_usec = msg->stamp.usec;
    // timersub(&msg_rec, &msg_sent, &msg_rec);

    // std::cerr << "Transmission delay: " << msg_rec.tv_sec << "s " << msg_rec.tv_usec << "us" << " for callback " << name << std::endl;

    // if(timercmp(&msg_rec, &message_delay, >))
    // {
    //     // this is a total transmission delay + queueing delay + blocking time -- do not use in calculations
    //     message_delay = msg_rec;
    //     //std::cerr << "Message delay: " << message_delay.tv_sec << "s " << message_delay.tv_usec << "us" << " for callback " << name << std::endl;
    // }

    setSequenceNumber(chain_instance_id);
    // std::cout << "Executing sub callback " << name << "(thread " << thread_id << ") Instance: " << chain_instance_id << std::endl;
    add_branch_timestamp(chain_instance_id); // Add the timestamp for the specific instance

    volatile uint64_t result = number_cruncher(num_cruncher_limit);
    (void)result;

    // gettimeofday(&current_time, NULL);
    test_interfaces::msg::TestString::UniquePtr message(new test_interfaces::msg::TestString());
    message->data = std::to_string(chain_instance_id);
    // message->stamp.sec = current_time.tv_sec;
    // message->stamp.usec = current_time.tv_usec;
    if (publisher_)
        publisher_->publish(std::move(message));

    // auto message = test_interfaces::msg::TestString();
    // message.data = std::to_string(chain_instance_id);
    // if (publisher_)
    //     publisher_->publish(message);
    // if the recording delay has not been changed from the default value, record the time it takes to log the response time
    struct timespec start_ts, end_ts;

    if (!timerisset(&recording_delay))
    {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start_ts);
        // gettimeofday(&rec_start, NULL);
    }
    ///////////////////////////////////////////////////////////
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

    timespec_to_timeval(&start, &start_tv);
    timespec_to_timeval(&end, &end_tv);
    // gettimeofday(&end_tv, NULL);
    struct timeval execution_time;
    timersub(&end_tv, &start_tv, &execution_time);
    timeradd(&execution_time, &recording_delay, &execution_time);
    // timeradd(&execution_time, &message_delay, &execution_time);
    //  std::cout << "Callback: " << callback->getName() << " executed in " << execution_time.tv_sec << "s " << execution_time.tv_usec << "us by thread: " << std::this_thread::get_id() << std::endl;
    addExecutionTimeToHistory(execution_time);
    if (!publisher_)
    { // last callback: record response time
        record_response_time(chain_instance_id);
    }
    if (!timerisset(&recording_delay))
    {
        // gettimeofday(&rec_end, NULL);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end_ts);
        timespec_to_timeval(&start_ts, &rec_start);
        timespec_to_timeval(&end_ts, &rec_end);
        struct timeval rec_time;
        timersub(&rec_end, &rec_start, &rec_time);
        recording_delay = rec_time;
        std::cout << "Recording delay: " << recording_delay.tv_sec << "s " << recording_delay.tv_usec << "us" << " for callback " << name << std::endl;
    }
}

void Callback::record_response_time(int chain_instance_id)
{
    add_branch_timestamp(chain_instance_id);

    // std::cout << "Callback: " << getName() << " ending chain with timestamp: "
    //           << get_last_branch_timestamp(chain_instance_id).tv_sec << "s "
    //           << get_last_branch_timestamp(chain_instance_id).tv_usec << "us" << std::endl;
    struct timeval chain_stop_time = get_last_branch_timestamp(chain_instance_id);
    // The last callback of a chain records branch timestamp twice (before callback execution and now)
    // Other callbacks record branch timestamp only once before callback execution.
    // To handle a chain instance with a single callback, we need to get the first branch timestamp instead of the last one.
    // struct timeval chain_base_time = chain->getFirstCallback()->get_last_branch_timestamp(chain_instance_id);

    struct timeval chain_base_time;
    if (branch_root_cb != nullptr)
    {
        // check to see if the if the current callback is

        // volatile int branch_root_root_time = branch_root_cb->root_rt.tv_usec + branch_root_cb->root_rt.tv_sec*1e6;
        // std::cout << "end callback: " << this->name << " branch_root time: " << branch_root_root_time << " branch_root_cb_name: " << this->branch_root_cb->getName() << std::endl;

        chain_base_time = this->branch_root_cb->root_cb->get_first_branch_timestamp(chain_instance_id);
    }
    else
    {
        chain_base_time = chain->getFirstCallback()->get_first_branch_timestamp(chain_instance_id);
    }

    struct timeval chain_ex_time;

    timersub(&chain_stop_time, &chain_base_time, &chain_ex_time);
    chain->add_response_time_to_history(get_branch_id(), chain_ex_time);

    // struct timeval chain_ex_time = {chain_stop_time.tv_sec - chain_base_time.tv_sec,
    //                                 chain_stop_time.tv_usec - chain_base_time.tv_usec};

    // callback->addExecutionTimeToHistory(chain_ex_time);
    auto is_rt_chain = this->chain->get_branch_rt(0) ? "RT" : "BE";
    std::cout << is_rt_chain << " Chain: " << getChainID() << " prio: " << priority
              << " Instance: " << chain_instance_id
              << " Response Time: " << (chain_ex_time.tv_sec * 1000000) + chain_ex_time.tv_usec << std::endl;
    // std::cout << "Chain " << getChainID() << " (prio " << priority << ")"
    //           << " Branch " << get_branch_id()
    //           << " Instance: " << chain_instance_id
    //           << " Response Time: " << chain_ex_time.tv_sec << "s "
    //           << chain_ex_time.tv_usec << "us" << std::endl;
}

int Callback::getPlaceInChain()
{
    return placeInChain;
}

void Callback::setPlaceInChain(int place_in_chain)
{
    placeInChain = place_in_chain;
}

void Callback::set_raw_pointer(Callback *callback)
{
    this->raw_pointer = callback;
}

Callback *Callback::get_raw_pointer()
{
    return this->raw_pointer;
}

void Callback::setSharedPtr(std::shared_ptr<Callback> cb_ptr)
{
    this->shared_ptr = cb_ptr;
}

std::shared_ptr<Callback> Callback::getSharedPtr()
{
    return this->shared_ptr;
}

int Callback::get_branch_id()
{
    return branch_id;
}

void Callback::set_branch_id(int branch_id)
{
    this->branch_id = branch_id;
}

bool Callback::is_non_linear()
{
    return is_nonlinear;
}

void Callback::set_nonlinear(bool is_nonlinear)
{
    this->is_nonlinear = is_nonlinear;
}

bool Callback::is_timer_running()
{
    return timer_running;
}

void Callback::set_timer_running(bool timer_running)
{
    this->timer_running = timer_running;
}

// the following have memory leaks
void Callback::add_branch_timestamp(int chain_instance_id)
{
    struct timeval timestamp;
    gettimeofday(&timestamp, NULL);
    branch_timestamps[chain_instance_id].push_back(timestamp);
}

void Callback::add_branch_timestamp(int chain_instance_id, uint64_t elapsed_from_release_usec)
{
    struct timeval timestamp;
    gettimeofday(&timestamp, NULL);
    uint64_t timestamp_usec = timestamp.tv_sec * 1000000 + timestamp.tv_usec - elapsed_from_release_usec;
    timestamp.tv_sec = timestamp_usec / 1000000;
    timestamp.tv_usec = timestamp_usec % 1000000;
    branch_timestamps[chain_instance_id].push_back(timestamp);
}

// Get the last timestamp for a specific chain instance
struct timeval Callback::get_last_branch_timestamp(int chain_instance_id) const
{
    if (branch_timestamps.count(chain_instance_id) > 0 && !branch_timestamps.at(chain_instance_id).empty())
    {
        return branch_timestamps.at(chain_instance_id).back();
    }
    return {0, 0}; // Default value if no timestamp is found
}

// Get the first timestamp for a specific chain instance
struct timeval Callback::get_first_branch_timestamp(int chain_instance_id) const
{
    if (branch_timestamps.count(chain_instance_id) > 0 && !branch_timestamps.at(chain_instance_id).empty())
    {
        return branch_timestamps.at(chain_instance_id).front();
    }
    return {0, 0}; // Default value if no timestamp is found
}

// Get all timestamps for a specific chain instance
std::deque<struct timeval> Callback::get_branch_timestamps(int chain_instance_id) const
{
    // std::lock_guard<std::mutex> lock(timestamp_mutex);
    // timestamp_mutex.lock();
    if (branch_timestamps.count(chain_instance_id) > 0)
    {
        // timestamp_mutex.unlock();
        return branch_timestamps.at(chain_instance_id);
    }
    // timestamp_mutex.unlock();
    return {}; // Return empty deque if no timestamps exist
}

void Callback::increment_sequence_number()
{
    sequence_number++;
}

void Callback::setChain(std::shared_ptr<Chain> chain)
{
    this->chain = chain;
    this->chainID = chain->getChainID();
}

std::shared_ptr<Chain> Callback::getChain()
{
    return chain;
}

void Callback::set_callback_affinity(uint64_t affinity_mask)
{
    if (!exec)
    {
        std::cout << "Warning: callback affinity can be set only after added to the executor" << std::endl;
        return;
    }
    if (timer_)
        exec->set_callback_affinity(timer_, affinity_mask); // write needs this function
    if (subscription_)
        exec->set_callback_affinity(subscription_, affinity_mask);
}

uint64_t Callback::get_callback_affinity()
{
    if (timer_)
        return timer_->callback_affinity; // read is ok
    if (subscription_)
        return subscription_->callback_affinity;
    return 0;
}

#endif // CALLBACK_CPP
