#ifndef CALLBACK_CPP
#define CALLBACK_CPP

#include <callback.hpp>

#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/msg/test_string.hpp"

#define USE_INTRA_PROCESS_COMMS true
using std::placeholders::_1;

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

Callback::Callback(CallbackType type, struct timeval period, int chainID, int placeInChain, int priority, std::string name, std::string sub_topic, std::string pub_topic, int num_cruncher_limit, bool is_nonlinear)
    : type(type), timerPeriod(period), chainID(chainID), placeInChain(placeInChain), priority(priority), name(name), num_cruncher_limit(num_cruncher_limit), is_nonlinear(is_nonlinear),
      Node(name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS))
{
    uuid = boost::uuids::random_generator()(); // Generate a random UUID for the Callback
    setPeriod(period);
    if (sub_topic != "") {
        subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&Callback::execute_sub, this, _1));
        subscription_->data_ = (void*)this;
    }
    if (pub_topic != "") {
        publisher_ = this->create_publisher<test_interfaces::msg::TestString>(pub_topic, 1);
    }
}

Callback::Callback(CallbackType type, struct timeval period, int chainID, int placeInChain, int priority, int branch_id, std::string name, std::string sub_topic, std::string pub_topic, int num_cruncher_limit, bool is_nonlinear)
    : type(type), timerPeriod(period), chainID(chainID), placeInChain(placeInChain), priority(priority), branch_id(branch_id), name(name), num_cruncher_limit(num_cruncher_limit), is_nonlinear(is_nonlinear),
      Node(name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS))
{
    uuid = boost::uuids::random_generator()(); // Generate a random UUID for the Callback
    setPeriod(period);
    if (sub_topic != "") {
        subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&Callback::execute_sub, this, _1));
        subscription_->data_ = (void*)this;
    }
    if (pub_topic != "") {
        publisher_ = this->create_publisher<test_interfaces::msg::TestString>(pub_topic, 1);
    }
}

void Callback::setPeriod(struct timeval period) 
{
    this->timerPeriod = period;
}

void Callback::stop_timer() 
{
    if (timer_) {
        timer_->cancel();
        timer_ = NULL;
    }
}

void Callback::start_timer() 
{
    if (timerPeriod.tv_sec > 0 || timerPeriod.tv_usec > 0) {
        if (timer_) timer_->cancel();
        auto chrono_period = std::chrono::seconds(timerPeriod.tv_sec) + std::chrono::microseconds(timerPeriod.tv_usec);
        timer_ = this->create_wall_timer(chrono_period, std::bind(&Callback::execute_timer, this));
        timer_->data_ = (void*)this;
    }
}

struct timeval Callback::getPeriod()
{
    return this->timerPeriod;
}

void Callback::setExecutionTime(struct timeval executionTime, const State &state)
{
    (void)state;
    this->addExecutionTimeToHistory(state, executionTime);
    // // Calculate the 95th percentile of the execution time history for the state
    // std::deque<struct timeval> executionTimes = this->getExecutionTimeHistory(state);
    // std::sort(executionTimes.begin(), executionTimes.end(), [](const struct timeval &a, const struct timeval &b) {
    //     return (a.tv_sec * 1000000 + a.tv_usec) < (b.tv_sec * 1000000 + b.tv_usec);
    // });
    // int percentileIndex = static_cast<int>(0.95 * executionTimes.size());
    // this->executionTime = executionTimes[percentileIndex];

    this->executionTime = executionTime;
}

struct timeval Callback::getExecutionTime(const State &state)
{
    std::lock_guard mutex{execution_history_mutex_};
    (void)state;
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
    std::sort(non_state_aware_ex_time_history.begin(), non_state_aware_ex_time_history.end(), [](const struct timeval &a, const struct timeval &b) {
        return (a.tv_sec * 1000000 + a.tv_usec) < (b.tv_sec * 1000000 + b.tv_usec);
    });
    std::cout << "Execution time history size: " << non_state_aware_ex_time_history.size() << std::endl;
    int percentileIndex = static_cast<int>(0.95 * non_state_aware_ex_time_history.size());
    if (non_state_aware_ex_time_history.size() > 0)
        this->executionTime = non_state_aware_ex_time_history[percentileIndex];
    else
        this->executionTime = {0,0};
    std::cout << "Execution time: " << this->executionTime.tv_sec << "s " << this->executionTime.tv_usec << "us" << std::endl;
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
    if (timer_) timer_->callback_priority = priority;
    if (subscription_) subscription_->callback_priority = priority;
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
}
int Callback::getNumCruncherLimit()
{
    return num_cruncher_limit;
}

void Callback::addExecutionTimeToHistory(const State &state, const timeval &executionTime) {
    std::lock_guard mutex{execution_history_mutex_};
    (void)state;
    // if (executionTimeHistory.find(state) == executionTimeHistory.end()) {
    //     executionTimeHistory[state] = std::deque<struct timeval>();
    // }
    // executionTimeHistory[state].push_back(executionTime);
    non_state_aware_ex_time_history.push_back(executionTime);
    if(non_state_aware_ex_time_history.size() > 100){
        non_state_aware_ex_time_history.pop_front();
    }
    //std::cout << "Added execution time to history for callback: " << name << ": " << executionTime.tv_sec << "s " << executionTime.tv_usec << "us" << std::endl;
}

std::deque<struct timeval> Callback::getExecutionTimeHistory(const State &state) {
    std::lock_guard mutex{execution_history_mutex_};
    auto it = executionTimeHistory.find(state);
    if (it != executionTimeHistory.end()) {
        return it->second;
    }
    return {};  // Return an empty deque if the state is not found
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

extern thread_local size_t thread_id;

void Callback::execute_timer()
{
    // Increment and assign the sequence number atomically for the global sequence
    increment_sequence_number();                 // Increment the sequence number
    int chain_instance_id = getSequenceNumber(); // Capture the chain instance ID
    uint64_t time_until_next_period_usec = std::chrono::duration_cast<std::chrono::microseconds>(timer_->time_until_trigger()).count();
    uint64_t period_usec = timerPeriod.tv_sec * 1e6 + timerPeriod.tv_usec;
    add_branch_timestamp(chain_instance_id, period_usec - time_until_next_period_usec); // Add the initial timestamp for the chain instance

    //std::cout << "Executing callback " << name << "(thread " << thread_id << ")" << std::endl;
    std::cout << "Executing timer callback " << name << "(thread " << thread_id << ") Instance: " << chain_instance_id << 
        " Elapsed from release(ms): " << (period_usec - time_until_next_period_usec) / 1000. << std::endl;

    volatile uint64_t result = number_cruncher(num_cruncher_limit);
    (void)result;

    auto message = test_interfaces::msg::TestString();
    message.data = std::to_string(chain_instance_id);
    if (publisher_) publisher_->publish(message);
}

void Callback::execute_sub(const test_interfaces::msg::TestString::SharedPtr msg)
{
    int chain_instance_id = std::stoi(msg->data); // Capture the chain instance ID
    std::cout << "Executing sub callback " << name << "(thread " << thread_id << ") Instance: " << chain_instance_id << std::endl;
    add_branch_timestamp(chain_instance_id); // Add the timestamp for the specific instance

    volatile uint64_t result = number_cruncher(num_cruncher_limit);

    auto message = test_interfaces::msg::TestString();
    message.data = std::to_string(chain_instance_id);
    if (publisher_) publisher_->publish(message);
}

void Callback::check_if_last_callback(int chain_instance_id)
{
    if (chain->getCallbacks().size() == placeInChain + 1) {    
        add_branch_timestamp(chain_instance_id);

        // std::cout << "Callback: " << getName() << " ending chain with timestamp: "
        //           << get_last_branch_timestamp(chain_instance_id).tv_sec << "s "
        //           << get_last_branch_timestamp(chain_instance_id).tv_usec << "us" << std::endl;
        struct timeval chain_stop_time = get_last_branch_timestamp(chain_instance_id);
        struct timeval chain_base_time = chain->getFirstCallback()->get_last_branch_timestamp(chain_instance_id);
        struct timeval chain_ex_time;
        timersub(&chain_stop_time, &chain_base_time, &chain_ex_time);
        chain->add_response_time_to_history(exec->get_state(), get_branch_id(), chain_ex_time);
        // struct timeval chain_ex_time = {chain_stop_time.tv_sec - chain_base_time.tv_sec,
        //                                 chain_stop_time.tv_usec - chain_base_time.tv_usec};

        // callback->addExecutionTimeToHistory(chain_ex_time);

        std::cout << "Chain " << getChainID()
                  << " Branch " << get_branch_id()
                  << " Instance: " << chain_instance_id
                  << " Execution Time: " << chain_ex_time.tv_sec << "s "
                  << chain_ex_time.tv_usec << "us" << std::endl;
    }
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
}

std::shared_ptr<Chain> Callback::getChain()
{
    return chain;
}

void Callback::set_callback_affinity(uint64_t affinity_mask)
{
    if (timer_) timer_->callback_affinity = affinity_mask;
    if (subscription_) subscription_->callback_affinity = affinity_mask;
}

uint64_t Callback::get_callback_affinity()
{
    if (timer_) return timer_->callback_affinity;
    if (subscription_) return subscription_->callback_affinity;
    return 0;
}

#endif // CALLBACK_CPP
