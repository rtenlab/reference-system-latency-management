#pragma once

#ifndef CALLBACK_HPP
#define CALLBACK_HPP
class executor;
class State;

#include <state.hpp>
#include <executor.hpp>
#include <ftxmtx.hpp>
#include <iostream>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <deque>
#include <memory>
#include <map>
#include <sys/time.h>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/msg/test_string.hpp"

class Callback;
class Chain;
struct callback_entry
{
    Callback *raw_ptr;
    int global_sequence_number;
    unsigned int chain_instance_id;
    bool operator<(const callback_entry &other) const
    {
        return global_sequence_number < other.global_sequence_number;
    }
};

enum class CallbackType
{
    TIMER,
    SUBSCRIPTION
}; 

class Callback : public rclcpp::Node
{
public:
    Callback(CallbackType type, struct timeval period, int chainID, int placeInChain, int priority, std::string name, std::string sub_topic, std::string pub_topic, int num_cruncher_limit, bool is_nonlinear = false);
    Callback(CallbackType type, struct timeval period, int chainID, int placeInChain, int priority, int branch_id, std::string name, std::string sub_topic, std::string pub_topic, int num_cruncher_limit, bool is_nonlinear = false);

    void setPeriod(struct timeval period);
    struct timeval getPeriod();

    void setExecutionTime(struct timeval executionTime, const State &state);
    struct timeval getExecutionTime(const State &state);

    void setChainID(int chainID);
    int getChainID();

    void setPriority(int priority);
    int getPriority();

    boost::uuids::uuid getUUID();

    void printCallback();

    std::deque<struct timeval> getExecutionTimeHistory(const State &state);
    void addExecutionTimeToHistory(const State &state, const timeval &executionTime);

    void setType(CallbackType type);
    CallbackType getType();

    void setName(std::string name);
    std::string getName();

    void setSequenceNumber(int sequence_number);
    int getSequenceNumber();

    void execute_timer();
    void execute_sub(const test_interfaces::msg::TestString::SharedPtr msg);

    void setNumCruncherLimit(int limit);

    int getPlaceInChain();
    void setPlaceInChain(int place_in_chain);

    std::shared_ptr<Callback> getNextCallback(std::shared_ptr<Callback> current_callback);
    void set_raw_pointer(Callback *callback);
    Callback *get_raw_pointer();
    void setSharedPtr(std::shared_ptr<Callback> cb_ptr);
    std::shared_ptr<Callback> getSharedPtr();

    bool is_timer_running();
    void set_timer_running(bool running);

    bool is_non_linear();
    void set_nonlinear(bool is_nonlinear);

    int get_branch_id();
    void set_branch_id(int id);
    
    struct timeval get_branch_timestamp(int sequence_number);
    void increment_sequence_number();

    // Add a timestamp for a specific chain instance
    void add_branch_timestamp(int chain_instance_id);
    void add_branch_timestamp(int chain_instance_id, uint64_t elapsed_since_release_usec);
    struct timeval get_last_branch_timestamp(int chain_instance_id) const;
    struct timeval get_first_branch_timestamp(int chain_instance_id) const;
    std::deque<struct timeval> get_branch_timestamps(int chain_instance_id) const;

    void setPriorityScheduling(bool priority_scheduling);
    bool getPriorityScheduling();

    void setChain(std::shared_ptr<Chain> chain);
    std::shared_ptr<Chain> getChain();

    int getNumCruncherLimit();

    void set_callback_affinity(uint64_t affinity_mask);
    uint64_t get_callback_affinity();
    void stop_timer();
    void start_timer();
    void record_response_time(int chain_instance_id);
    std::mutex& get_mutex();

    rclcpp::TimerBase::SharedPtr timer_ = NULL;
    rclcpp::Publisher<test_interfaces::msg::TestString>::SharedPtr publisher_ = NULL;
    rclcpp::Subscription<test_interfaces::msg::TestString>::SharedPtr subscription_ = NULL;    
    executor* exec;

private:
    struct timeval timerPeriod;
    struct timeval executionTime;
    int chainID;
    boost::uuids::uuid uuid;
    int placeInChain;
    int priority;
    bool priority_scheduling = false;
    CallbackType type;
    std::string name;
    int sequence_number = 0;
    std::map<State, std::deque<struct timeval>> executionTimeHistory;
    int num_cruncher_limit = 65536;
    std::shared_ptr<Callback> shared_ptr;
    Callback *raw_pointer;
    bool timer_running = false; // Flag to track if the timer is running
    int branch_id = 0;          // New branch identifier
    bool is_nonlinear = false;
    std::unordered_map<int, std::deque<struct timeval>> branch_timestamps;
    std::shared_ptr<Chain> chain;
    std::deque<struct timeval> non_state_aware_ex_time_history;
    ordered_mutex execution_history_mutex_;
    std::mutex timer_mutex_;
};


#endif // CALLBACK_HPP
