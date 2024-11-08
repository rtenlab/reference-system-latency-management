#pragma once
#ifndef CHAIN_HPP
#define CHAIN_HPP
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <random>
#include <callback.hpp>
#include <deque>
#include <state.hpp>
class State;
class Callback;
class Chain
{
public:
    Chain(int chainID);

    void setChainID(int chainID);
    int getChainID();

    void setPriorities(std::vector<int> priority);
    void setBranchPriority(int priority, size_t branch_id);
    int getBranchPriority(size_t branch_id);

    bool get_branch_rt(size_t branch_id);
    std::vector<int> getPriorities();

    boost::uuids::uuid getUUID();

    void addCallback(std::shared_ptr<Callback> callback);
    void addCallback(std::shared_ptr<Callback> callback, bool copy);

    void removeCallback(boost::uuids::uuid callbackUUID);
    std::vector<std::shared_ptr<Callback>>& getCallbacks();
    std::shared_ptr<Callback> getCallback(boost::uuids::uuid callbackUUID);
    std::shared_ptr<Callback> getFirstCallback();

    void clearCallbacks();
    void sortCallbacks();

    void setPeriod(struct timeval period);
    struct timeval getPeriod();

    void setBranchExecutionTime(struct timeval executionTime, size_t branch_id, State current_state);
    struct timeval getBranchExecutionTime(size_t branch_id, State current_state);

    void setDeadline(struct timeval deadline);
    struct timeval getDeadline();

    void setLatencyTarget(struct timeval latencyTarget, size_t branch_id, bool rt);
    std::vector<struct timeval>* getLatencyTargets();

    void printChain();
    void printCallbacks();

    int getNumCallbacks();
    int get_num_branches();
    std::deque<struct timeval> getChainExecutionTimeHistory() const;
    void add_response_time_to_history(State current_state, size_t branch_id, struct timeval execution_time);
    std::deque<struct timeval> get_branch_response_time_history(State current_state, size_t branch_id);


private:
    int chainID;
    std::vector<std::shared_ptr<Callback>> callbacks;
    boost::uuids::uuid uuid;
    int chain_type;
    std::vector<int> branch_priorities;
    struct timeval chain_period;
    struct timeval chain_execution_time = {0,0};
    struct timeval chain_deadline;
    bool branches = false;
    int num_branches = 0;
    std::vector<bool> branch_rt;
    std::vector<struct timeval> branch_latency_targets;
    std::map<State, std::vector<std::deque<struct timeval>>> branch_chain_history;
    std::deque<struct timeval> chainExecutionTimeHistory; // History of chain execution times

};

#endif // CHAIN_HPP
