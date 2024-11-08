#ifndef CHAIN_CPP
#define CHAIN_CPP

#include <chain.hpp>

Chain::Chain(int chainID)
    : chainID(chainID)
{
    uuid = boost::uuids::random_generator()(); // Generate a random UUID for the chain
}

void Chain::setChainID(int chainID)
{
    this->chainID = chainID;
}

int Chain::getChainID()
{
    return chainID;
}

void Chain::setBranchPriority(int priority, size_t branch_id)
{
    if (branch_priorities.size() <= branch_id)
    {
        branch_priorities.resize(branch_id + 1);
    }
    branch_priorities[branch_id] = priority;
}

int Chain::getBranchPriority(size_t branch_id)
{
    if (branch_priorities.size() <= branch_id)
    {
        std::cerr << "Branch ID " << branch_id << " does not exist in chain" << std::endl;
        return -1;
    }
    return branch_priorities[branch_id];
}

void Chain::setPriorities(std::vector<int> priority)
{
    this->branch_priorities = priority;
}

std::vector<int> Chain::getPriorities()
{
    return branch_priorities;
}

boost::uuids::uuid Chain::getUUID()
{
    return uuid;
}

int Chain::get_num_branches()
{
    return num_branches;
}

void Chain::addCallback(std::shared_ptr<Callback> callback)
{
    callbacks.push_back(callback);
    callback->set_raw_pointer(callback.get());
    callback->setSharedPtr(callback);
    if (callback->get_branch_id() > num_branches)
    {
        branches = true;
        num_branches = callback->get_branch_id();
    }
}
void Chain::addCallback(std::shared_ptr<Callback> callback, bool copy)
{
    callbacks.push_back(callback);
    if (copy)
    {
        callback->set_raw_pointer(callback.get());
        callback->setSharedPtr(callback);
        if (callback->get_branch_id() > num_branches)
        {
            branches = true;
            num_branches = callback->get_branch_id();
        }
    }
}
void Chain::removeCallback(boost::uuids::uuid callbackUUID)
{
    for (auto it = callbacks.begin(); it != callbacks.end(); ++it)
    {
        if ((*it)->getUUID() == callbackUUID)
        {
            callbacks.erase(it);
            break;
        }
    }
}

std::shared_ptr<Callback> Chain::getFirstCallback()
{
    if (callbacks.size() > 0)
    {
        return callbacks[0];
    }
    return nullptr;
}

std::vector<std::shared_ptr<Callback>>& Chain::getCallbacks()
{
    return callbacks;
}

void Chain::clearCallbacks()
{
    callbacks.clear();
}

void Chain::sortCallbacks()
{
    std::sort(callbacks.begin(), callbacks.end(), [](std::shared_ptr<Callback> a, std::shared_ptr<Callback> b)
              { return a->getPriority() < b->getPriority(); });
}

void Chain::setPeriod(struct timeval period)
{
    this->chain_period = period;
}

struct timeval Chain::getPeriod()
{
    return chain_period;
}

void Chain::setDeadline(struct timeval deadline)
{
    this->chain_deadline = deadline;
}

struct timeval Chain::getDeadline()
{
    return chain_deadline;
}

bool Chain::get_branch_rt(size_t branch_id)
{
    return branch_rt[branch_id];
}

void Chain::setLatencyTarget(struct timeval latencyTarget, size_t branch_id, bool rt)
{
    // this->latency_target = latencyTarget;
    if (branch_id >= branch_latency_targets.size())
    {
        branch_latency_targets.resize(branch_id + 1);
        branch_rt.resize(branch_id + 1);
    }
    branch_latency_targets[branch_id] = latencyTarget;
    branch_rt[branch_id] = rt;
}

std::vector<struct timeval> *Chain::getLatencyTargets()
{
    return &branch_latency_targets;
}

std::deque<struct timeval> Chain::getChainExecutionTimeHistory() const
{
    return chainExecutionTimeHistory;
}

void Chain::printChain()
{
    std::cout << "Chain UUID: " << boost::uuids::to_string(uuid) << std::endl;
    std::cout << "Chain Period: " << chain_period.tv_sec << "s " << chain_period.tv_usec << "us" << std::endl;
    std::cout << "Chain Execution Time: " << chain_execution_time.tv_sec << "s " << chain_execution_time.tv_usec << "us" << std::endl;
    std::cout << "Chain Deadline: " << chain_deadline.tv_sec << "s " << chain_deadline.tv_usec << "us" << std::endl;
    // std::cout << "Chain Latency Target: " << latency_target.tv_sec << "s " << latency_target.tv_usec << "us" << std::endl;
    std::cout << "Chain ID: " << chainID << std::endl;
    std::cout << "Number of Branches: " << num_branches << std::endl;
    std::cout << "Branch Priority: ";
    for (auto &priority : branch_priorities)
    {
        std::cout << priority << " ";
    }
    std::cout << std::endl;
    std::cout << "Branch RT: ";
    for (int i = 0; i <= num_branches; i++)
    {
        std::cout << branch_rt[i] << " ";
    }
    std::cout << std::endl;

    // std::cout << "Chain Priority: " << priority << std::endl;
}

void Chain::printCallbacks()
{
    for (auto &callback : callbacks)
    {
        callback->printCallback();
    }
}

int Chain::getNumCallbacks()
{
    return callbacks.size();
}

std::shared_ptr<Callback> Chain::getCallback(boost::uuids::uuid callbackUUID)
{
    for (auto &callback : callbacks)
    {
        if (callback->getUUID() == callbackUUID)
        {
            return callback;
        }
    }
    return nullptr;
}

void Chain::add_response_time_to_history(State current_state, size_t branch_id, struct timeval execution_time)
{
    // Ensure that current_state exists in the branch_chain_history map
    if (branch_chain_history.find(current_state) == branch_chain_history.end())
    {
        // If it doesn't exist, insert a new vector of deques for the current_state
        branch_chain_history[current_state] = std::vector<std::deque<struct timeval>>();
    }

    // Ensure the vector is large enough to accommodate the branch_id index
    if (branch_chain_history[current_state].size() <= branch_id)
    {
        // Resize the vector to accommodate the branch_id
        branch_chain_history[current_state].resize(branch_id + 1);
    }

    // Now it's safe to push_back the execution_time into the appropriate deque
    // branch_chain_history[current_state][branch_id].push_back(execution_time);
    if (branch_chain_history[current_state][branch_id].size() >= 10)
    {
        branch_chain_history[current_state][branch_id].pop_front();
    }
    branch_chain_history[current_state][branch_id].push_back(execution_time);
}

std::deque<struct timeval> Chain::get_branch_response_time_history(State current_state, size_t branch_id)
{
    if (branch_chain_history.find(current_state) == branch_chain_history.end())
    {
        return std::deque<struct timeval>();
    }
    return branch_chain_history[current_state][branch_id];
}

#endif // CHAIN_CPP
