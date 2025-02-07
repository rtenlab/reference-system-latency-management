#ifndef threadclass_CPP
#define threadclass_CPP
#include <threadclass.hpp>
threadclass::threadclass()
{
    rt_threadclass = false;
    total_budget = 0;
    num_threads = 0;
    id = 0;
}

threadclass::threadclass(bool rt_threadclass)
{
    this->rt_threadclass = rt_threadclass;
    total_budget = 0;
    num_threads = 0;
    id = 0;
}

threadclass::threadclass(bool rt_threadclass, int id)
{
    this->rt_threadclass = rt_threadclass;
    total_budget = 0;
    num_threads = 0;
    this->id = id;
}

void threadclass::add_thread(std::shared_ptr<executor_thread> thread)
{
    threads.push_back(thread);
    num_threads++;
}

void threadclass::add_chain(std::shared_ptr<Chain> chain)
{
    chains.push_back(chain);
    for( auto &callback : chain->getCallbacks()){
        callbacks.push_back(callback);
    }
    for(auto &thread : threads){
        thread->add_chain_to_thread(chain);
    }
}
void threadclass::remove_chain(std::shared_ptr<Chain> chain)
{
    chains.erase(std::remove(chains.begin(), chains.end(), chain), chains.end());
    for(auto &callback : chain->getCallbacks()){
        callbacks.erase(std::remove(callbacks.begin(), callbacks.end(), callback), callbacks.end());
    }
    for(auto &thread : threads){
        thread->remove_chain_from_thread(chain);
    }
}

void threadclass::remove_thread(std::shared_ptr<executor_thread> thread)
{
    threads.erase(std::remove(threads.begin(), threads.end(), thread), threads.end());
    num_threads--;
}

void threadclass::set_utilization(double utilization)
{
    this->utilization = utilization;
}

double threadclass::get_utilization()
{
    // if util is infinite, compute utilization
    if (utilization == std::numeric_limits<double>::infinity())
    {
        double total_util = 0, chain_util = 0;
        for (auto &chain : chains)
        {
            for(auto &callback : chain->getCallbacks()){
                chain_util += callback->getExecutionTime().tv_sec * 1e6 + callback->getExecutionTime().tv_usec;
            }
            total_util += chain_util / (chain->getPeriod().tv_sec * 1e6 + chain->getPeriod().tv_usec);
        }
        utilization = total_util;
        //return total_util;
    }
    return utilization;
}

void threadclass::apply_budgets(int budget_us)
{
    total_budget = budget_us;
    for (auto &thread : threads)
    {
        thread->set_budget(budget_us);
    }
}

int threadclass::get_period(){
    return thread_period;
}

void threadclass::set_period(int period){
    thread_period = period;
}

void threadclass::merge_threadclasss(std::shared_ptr<threadclass> tg1, std::shared_ptr<threadclass> tg2)
{
    // for (auto &thread : tg1->threads)
    // {
    //     threads.push_back(thread);
    // }
    for (auto &thread : tg2->threads)
    {
        threads.push_back(thread);
        std::cerr << "Thread: " << thread->get_threadID() <<  "added to threadclass " << id  << " from threadclass: " << tg2->id <<std::endl;
        std::cerr << "Threadclass " << id << " now has " << threads.size() << " threads" << std::endl;
    }
    
    // for( auto &chain : tg1->chains){
    //     chains.push_back(chain);
    // }
    for( auto &chain : tg2->chains){
        chains.push_back(chain);
    }
    for (auto &chain : chains)
    {
        for(auto &thread : threads){
            thread->add_chain_to_thread(chain);
        }
    }
    num_threads = tg1->num_threads + tg2->num_threads;
    rt_threadclass = tg1->rt_threadclass && tg2->rt_threadclass;
    // if thr threadclass already has 2+ threads, we need to recompute the utilization
    
    auto current_util_per_thread = tg1->utilization*tg1->num_threads;
    current_util_per_thread += tg2->utilization*tg2->num_threads;
    utilization = current_util_per_thread/num_threads;
    //utilization = (tg1->utilization + tg2->utilization);
    //utilization = tg1->utilization + tg2->utilization;
    

}

std::vector<std::shared_ptr<Chain>> threadclass::get_chains()
{
    return chains;
}

bool operator==(const threadclass &tg1, const threadclass &tg2)
{
    return &tg1 == &tg2;
}


#endif // threadclass_CPP