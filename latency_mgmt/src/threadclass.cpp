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
    }
    // for( auto &chain : tg1->chains){
    //     chains.push_back(chain);
    // }
    for( auto &chain : tg2->chains){
        chains.push_back(chain);
        for(auto &thread : threads){
            thread->add_chain_to_thread(chain);
        }
    }
    num_threads = tg1->num_threads + tg2->num_threads;
    rt_threadclass = tg1->rt_threadclass && tg2->rt_threadclass;
    utilization = tg1->utilization + tg2->utilization;
    

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