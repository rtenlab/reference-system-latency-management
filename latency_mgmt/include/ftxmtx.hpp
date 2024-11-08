#ifndef FTXMTX_HPP
#define FTXMTX_HPP
#include <atomic>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <atomic>
#include <cstdio> 
#include <cerrno> 
#include <unistd.h> 
#include <sched.h>    // for sched_yield
#include <cerrno>     // for errno
class FutexMutex {
public:
    FutexMutex();

    void lock();

    void unlock();

private:
    std::atomic<int> futex_value;

    int futex_wait(std::atomic<int>* addr, int expected);

    int futex_wake(std::atomic<int>* addr, int count);
};

#endif
