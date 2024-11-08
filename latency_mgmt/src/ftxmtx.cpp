#ifndef FTXMTX_CPP
#define FTXMTX_CPP

#include <ftxmtx.hpp>


FutexMutex::FutexMutex() : futex_value(0) {}

void FutexMutex::lock() {
    int expected = 0;

    // Attempt to lock by setting futex_value to 1
    if (!futex_value.compare_exchange_strong(expected, 1, std::memory_order_relaxed)) {
        // If the futex_value is non-zero, we need to wait
        while (true) {
            if (futex_value.load(std::memory_order_relaxed) == 1) {
                // Wait on futex if futex_value is still 1
                if (futex_wait(&futex_value, 1) != 0 && errno != EAGAIN) {
                    perror("futex_wait failed");
                    return;
                }
            }
            expected = 0;
            // Try to lock again
            if (futex_value.compare_exchange_strong(expected, 1, std::memory_order_relaxed)) {
                break;
            }

            // Avoid busy waiting, yield to the OS
            sched_yield();
        }
    }
}

void FutexMutex::unlock() {
    futex_value.store(0, std::memory_order_relaxed);
    futex_wake(&futex_value, 1);  // Wake one waiting thread, if any
}

int FutexMutex::futex_wait(std::atomic<int>* addr, int expected) {
    return syscall(SYS_futex, addr, FUTEX_WAIT, expected, NULL, NULL, 0);
}

int FutexMutex::futex_wake(std::atomic<int>* addr, int count) {
    return syscall(SYS_futex, addr, FUTEX_WAKE, count, NULL, NULL, 0);
}

#endif

// #ifndef FTXMTX_CPP
// #define FTXMTX_CPP

// #include <ftxmtx.hpp>


// FutexMutex::FutexMutex() : futex_value(0) {}

// void FutexMutex::lock() {
//     int expected = 0;
//     if (!futex_value.compare_exchange_strong(expected, 1)) {
//         // If the value is already non-zero, wait
//         while (true) {
//             if (futex_wait(&futex_value, 1) != 0 && errno != EAGAIN) {
//                 perror("futex_wait failed");
//                 return;
//             }
//             expected = 0;
//             if (futex_value.compare_exchange_strong(expected, 1)) {
//                 break;
//             }
//         }
//     }
// }

// void FutexMutex::unlock() {
//     futex_value.store(0);
//     futex_wake(&futex_value, 1);  // Wake one waiting thread, if any
// }

// int FutexMutex::futex_wait(std::atomic<int>* addr, int expected) {
//     return syscall(SYS_futex, addr, FUTEX_WAIT, expected, NULL, NULL, 0);
// }

// int FutexMutex::futex_wake(std::atomic<int>* addr, int count) {
//     return syscall(SYS_futex, addr, FUTEX_WAKE, count, NULL, NULL, 0);
// }

// #endif