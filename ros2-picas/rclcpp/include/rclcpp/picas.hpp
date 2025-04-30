#ifndef RCLCPP__CB_SCHED_HPP_
#define RCLCPP__CB_SCHED_HPP_

#ifdef PICAS
//#define PICAS_DEBUG // comment this out for non-debug mode

//#define PICAS_THREAD_AFFINITY

#ifdef PICAS_DEBUG
  #define PICAS_INFO(fmt, ...) RCLCPP_INFO(rclcpp::get_logger("picas"), fmt, ##__VA_ARGS__)
#else
  #define PICAS_INFO(fmt, ...) ((void)0)
#endif

/*class atomic_bitmask {
  std::atomic<uint64_t> bits{0};
public:
  void set_flag(uint64_t flag) {
    uint64_t old_val, new_val;
    do {
      old_val = bits.load();
      new_val = old_val | flag;
    } while (!bits.compare_exchange_weak(old_val, new_val));
  }
  void clear_flag(uint64_t flag) {
    uint64_t old_val, new_val;
    do {
      old_val = bits.load();
      new_val = old_val & ~flag;
    } while (!bits.compare_exchange_weak(old_val, new_val));
  }
  uint64_t get_flag() {
    return bits.load();
  }
};

static inline bool is_timespec_greater(struct timespec &t1, struct timespec &t2)
{
  if (t1.tv_sec != t2.tv_sec) return t1.tv_sec > t2.tv_sec;
  return t1.tv_nsec > t2.tv_nsec;
}
static inline bool is_timespec_smaller(struct timespec &t1, struct timespec &t2)
{
  if (t1.tv_sec != t2.tv_sec) return t1.tv_sec < t2.tv_sec;
  return t1.tv_nsec < t2.tv_nsec;
}
static inline bool is_timespec_equal(struct timespec &t1, struct timespec &t2)
{
  return t1.tv_sec == t2.tv_sec && t1.tv_nsec == t2.tv_nsec;
}*/

#include <mutex>
#include <sstream>
#include <condition_variable>
#include <queue>
#include <pthread.h>
#include <cstring>
#include <stdexcept>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <atomic>
#include <cerrno>
#include <stdexcept>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstring>
extern thread_local size_t thread_id;
extern thread_local bool is_rt_thread;

/* Latency management executor mutex 
Depends on CONFIG_LAME_MUTEX=y in the linux kernel 
Performs ioctl call to the lame mutex device which interacts with the deadline scheduler 
to boost the thread priority of the thread. Grabbing and releasing the lock require ioctl calls. 
*/

class ordered_mutex {
  private:
      int fd_;
      static constexpr const char* DEVICE_PATH = "/dev/rt_be_mutex";
      
      // ioctl commands
      static constexpr unsigned long IOCTL_LOCK = _IOW('r', 1, int);
      static constexpr unsigned long IOCTL_UNLOCK = _IOW('r', 2, int);
      
      // Task types
      static constexpr int TASK_TYPE_RT = 1;
      static constexpr int TASK_TYPE_BE = 2;
      
      bool is_open() const {
          return fd_ >= 0;
      }
  
  public:
      ordered_mutex() : fd_(-1) {
          // Open the device file
          fd_ = open(DEVICE_PATH, O_RDWR);
          if (fd_ < 0) {
              // Handle error but don't throw - make this a soft failure
              // that falls back to regular mutex behavior
              PICAS_INFO("Failed to open RT mutex device: %s", strerror(errno));
          } else {
              PICAS_INFO("RT mutex device opened successfully (fd=%d)", fd_);
          }
      }
      
      ~ordered_mutex() {
          if (is_open()) {
              close(fd_);
              PICAS_INFO("RT mutex device closed");
          }
      }
      
      void lock() {
          if (!is_open()) {
              throw std::runtime_error("RT mutex device not available");
          }
          
          // Determine if this is an RT or BE task
          int task_type = is_rt_thread ? TASK_TYPE_RT : TASK_TYPE_BE;
          
          // Request the lock via ioctl
          if (ioctl(fd_, IOCTL_LOCK, &task_type) < 0) {
              throw std::runtime_error(std::string("RT mutex lock failed: ") + 
                                     strerror(errno));
          }
          
          PICAS_INFO("Thread %lu acquired RT mutex lock (RT=%d)", 
                    thread_id, is_rt_thread ? 1 : 0);
      }
      
      void unlock() {
          if (!is_open()) {
              throw std::runtime_error("RT mutex device not available");
          }
          
          // Release the lock via ioctl
          if (ioctl(fd_, IOCTL_UNLOCK, nullptr) < 0) {
              throw std::runtime_error(std::string("RT mutex unlock failed: ") + 
                                     strerror(errno));
          }
          
          PICAS_INFO("Thread %lu released RT mutex lock", thread_id);
      }
      
      // Check if the mutex kernel module is available
      static bool is_available() {
          int fd = open(DEVICE_PATH, O_RDWR);
          if (fd >= 0) {
              close(fd);
              return true;
          }
          return false;
      }
  };



class ordered_mutex_v1 {
private:
    std::atomic<uint32_t> futex_{0};  // futex

    void futex_wait(int op) {
        while (syscall(SYS_futex, &futex_, op, 1, nullptr, nullptr, 0) == -1) {
            if (errno != EINTR) { 
                throw std::runtime_error("Futex operation failed");
            }
        }
    }

public:
    ordered_mutex_v1() = default;

    ~ordered_mutex_v1() = default;

    void lock() {
        futex_wait(FUTEX_LOCK_PI);
    }

    void unlock() {
        if (syscall(SYS_futex, &futex_, FUTEX_UNLOCK_PI, 0, nullptr, nullptr, 0) == -1) {
            throw std::runtime_error("Futex unlock failed");
        }
    }
};







// class ordered_mutex {
//   std::atomic<bool> locked_{false};
//   std::queue<size_t> rt_wait_queue_;
//   std::queue<size_t> be_wait_queue_;
//   pthread_mutex_t m_ = PTHREAD_MUTEX_INITIALIZER;
//   std::atomic<size_t> owner_id_{0};

//   public:

//   using native_handle_type = pthread_mutex_t*;

//   ordered_mutex() {
//     pthread_mutexattr_t attr;

//     int res = pthread_mutexattr_init(&attr);
//     if (res != 0) {
//       throw std::runtime_error{std::string("cannot pthread_mutexattr_init: ") + std::strerror(res)};
//     }

//     res = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
//     if (res != 0) {
//       throw std::runtime_error{std::string("cannot pthread_mutexattr_setprotocol: ") + std::strerror(res)};
//     }

//     res = pthread_mutex_init(&m_, &attr);
//     if (res != 0) {
//       throw std::runtime_error{std::string("cannot pthread_mutex_init: ") + std::strerror(res)};
//     }
//   }

//   ~ordered_mutex() {
//     pthread_mutex_destroy(&m_);
//   }

//   void lock() {
//     auto res = pthread_mutex_lock(&m_);
//     if (res != 0) {
//       throw std::runtime_error(std::string("failed pthread_mutex_lock: ") + std::strerror(res));
//     }
//   }

//   void unlock() noexcept {
//     pthread_mutex_unlock(&m_);
//   }

//   bool try_lock() noexcept {
//     return pthread_mutex_trylock(&m_) == 0;
//   }

//   native_handle_type native_handle() noexcept {
//     return &m_;
//   };

// };

// class ordered_mutex {
//   std::atomic<bool> locked_{false};
//   std::queue<size_t> rt_wait_queue_;
//   std::queue<size_t> be_wait_queue_;
//   std::mutex global_mutex_;
//   std::atomic<size_t> owner_id_{0};

// public:
//   void lock() {
//     size_t tid = thread_id;
//     {
//       std::lock_guard<std::mutex> lock(global_mutex_);
//       if (is_rt_thread) {
//         rt_wait_queue_.push(tid);
//       } else {
//         be_wait_queue_.push(tid);
//       }
//     }

//     while (true) {
//       std::lock_guard<std::mutex> lock(global_mutex_);
//       if (!locked_.load(std::memory_order_acquire)) {
//         if (!rt_wait_queue_.empty() && rt_wait_queue_.front() == tid) {
//           rt_wait_queue_.pop();
//         } else if (!be_wait_queue_.empty() && be_wait_queue_.front() == tid) {
//           be_wait_queue_.pop();
//         } else {
//           continue;
//         }
        
//         locked_.store(true, std::memory_order_acquire);
//         owner_id_.store(tid, std::memory_order_release);
//         return;
//       }
//     }
//   }

//   void unlock() {
//     std::lock_guard<std::mutex> lock(global_mutex_);
//     if (owner_id_.load(std::memory_order_acquire) == thread_id) {
//       owner_id_.store(0, std::memory_order_release);
//       locked_.store(false, std::memory_order_release);
//     }
//   }
// };





// #if 1
// class ordered_mutex {
//     std::atomic<bool> locked_{false};
//     std::queue<size_t> rt_wait_queue_;
//     std::queue<size_t> be_wait_queue_;
//     std::mutex global_mutex_;
//     std::atomic<size_t> owner_id_{0};
//     std::condition_variable cv_;

// public:
//     bool try_lock() {
//         bool expected = false;
//         if (locked_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
//             owner_id_.store(thread_id, std::memory_order_release);
//             return true;
//         }
//         return false;
//     }

//     void lock() {
//         //use trylock to avoid queueing if possible -- not safe FIFO is not respected
//         //if (try_lock()) return;

//         // grab queue lock
//         std::unique_lock<std::mutex> lock(global_mutex_);
//         // push tid to queue
//         if (is_rt_thread) {
//             rt_wait_queue_.push(thread_id);
//         } else {
//             be_wait_queue_.push(thread_id);
//         }
//         // wait on cv while not first in queue
//         cv_.wait(lock, [this] {
//             if (locked_.load(std::memory_order_acquire)) return false;
            
//             size_t tid = thread_id;
//             if (!rt_wait_queue_.empty() && rt_wait_queue_.front() == tid) {
//                 rt_wait_queue_.pop();
//                 return true;
//             }
//             if (rt_wait_queue_.empty() && !be_wait_queue_.empty() && be_wait_queue_.front() == tid) {
//                 be_wait_queue_.pop();
//                 return true;
//             }
//             return false;
//         });
//         // set lock and owner
//         locked_.store(true, std::memory_order_release);
//         owner_id_.store(thread_id, std::memory_order_release);
//     }

//     void unlock() {
//         if (owner_id_.load(std::memory_order_acquire) == thread_id) {
//             owner_id_.store(0, std::memory_order_release);
//             locked_.store(false, std::memory_order_release);
            
//             std::lock_guard<std::mutex> lock(global_mutex_);
//             cv_.notify_all();
//         }
//     }
// };


//#endif



#if 0
class ordered_mutex {
  std::mutex mutex_;
  std::condition_variable cv_;
  bool locked_ = false;
  std::queue<size_t> rt_wait_queue_;
  std::queue<size_t> be_wait_queue_;

public:
  static std::string queue_to_string(std::queue<size_t> q) {
    std::stringstream ss;
    ss << "[";
    bool first = true;
    while (!q.empty()) {
      if (!first) ss << ",";
      ss << q.front();
      q.pop();
      first = false;
    }
    ss << "]";
    return ss.str();
  }

  void lock() {
    std::unique_lock<std::mutex> lock(mutex_);
    //PICAS_INFO("lock (%lu)", thread_id);

    if (locked_ || !rt_wait_queue_.empty() || (!be_wait_queue_.empty() && !is_rt_thread)) {
      if (is_rt_thread) {
        rt_wait_queue_.push(thread_id);
        while (locked_ || rt_wait_queue_.front() != thread_id) {
          cv_.wait(lock);
        }
        //PICAS_INFO("lock wait (winner: %lu): %s", thread_id, queue_to_string(rt_wait_queue_).c_str());
        rt_wait_queue_.pop();
      }
      else {
        be_wait_queue_.push(thread_id);
        while (locked_ || be_wait_queue_.front() != thread_id) {
          cv_.wait(lock);
        }
        //PICAS_INFO("lock wait (winner: %lu): %s", thread_id, queue_to_string(be_wait_queue_).c_str());
        be_wait_queue_.pop();
      }
    }
    else {
      //PICAS_INFO("lock no wait (winner: %lu): %s", thread_id, queue_to_string(be_wait_queue_).c_str());
    }
    locked_ = true;
  }

  void unlock() {
    std::lock_guard<std::mutex> lock(mutex_);
    locked_ = false;
    //PICAS_INFO("unlock (%lu)", thread_id);
    cv_.notify_all();
  }
};
#endif
#if 0
class ordered_mutex {
  std::mutex mutex_;
  std::condition_variable cv_;
  bool locked_ = false;
  std::queue<size_t> wait_queue_;

public:
  static std::string queue_to_string(std::queue<size_t> q) {
    std::stringstream ss;
    ss << "[";
    bool first = true;
    while (!q.empty()) {
      if (!first) ss << ",";
      ss << q.front();
      q.pop();
      first = false;
    }
    ss << "]";
    return ss.str();
  }

  void lock() {
    std::unique_lock<std::mutex> lock(mutex_);

    if (locked_ || !wait_queue_.empty()) {
      wait_queue_.push(thread_id);
      while (locked_ || wait_queue_.front() != thread_id) {
        cv_.wait(lock);
      }
      wait_queue_.pop();
    }
    else {
    }
    locked_ = true;
  }

  void unlock() {
    std::lock_guard<std::mutex> lock(mutex_);
    locked_ = false;
    cv_.notify_all();
  }
};
#endif
#ifdef PICAS_DEBUG
#include <sys/time.h>
#include <execinfo.h>
#include <cxxabi.h>

static inline void print_stacktrace() // from https://panthema.net/
{
  RCLCPP_INFO(rclcpp::get_logger("stack"), "trace start");

  // storage array for stack trace address data
  void* addrlist[20];

  // retrieve current stack addresses
  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void*));

  if (addrlen == 0) {
    RCLCPP_INFO(rclcpp::get_logger("stack"), "<empty>");
    return;
  }

  // resolve addresses into strings containing "filename(function+address)",
  // this array must be free()-ed
  char** symbollist = backtrace_symbols(addrlist, addrlen);

  // allocate string which will be filled with the demangled function name
  size_t funcnamesize = 256;
  char* funcname = (char*)malloc(funcnamesize);

  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < addrlen; i++)
  {
    char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char *p = symbollist[i]; *p; ++p)
    {
      if (*p == '(')
        begin_name = p;
      else if (*p == '+')
        begin_offset = p;
      else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset
	    && begin_name < begin_offset)
    {
	    *begin_name++ = '\0';
	    *begin_offset++ = '\0';
	    *end_offset = '\0';

	    // mangled name is now in [begin_name, begin_offset) and caller
	    // offset in [begin_offset, end_offset). now apply
	    // __cxa_demangle():

	    int status;
	    char* ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize, &status);
	    if (status == 0) {
        funcname = ret; // use possibly realloc()-ed string
        RCLCPP_INFO(rclcpp::get_logger("stack"), "  %s : %s+%s", symbollist[i], funcname, begin_offset);
	    }
	    else {
        // demangling failed. Output function name as a C function with
        // no arguments.
        RCLCPP_INFO(rclcpp::get_logger("stack"), "  %s : %s()+%s", symbollist[i], begin_name, begin_offset);
	    }
    }
    else
    {
	    // couldn't parse the line? print the whole line.
      RCLCPP_INFO(rclcpp::get_logger("stack"), "  %s", symbollist[i]);
    }
  }
  free(funcname);
  free(symbollist);
}
#endif

#else // PICAS

#define PICAS_INFO(fmt, ...) ((void)0)

#endif
#endif // RCLCPP__CB_SCHED_HPP_
