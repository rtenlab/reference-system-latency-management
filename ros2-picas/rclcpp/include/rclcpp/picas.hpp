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
#include <condition_variable>
#include <queue>

extern thread_local size_t thread_id;
extern thread_local bool is_rt_thread;

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
