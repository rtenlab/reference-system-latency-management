// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <nvtx3/nvToolsExt.h>  // sometimes needed

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/utilities.hpp"

using rclcpp::executors::MultiThreadedExecutor;

#ifdef PICAS
extern thread_local size_t thread_id;
extern thread_local bool is_rt_thread;

#include <cerrno>
static long int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

// static long int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags)
//{
//   return syscall(__NR_sched_getattr, pid, attr, size, flags);
// }

#endif

MultiThreadedExecutor::MultiThreadedExecutor(
    const rclcpp::ExecutorOptions &options,
    size_t number_of_threads,
    bool yield_before_execute,
    std::chrono::nanoseconds next_exec_timeout)
    : rclcpp::Executor(options),
      yield_before_execute_(yield_before_execute),
      next_exec_timeout_(next_exec_timeout)
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0)
  {
    number_of_threads_ = 1;
  }
#ifdef PICAS
  cpus.clear();

  rt_attr.size = sizeof(rt_attr);
  rt_attr.sched_flags = 0;
  rt_attr.sched_nice = 0;
  rt_attr.sched_priority = 0;
  rt_attr.sched_policy = 0;
  rt_attr.sched_runtime = 0;
  rt_attr.sched_period = 0;
  rt_attr.sched_deadline = 0;
#endif
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true))
  {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard wait_lock{wait_mutex_};
    for (; thread_id < number_of_threads_ - 1; ++thread_id)
    {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  run(thread_id);
  for (auto &thread : threads)
  {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}
template <typename... Args>
std::string string_format(const std::string &format, Args... args)
{
  // First, find the size needed
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
  if (size_s <= 0)
  {
    throw std::runtime_error("Error in string_format while formatting.");
  }

  // Allocate the exact buffer
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);

  // Do the actual formatting
  std::snprintf(buf.get(), size, format.c_str(), args...);

  // Convert to std::string (excluding the null terminator)
  return std::string(buf.get(), buf.get() + size - 1);
}
uint32_t s_colorPalette[] = {
    0xFFFF0000, // red
    0xFF00FF00, // green
    0xFF0000FF, // blue
    0xFFFFFF00, // yellow
    0xFFFF00FF, // magenta
    0xFF00FFFF, // cyan
    0xFFFF8000, // orange
    0xFF808000, // olive
    0xFF808080  // gray
                // Add more if desired
};
class NvtxScopedRange
{
public:
  NvtxScopedRange(const std::string &message)
  {
    std::stringstream ss;
    ss << std::this_thread::get_id();
    std::string thread_name = ss.str();
    int color_idx = 0;
    if (strcmp(thread_name.substr(0, 2).c_str(), "RT"))
    {
      color_idx = std::stoi(thread_name.substr(10, 1).c_str()) - 1;
    }
    else
    {
      // num threads per threadclass = 4
      color_idx = 3 + std::stoi(thread_name.substr(10, 1));
    }
    nvtxEventAttributes_t eventAttrib = {};
    eventAttrib.version = NVTX_VERSION;
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    // Which thread am I on? Pick color based on the name
    uint32_t argbColor = s_colorPalette[color_idx];
    eventAttrib.colorType = NVTX_COLOR_ARGB;
    eventAttrib.color = argbColor;
    // Set the label
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii = message.c_str();
    // range_id_ = nvtxRangeStartA(message.c_str());
    nvtxRangePushEx(&eventAttrib);
  }
  ~NvtxScopedRange()
  {
    nvtxRangePop();
    // nvtxRangeEnd(range_id_);
  }

private:
  nvtxRangeId_t range_id_;
};

void MultiThreadedExecutor::run(size_t this_thread_number)
{
  (void)this_thread_number;
#ifdef PICAS
  thread_id = (int)this_thread_number;
  if (cpus.size() > 0)
  {
    if (cpus.size() <= thread_id)
    {
      PICAS_INFO("MultiThreadedExecutor: spin: Thread %lu (PID %ld): no CPU assigned", thread_id, gettid());
    }
    else
    {
      PICAS_INFO("MultiThreadedExecutor: spin: Thread %lu (PID %ld) on CPU %d", thread_id, gettid(), cpus[thread_id]);
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(cpus[thread_id], &cpuset);
      if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset))
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MultiThreadedExecutor: spin: Thread %lu: spin_cpu has an error", thread_id);
      }
    }
  }
  if (rt_attr.sched_policy != 0)
  {
    long int ret;
    unsigned int flags = 0;
    PICAS_INFO("MultiThreadedExecutor: spin: Thread %lu (PID %ld) %s prio %d", thread_id, gettid(),
               rt_attr.sched_policy == SCHED_FIFO ? "FIFO" : rt_attr.sched_policy == SCHED_RR     ? "RR"
                                                         : rt_attr.sched_policy == SCHED_DEADLINE ? "DEADLINE"
                                                                                                  : "N/A",
               rt_attr.sched_priority);
    ret = sched_setattr(0, &rt_attr, flags);
    if (ret < 0)
    {
      PICAS_INFO("MultiThreadedExecutor: spin: Thread %lu: sched_setattr has an error (%s)", thread_id, strerror(errno));
    }
  }
  // RT threads: Update is_rt_threads if RT/BE thread information was provided
  // when the executor was created. (by default, all threads are RT threads)
  if (rt_threads.size() > thread_id)
  {
    is_rt_thread = rt_threads[thread_id];
  }
#endif

  while (rclcpp::ok(this->context_) && spinning.load())
  {
    rclcpp::AnyExecutable any_exec;

#ifdef PICAS_THREAD_AFFINITY
    if (!(active_thread_mask & (1 << thread_id)))
    {
      std::unique_lock<std::mutex> lock(thread_sync_mutex);
      if (!(active_thread_mask & (1 << thread_id)))
        // thread_sync_cv.wait(lock);
        thread_sync_cv.wait_for(lock, std::chrono::milliseconds(100));
    }
#endif
    {
      NvtxScopedRange r(string_format("%s Thread %i getting lock", thread_id));
      PICAS_INFO("[run] thread %lu", thread_id);
      std::lock_guard wait_lock{wait_mutex_};
      {
        NvtxScopedRange r(string_format("%s Thread %i holding lock", is_rt_thread ? "RT" : "BE", thread_id));
        PICAS_INFO("[run] thread %lu - lock acquired", thread_id);
        if (!rclcpp::ok(this->context_) || !spinning.load())
        {
          return;
        }
        if (!get_next_executable(any_exec, next_exec_timeout_))
        {
          continue;
        }
      }
    }
    if (yield_before_execute_)
    {
      std::cerr << "Yielding before excecuting" << std::endl;
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}
