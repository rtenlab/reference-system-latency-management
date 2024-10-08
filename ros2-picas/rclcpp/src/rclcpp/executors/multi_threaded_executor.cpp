// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/utilities.hpp"

using rclcpp::executors::MultiThreadedExecutor;

#ifdef PICAS
#include <cerrno>
static long int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

//static long int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags)
//{
//  return syscall(__NR_sched_getattr, pid, attr, size, flags);
//}
#endif

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: rclcpp::Executor(options),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout)
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
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
  rt_attr.sched_period  = 0;
  rt_attr.sched_deadline= 0;  
#endif
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard wait_lock{wait_mutex_};
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  run(thread_id);
  for (auto & thread : threads) {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

#ifdef PICAS
void
MultiThreadedExecutor::run(size_t thread_id)
{
  if (cpus.size() > 0 && cpus.size() <= thread_id) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MultiThreadedExecutor: spin: Thread %lu (PID %ld): no CPU assigned", thread_id, gettid());
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MultiThreadedExecutor: spin: Thread %lu (PID %ld) on CPU %d", thread_id, gettid(), cpus[thread_id]);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpus[thread_id], &cpuset);
    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MultiThreadedExecutor: spin: Thread %lu: spin_cpu has an error", thread_id);
    }
  }
  if (rt_attr.sched_policy != 0) {
    long int ret;
    unsigned int flags = 0;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MultiThreadedExecutor: spin: Thread %lu (PID %ld) %s prio %d", thread_id, gettid(), 
      rt_attr.sched_policy == SCHED_FIFO ? "FIFO" : rt_attr.sched_policy == SCHED_RR ? "RR" : rt_attr.sched_policy == SCHED_DEADLINE ? "DEADLINE" : "N/A",
      rt_attr.sched_priority);
    ret = sched_setattr(0, &rt_attr, flags);
    if (ret < 0) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MultiThreadedExecutor: spin: Thread %lu: sched_setattr has an error (%s)", thread_id, strerror(errno));
    }
  }
#else
void
MultiThreadedExecutor::run(size_t this_thread_number)
{
  (void)this_thread_number;
#endif
  //(void)this_thread_number; // comment out this line, couldn't find usage of it
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      std::lock_guard wait_lock{wait_mutex_};
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}
