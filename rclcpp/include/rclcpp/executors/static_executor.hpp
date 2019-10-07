// Copyright 2019 Nobleo Technology.
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

#ifndef RCLCPP__EXECUTORS__STATIC_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__STATIC_EXECUTOR_HPP_

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>
#include <future>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{

/// Static executor implementation
// This is the default executor created by rclcpp::spin.
class StaticExecutor : public executor::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(StaticExecutor)

  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  StaticExecutor(
    const executor::ExecutorArgs & args = executor::ExecutorArgs());

  /// Default destrcutor.
  RCLCPP_PUBLIC
  virtual ~StaticExecutor();

  /// StaticExecutor implementation of spin.
  // This function will block until work comes in, execute it, and keep blocking.
  // It will only be interrupt by a CTRL-C (managed by the global signal handler).
  RCLCPP_PUBLIC
  void
  spin();

protected:

  RCLCPP_PUBLIC
  void
  execute_wait_set(executor::ExecutableList & exec_list,
  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void
  get_timer_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_subscription_list(
   executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_service_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_client_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_waitable_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_executable_list(executor::ExecutableList & executable_list,
  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void run_collect_entities();

  RCLCPP_PUBLIC
  void refresh_wait_set(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void prepare_wait_set();

public:

  /*
    For running client:
    auto result = client->async_send_request(request);
    rclcpp::executors::StaticExecutor static_exec;
    static_exec.add_node(node);
    if (static_exec.spin_until_future_complete(result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  */
  template<typename ResponseT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  rclcpp::executor::FutureReturnCode
  spin_until_future_complete(
    std::shared_future<ResponseT> & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return rclcpp::executor::FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    rclcpp::executor::ExecutableList executable_list;
    run_collect_entities();
    get_executable_list(executable_list);
    while (rclcpp::ok(this->context_)) {
      execute_wait_set(executable_list, timeout_left);
      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(std::chrono::seconds(0));
      if (status == std::future_status::ready) {
        return rclcpp::executor::FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return rclcpp::executor::FutureReturnCode::TIMEOUT;
      }
      // Subtract the elapsed time from the original timeout.
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    return rclcpp::executor::FutureReturnCode::INTERRUPTED;
  }


private:
  RCLCPP_DISABLE_COPY(StaticExecutor)
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_EXECUTOR_HPP_