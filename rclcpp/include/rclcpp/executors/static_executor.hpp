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


private:
  RCLCPP_DISABLE_COPY(StaticExecutor)
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_EXECUTOR_HPP_