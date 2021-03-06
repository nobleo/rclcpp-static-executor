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

#include "rclcpp/executors/static_executor.hpp"
#include "rclcpp/executable_list.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::StaticExecutor;
using rclcpp::executor::ExecutableList;

StaticExecutor::StaticExecutor(const rclcpp::executor::ExecutorArgs & args)
: executor::Executor(args) {}

StaticExecutor::~StaticExecutor() {}


void
StaticExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  rclcpp::executor::ExecutableList executable_list;
  run_collect_entities();
  get_executable_list(executable_list);

  while (rclcpp::ok(this->context_) && spinning.load()) {
    execute_wait_set(executable_list);
  }
}

void
StaticExecutor::get_timer_list(ExecutableList & exec_list)
{
  exec_list.timer.clear();      //in case the function is called twice timers will not append themselves
  exec_list.number_of_timer = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      for (auto & timer_ref : group->get_timer_ptrs()) {
        auto timer = timer_ref.lock();
        if (timer) {
          exec_list.timer.push_back(timer);
          exec_list.number_of_timer++;
          exec_list.callback_group = group;
          node = get_node_by_group(group);
        }
      }
    }
  }
}

void
StaticExecutor::get_subscription_list(ExecutableList & exec_list)
{
   exec_list.subscription.clear();   //in case the function is called twice subscribers will not append themselves
   exec_list.number_of_subscription = 0;
   for (auto & weak_node : weak_nodes_) {
     auto node = weak_node.lock();
     if (!node) {
       continue;
     }
     for (auto & weak_group : node->get_callback_groups()) {
       auto group = weak_group.lock();
       if (!group || !group->can_be_taken_from().load()) {
         continue;
       }
       for (auto & sub_ref : group->get_subscription_ptrs()) {
         auto subscription = sub_ref.lock();
         if (subscription) {
            exec_list.subscription.push_back(subscription);
            exec_list.number_of_subscription++;
          }
          exec_list.callback_group = group;
          node = get_node_by_group(group);
        }
      }
    }
}


void
StaticExecutor::get_service_list(ExecutableList & exec_list)
{
  exec_list.service.clear();      //in case the function is called twice service will not append themselves
  exec_list.number_of_service = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      for (auto & service_ref : group->get_service_ptrs()) {
        auto service = service_ref.lock();
        if (service) {
          exec_list.service.push_back(service);
          exec_list.number_of_service++;
          exec_list.callback_group = group;
          node = get_node_by_group(group);
        }
      }
    }
  }
}


void
StaticExecutor::get_client_list(ExecutableList & exec_list)
{
  exec_list.client.clear();      //in case the function is called twice client will not append themselves
  exec_list.number_of_client = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      for (auto & client_ref : group->get_client_ptrs()) {
        auto client = client_ref.lock();
        if (client) {
          exec_list.client.push_back(client);
          exec_list.number_of_client++;
          exec_list.callback_group = group;
          node = get_node_by_group(group);
        }
      }
    }
  }
}


void
StaticExecutor::get_waitable_list(ExecutableList & exec_list)
{
  exec_list.waitable.clear();
  exec_list.number_of_waitable = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      for (auto & waitable_ref : group->get_waitable_ptrs()) {
        auto waitable = waitable_ref.lock();
        if (waitable) {
          exec_list.waitable.push_back(waitable);
          exec_list.number_of_waitable++;
          exec_list.callback_group = group;
          node = get_node_by_group(group);
        }
      }
    }
  }
}

void
StaticExecutor::get_executable_list(
  ExecutableList & executable_list, std::chrono::nanoseconds timeout)
{
  // prepare the wait_set
  prepare_wait_set();
  refresh_wait_set(timeout);

  // Check the timers to see if there are any that are ready, if so return
  get_timer_list(executable_list);

  // Check the subscriptions to see if there are any that are ready
  get_subscription_list(executable_list);

  get_service_list(executable_list);

  get_client_list(executable_list);

  get_waitable_list(executable_list);
}

// Function to run the callbacks from wait_set directly
void
StaticExecutor::execute_wait_set(
  ExecutableList & exec_list, std::chrono::nanoseconds timeout)
{
    refresh_wait_set(timeout);  //need to change to refresh_wait_set
    for (size_t i = 0; i < wait_set_.size_of_subscriptions; ++i) {
      if (wait_set_.size_of_subscriptions && i < exec_list.number_of_subscription) {
        if (wait_set_.subscriptions[i]) {
          if (exec_list.subscription[i]->get_intra_process_subscription_handle()) {
            execute_intra_process_subscription(exec_list.subscription[i]);
          }
          else {
            execute_subscription(exec_list.subscription[i]);  //run the callback
          }
        }
      }
    }

    for (size_t i = 0; i < wait_set_.size_of_timers; ++i) {
      if (wait_set_.size_of_timers && i < exec_list.number_of_timer) {
        if (wait_set_.timers[i] && exec_list.timer[i]->is_ready()) {
            execute_timer(exec_list.timer[i]);
        }
      }
    }

    for (size_t i = 0; i < wait_set_.size_of_services; ++i) {
     if (wait_set_.size_of_services && i < exec_list.number_of_service) {
        if (wait_set_.services[i]) {
            execute_service(exec_list.service[i]);
        }
      }
    }

   for (size_t i = 0; i < wait_set_.size_of_clients; ++i) {
      if (wait_set_.size_of_clients && i < exec_list.number_of_client) {
        if (wait_set_.clients[i]) {
            execute_client(exec_list.client[i]);
        }
      }
    }

    for (size_t i = 0; i < exec_list.number_of_waitable; ++i) {
      if (exec_list.number_of_waitable && exec_list.waitable[i]->is_ready(&wait_set_)) {
        exec_list.waitable[i]->execute();
      }
    }

    for (size_t i = 0; i < wait_set_.size_of_guard_conditions; ++i) {
      if (wait_set_.guard_conditions[i] || guard_conditions_.size() != old_number_of_guard_conditions_) {
        // rebuild the wait_set
        run_collect_entities();
        get_executable_list(exec_list);
      }
    }

}

void
StaticExecutor::run_collect_entities()
{
  memory_strategy_->clear_handles();
  bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    auto node_it = weak_nodes_.begin();
    auto gc_it = guard_conditions_.begin();
    while (node_it != weak_nodes_.end()) {
      if (node_it->expired()) {
        node_it = weak_nodes_.erase(node_it);
        memory_strategy_->remove_guard_condition(*gc_it);
        gc_it = guard_conditions_.erase(gc_it);
      } else {
        ++node_it;
        ++gc_it;
      }
    }
  }
}

void
StaticExecutor::prepare_wait_set()
{
  // Collect the subscriptions and timers to be waited on

  // clear wait set
  if (rcl_wait_set_clear(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }
  old_number_of_guard_conditions_ = guard_conditions_.size();
  // The size of waitables are accounted for in size of the other entities
  rcl_ret_t ret = rcl_wait_set_resize(
    &wait_set_, memory_strategy_->number_of_ready_subscriptions(),
    memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
    memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
    memory_strategy_->number_of_ready_events());
  if (RCL_RET_OK != ret) {
    throw std::runtime_error(
            std::string("Couldn't resize the wait set : ") + rcl_get_error_string().str);
  }
}

void
StaticExecutor::refresh_wait_set(std::chrono::nanoseconds timeout)
{
    // clear wait set
  if (rcl_wait_set_clear(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  if (!memory_strategy_->add_handles_to_wait_set(&wait_set_)) {
    throw std::runtime_error("Couldn't fill wait set");
  }
  rcl_ret_t status =
    rcl_wait(&wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }
}