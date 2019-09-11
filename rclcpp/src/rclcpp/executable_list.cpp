
#include "rclcpp/executable_list.hpp"

using rclcpp::executor::ExecutableList;

ExecutableList::ExecutableList()
:
  number_of_subscription(0),
  number_of_subscription_intra_process(0),
  number_of_timer(0),
  number_of_service(0),
  number_of_client(0)
  {}

ExecutableList::~ExecutableList()
{
  // Make sure that discarded (taken but not executed) AnyExecutable's have
  // their callback groups reset. This can happen when an executor is canceled
  // between taking an AnyExecutable and executing it.
  if (callback_group) {
    callback_group->can_be_taken_from().store(true);
  }
}
