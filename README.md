# rclcpp
ROS2 dashing diademata rclcpp extended with a static executor POC

## Package goal
Provide a **proof of concept** of a static executor to show how much CPU overhead can be reduced. This executor was made in response to the findings here https://github.com/nobleo/ros2_performance and the discussion here https://discourse.ros.org/t/singlethreadedexecutor-creates-a-high-cpu-overhead-in-ros-2. The static executor is not yet ready to be used in actual products. Feel free to use it with your applications for testing. If you perform tests using this package please share your results on the discourse post. If we collect more data, we will be able to push for changes to the core ROS2 stack. 

## Test Results


## How to use
Replace your rclcpp folder with this version (Make sure you rebuild!).
You can then use the static executor by using: 
```
rclcpp::executors::StaticExecutor exec;
```
in source code instead of using the default:
```
rclcpp::executors::SingleThreadedExecutor exec;
```
no other changes to the source code are required. 
You can not use the StaticExecutor with rclcpp::spin(node), in this case the default SingleThreadedExecutor will be used. 
You have to explicitly create a static executor as follows:

```
rclcpp::executors::StaticExecutor exec;
auto node = std::make_shared<rclcpp::Node>("node_");
...
do other stuff
...
exec.add_node(node);
exec.spin();
...
rclcpp::shutdown();
return 0;
```

## Assumptions
The static executor makes the following assumptions:
1. The system does not change during runtime. This means that all nodes, callbackgroups, timers, subscriptions etc. are created before .spin() is called.
2. Happy flow. Since this is a proof of concept not much has been added yet in the form of checks and catches.

## Current functionality
The static executor has been tested with subscribers and timers. The other features such as clients, services and waitables are still being implemented and tested and may not function as expected. 

## Inner workings
