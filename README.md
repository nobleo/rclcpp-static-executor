# rclcpp
ROS2 dashing diademata rclcpp extended with a static executor POC

## Package goal
Provide a **proof of concept** of a static executor to show how much CPU overhead can be reduced. This executor was made in response to the findings here https://github.com/nobleo/ros2_performance and the discussion here https://discourse.ros.org/t/singlethreadedexecutor-creates-a-high-cpu-overhead-in-ros-2. The static executor is not yet ready to be used in actual products. Feel free to use it with your applications for testing. If you perform tests using this package please share your results on the discourse post. If we collect more data, we will be able to push for changes to the core ROS2 stack. 

## Test Results (Preliminary)
Depends on: 
- this version of rclcpp
- https://github.com/nobleo/ros2_performance/tree/master 
- https://github.com/nobleo/ros2_performance/tree/static_executor 

| Binary  | publishers | subscribers | ROS | ROS nodes | ROS timers | DDS participants |
| ------------- | ------------- |------------- |------------- |------------- |------------- |------------- |
| ros | 20  | 200 | yes | 10 | 10 | 10 |
| rosonenode | 20 | 200 | yes | 1 | 1 | 1 |
| FastRTPS | 20 | 200 | no | 0 | 0 | 1 |

For all binaries:
- Message frequency: 50Hz
- Message type: String
- Message content: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
- QOS: depth = 10, reliability = best effort, durability = transient local

![Alt text](/images/Static_executor_docker_stats.png?raw=true "Docker comparison between SingleThreadedExecutor, StaticExecutor and pure FastRTPS")

*These are initial results, we are still checking to make sure the executor is working as intended and the generated traffic is the same.*

## How to recreate test results
It is possible to git clone this repository, build the workspace using colcon build and inspect the CPU usage with top or a similar program for each binary individually. It is however much easier to give each binary its own container (make sure to separate their networks or give them a unique ROS_DOMAIN_ID) and measure the usage of each container.
If you don't have docker and docker compose installed first follow online tutorials on how to install these: https://docs.docker.com/install/ https://docs.docker.com/compose/install/ .

Assuming you replaced the existing rclcpp package in your ROS2 environment with this version perform the following steps:

1. Go to the repository with the dockerfiles
```
cd ~/ros2_ws/src/ros2/rclcpp/Dockerfiles
```
2. Build the images using the following names (building ROS2 from source will take along time! ~1 hour):
```
cd dashing_source
docker build -t ros:dashing_source .
cd ../ros2_performance_ste_executor
docker build -t ros2_performance:ste_executor .
cd ../ros2_performance_static_executor
docker build -t ros2_performance:static_executor .
```

3. Run the compose file 
```
cd ../compose
docker-compose up
```
4. Inspect the results in a new terminal (ctrl+alt+t)
```
docker stats --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.PIDs}}"
```

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

## Inner workings of the SingleThreadedExecutor
During initialization handles to nodes, callbackgroups, timers, subscriptions etc. are created in RCL and RMW.
The SingleThreadedExecutor works as follows:

**"Init:"**

1. The executor finds nothing to execute
2. The executor performs collect_entities (gets handles from RCL)
3. The executor creates a wait_set using these handles
4. The executor communicates with RMW -> DDS layer to see what callbacks are ready to execute (put NULL if not ready in the wait_set)
5. The NULL handles are removed from the wait_set -> a wait_set containing handles corresponding to executables that need to be executed is left

**Loop:**

6. The executor looks for the next thing to execute in the following order: timers, subscriptions, services, clients, waitables
7. The executor walks through a tree of weak_ptrs to nodes which contain callbackgroups that contain timers, subscriptions etc.
8. The executor checks if the handle inside the node inside the callbackgroup inside the timer corresponds to a handle in the wait_set
9. If a handle is found that is in the wait_set the corresponding callback is executed and the callbackgroup is updated. 
10. As long as there are still things to execute repeat 6 till 9, if nothing is left to execute go to 1

This process is visualized in the following flowchart:
![Alt text](/images/STE_flowchart.png?raw=true "STE flowchart")

## Changes in the StaticExecutor
The StaticExecutor works as follows:

**Init:**

1. The executor performs collect_entities (gets handles from RCL) ***ONCE***
2. The executor creates a struct with all executables with the same index as the handles in the wait_set by walking the tree of weak_ptrs ***ONCE***

**Loop:**

3. The executor adds RCL handles to the wait_set
4. The executor communicates with RMW -> DDS layer to see what callbacks are ready to execute (put NULL if not ready in the wait_set)
5. The executor walks over the wait_set, if it finds a NULL nothing happens, if a handle is found the executable is executed at the same index in the executables struct
6. Go to 3
