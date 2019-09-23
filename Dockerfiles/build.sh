#! /bin/bash

# uncomment if you need proxy variables to be passed on
# BUILD_ARGS="--build-arg http_proxy=$http_proxy --build-arg https_proxy=$https_proxy"

DB="docker build $BUILD_ARGS"

cd dashing_source
$DB -t ros:dashing_source .
cd ../ros2_performance_ste_executor
$DB -t ros2_performance:ste_executor .
cd ../ros2_performance_static_executor
$DB -t ros2_performance:static_executor .
cd ../ros2_performance_let_executor
$DB -t ros2_performance:let_executor .
