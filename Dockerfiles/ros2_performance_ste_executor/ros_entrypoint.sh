#!/bin/bash
set -e

# setup ros environment
source "/home/noblean/ros2_ws/install/local_setup.bash"
exec "$@"
