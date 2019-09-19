FROM ros:dashing_source

RUN bash -c 'cd ~/ros2_ws/src/ros2 && git clone https://github.com/micro-ROS/rcl_executor.git'
RUN bash -c 'cd ~/ros2_ws/src/ros2 && git clone https://github.com/nobleo/ros2_performance_c_LET_executor.git'

RUN bash -c 'cd ~/ros2_ws && source install/local_setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install --packages-select rcl_executor ros2_performance_c_let_executor'

# Entrypoint setup
COPY ./ros_entrypoint.sh /
RUN sudo chmod a+x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
