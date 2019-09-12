FROM ros:dashing_source

RUN bash -c 'cd ~/ros2_ws/ && sudo rm -rf build/rclcpp install/rclcpp && cd src/ros2 && sudo rm -rf rclcpp'
RUN bash -c 'cd ~/ros2_ws/src/ros2 && git clone https://github.com/nobleo/rclcpp.git'

RUN bash -c 'cd ~/ros2_ws/src/ros2 && git clone https://github.com/nobleo/ros2_performance.git'

RUN bash -c 'cd ~/ros2_ws && colcon build --symlink-install --packages-select rclcpp ros2_performance'

# Entrypoint setup
COPY ./ros_entrypoint.sh /
RUN sudo chmod a+x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
