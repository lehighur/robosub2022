#!/bin/bash

#ros2 pkg create --build-type ament_cmake --node-name test_node test --dependencies rclcpp std_msgs mavros_msgs
pushd src
rosdep install -i --from-path test --rosdistro foxy -y && \
colcon build --packages-select test && \
popd
