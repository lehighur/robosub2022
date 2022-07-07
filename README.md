# robosub22

## brain
  dispatch all controls and decisions form here

## state machine
### states
  start
  prequal video
  start gate
  choose side
  buoy
  end

## use headers and abstract classes


### Commands
```
$ ros2 pkg create --build-type ament_cmake --node-name movement sub --dependencies rclcpp std_msgs mavros_msgs
$ ros2 pkg create --build-type ament_cmake --node-name state_machine brain --dependencies rclcpp std_msgs mavros_msgs
$ ros2 pkg create --build-type ament_cmake --node-name test_node test --dependencies rclcpp std_msgs mavros_msgs
```
