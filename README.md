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
$ ros2 pkg create --build-type ament_cmake --node-name camera_node camera --dependencies rclcpp

$ rosdep install -i --from-path src --rosdistro foxy -y
 or
$ rosdep install -i --from-path src/test --rosdistro foxy -y
```


### install
```
reflash with jetpack
sudo apt update && sudo apt upgrade -y
add paths to bashrc
configure and build opencv
install ros
install mavros (sudo apt install ros-foxy-mavros)
install mavros-msgs (sudo apt install ros-foxy-mavros-msgs)
install geographic libs (mavros install docs)
install colcon (sudo apt install python3-colcon-common-extensions
install rosdep (sudo apt install python3-rosdep, also run init and update)
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
sudo gpasswd -a $USER dialout
```
