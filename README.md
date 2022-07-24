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

### checklist
- camera (and imu?) locking using udev rules
- systemd startup script, can we start something or is it when it turns on it goes?
- qualification task, go forward, try to adjust and find distance of objects
- distance to bottom of pool using bottom camera? or just try to hold depth
- how to handle finishing task, timeout? time since last detection?
- change mode and arm from node
- add more parameters to launch files
- get imu data and use orientation
- better dev flow
- tests
- docs
- simulation?
- docker
- why does it get disarmed after stopping publishing? publishing all 0s?
