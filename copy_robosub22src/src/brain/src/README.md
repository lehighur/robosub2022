# Brain
- Starts state machine(s)
- Subscribes to topics and manages event data

## Directory structure
- src
- - brain.cc
- - state\_machine.cc
- - publisher.cc  // try to make generic
- - subscriber.cc // try to make generic
- - event\_queue.cc
- CMakeLists.txt
- include
- - state\_machine.h
- package.xml

## Topics
### /brain
```
```

### /mavros/state
```
Type: mavros_msgs/msg/State

std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status

http://docs.ros.org/en/api/mavros_msgs/html/msg/State.html
```

### /sm
```
uint8_t current_state 
```

### /mavros/battery
```
Type: sensor_msgs/msg/BatteryState

std_msgs/Header header
float32 voltage # [V]
float32 current # [A]
float32 remaining # 0..1

http://docs.ros.org/en/api/mavros_msgs/html/msg/BatteryStatus.html
```

### /mavros/imu/data
### /mavros/imu/data\_raw
```
Type: sensor_msgs/msg/Imu

std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance

http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
```
### /mavros/imu/mag     
```
Type: sensor_msgs/msg/MagneticField

std_msgs/Header header
geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance

http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html
```
### /mavros/imu/temperature\_baro
```
Type: sensor_msgs/msg/Temperature

std_msgs/Header header
float64 temperature
float64 variance

http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
```
