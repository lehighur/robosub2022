#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

namespace lur {
  inline constexpr double pi { 3.14159 };
  typedef std_msgs::msg::Header RHeader;
  typedef std_msgs::msg::String RString;
  typedef sensor_msgs::msg::BatteryState RBatteryState;
  typedef mavros_msgs::msg::State RState;
  typedef mavros_msgs::msg::ManualControl RManualControl; 
  typedef sensor_msgs::msg::Imu RImu; 
}

#endif
