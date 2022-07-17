#ifndef COMMON_H
#define COMMON_H

#include <chrono>
#include <string>

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

// lur headers
#include "ts_queue.h"

// lur namespace
// contains constants and type definitions
namespace lur {

  inline constexpr double pi { 3.14159 };
  // ROS2 message types
  typedef std_msgs::msg::Header RHeader;
  typedef std_msgs::msg::String RString;
  typedef sensor_msgs::msg::BatteryState RBatteryState;
  typedef mavros_msgs::msg::State RState;
  typedef mavros_msgs::msg::ManualControl RManualControl; 
  typedef sensor_msgs::msg::Imu RImu; 

  // use a steady clock instead or something from ros?
  typedef std::chrono::time_point<std::chrono::system_clock> Timestamp;
  Timestamp now(){return std::chrono::system_clock::now();}

  // generic event type used in communication
  // might also need to create a custom ros msg
  struct Event {
    uint8_t id;
    std::string message;
    Timestamp timestamp;
  };
}

#endif
