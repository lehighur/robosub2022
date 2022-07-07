// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
//#include <uint16.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
/* message structure for manual_control
 * from mavros_msgs/msg/manual_control.hpp
 * // Include directives for member types
 * // Member 'header'
 * #include "std_msgs/msg/detail/header__struct.h"
 * 
 * // Struct defined in msg/ManualControl in the package mavros_msgs.
 * typedef struct mavros_msgs__msg__ManualControl
 * {
 *   std_msgs__msg__Header header;
 *   float x;
 *   float y;
 *   float z;
 *   float r;
 *   uint16_t buttons;
 * } mavros_msgs__msg__ManualControl;
 *
 * ---rc_override message definition---
 * std_msgs/Header header
 * uint8 rssi
 * uint16[] channels
 */
typedef mavros_msgs::msg::ManualControl manualMsg;
typedef mavros_msgs::msg::RCOut rcMsg;
//typedef mavros_msgs::msg::OverrideRCIn rcMsg;

manualMsg returnMsg(int x, int y, int z, int r, int buttons);

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    //    publisher_ = this->create_publisher<std_msgs::msg::String>("/mavros/manual_control/control", 10);
    //define message
    //define header
    std_msgs::msg::Header head;
    //    manualMsg msg;
    //    msg.x = 1000; msg.y = 1000; msg.z = 1000;
    //    msg.r = 0; msg.buttons = 0;
    publisher_ = this->create_publisher<rcMsg>("/mavros/rc/out", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    /*
    manualMsg msg;
    if(count_ > 0 && count_ <= 20){ //first ten seconds
      msg = returnMsg(250, 0,0,0,0);
    }
    else if(count_ > 20 && count_ <= 40){ //10 to 20 seconds
      msg = returnMsg(-250,0,0,0,0);
    }
    else if(count_ > 40 && count_ <= 60){ //20 to 30 seconds
      msg = returnMsg(0,250,0,0,0);
    }
    else if(count_ > 60 && count_ <= 80){ //30 to 40 seconds
      msg = returnMsg(0,-250,0,0,0);
    }
    else if(count_ > 80 && count_ <= 100){ //40 to 50 seconds
      msg = returnMsg(0,0,-250,0,0);
    }
    else if(count_ > 100 && count_ <= 120){ //50 to 60 seconds
      msg = returnMsg(0,0,250,0,0);
    }
    else{
      msg = returnMsg(0,0,0,0,0);

    }
    */
    rcMsg msg;
    //uint16 arr[8];
    /*
    short unsigned int arr[8];
    arr[0] = 0; arr[1] = 0; arr[2] = 0; arr[3] = 0;
    arr[4] = 1000; arr[5] = 1000; arr[6] = 1000; arr[7] = 1000;
    */
    //    msg.channels = arr;
    msg.channels[0] = 0; msg.channels[1] = 0; msg.channels[2] = 0; msg.channels[3] = 0;
    msg.channels[4] = 1000; msg.channels[5] = 1000; msg.channels[6] = 1000; msg.channels[7] = 1000;
    auto message = std_msgs::msg::String();
    message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rcMsg>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

manualMsg returnMsg(int x, int y, int z, int r, int buttons){
   manualMsg msg;
   msg.x = x; msg.y = y; msg.z = z;
   msg.r = r; msg.buttons = buttons;
   return msg;
}
