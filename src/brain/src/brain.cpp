#include <cstdio>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

// how do I handle double dependencies or
// does the compiler handle that?
#include "state_machine.h"
#include "event_queue.h"

typedef std_msgs::msg::Header RHeader;
typedef std_msgs::msg::String RString;
typedef sensor_msgs::msg::BatteryState RBatteryState;
typedef mavros_msgs::msg::State RState;
typedef mavros_msgs::msg::ManualControl RManualControl; 

class Brain : public rclcpp::Node {
  public:
    Brain() : Node("Brain"), count(0) {
      printf("Brain constructor\n");
      StateMachine sm;
      EventQueue eq;
      mc_pub = this->create_publisher<RManualControl>("/mavros/manual_control/control", 10);
      brain_pub = this->create_publisher<std_msgs::msg::String>("/brain", 10);
      //battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", 10, battery_callback);
    }

    uint8_t get_state() {
      return sm.get_state();
    }

  RManualControl create_manual_msg(int x, int y, int z, int r, int buttons) {
    RManualControl msg;
    msg.x = x; msg.y = y; msg.z = z;
    msg.r = r; msg.buttons = buttons;

    //set header
    RHeader head;
    head.frame_id = "VECTORED_6DOF";
    //head.seq = count_;
    msg.header = head;

    return msg;
  }

  bool publish_manual_msg(RManualControl *msg) {
    mc_pub->publish(*msg);
    return true;
  }

  private:
    StateMachine sm;
    std::size_t count;
    EventQueue eq;

    // Publishers
    rclcpp::Publisher<RString>::SharedPtr brain_pub;
    rclcpp::Publisher<RManualControl>::SharedPtr mc_pub;

    // Subscribers
    //rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
};

int main(int argc, char ** argv) {
  printf("hello world brain package\n");
  // this works but look at this for improvements:
  // https://github.com/ros2/examples/blob/master/rclcpp/executors/multithreaded_executor/multithreaded_executor.cpp
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto brain_node = std::make_shared<Brain>();

  executor.add_node(brain_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
