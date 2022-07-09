#include <cstdint>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "state_machine.h"

class Brain : public rclcpp::Node {
  public:
    Brain() : Node("Brain"), count(0) {
      printf("Brain constructor\n");
      StateMachine sm;
      state_pub = this->create_publisher<mavros_msgs::msg::State>("/mavros/state", 10);
      sm_pub = this->create_publisher<std_msgs::msg::String>("/sm", 10);
      brain_pub = this->create_publisher<std_msgs::msg::String>("/brain", 10);
      //battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", 10, battery_callback);
    }

    uint8_t get_state() {
      return this->sm.get_state();
    }

  private:
    StateMachine sm;
    size_t count;
    // event_queue

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sm_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brain_pub;
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
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
