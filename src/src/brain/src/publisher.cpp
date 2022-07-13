#include <chrono>
#include <cstdio>
#include <memory>

#include "mavros_msgs/msg/manual_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

template <typename T> class Publisher {
public:
  Publisher() : count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/sm", 10);
    // publisher_ =
    // this->create_publisher<manualMsg>("/mavros/manual_control/control", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "hey" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};
