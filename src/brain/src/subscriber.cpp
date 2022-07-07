#include <cstdio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("Subscriber")
  {
    auto callback = [this](std_msgs::msg::String::SharedPtr msg) {
        this->topic_callback(msg);
      };

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/sm", 10, callback);
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
