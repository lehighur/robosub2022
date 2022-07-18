#include <cstdio>
#include <chrono>

#include "common.h"

class Subscriber : public rclcpp::Node {
public:
  Subscriber() : Node("Subscriber") {
    auto callback = [this](lur::RString msg) {
        this->topic_callback(msg);
      };

    subscription_ = this->create_subscription<std_msgs::msg::String>("/sm", 10, topic_callback);
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
};
