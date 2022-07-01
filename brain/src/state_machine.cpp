#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "subscriber.cpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class StateMachine : public rclcpp::Node
{
public:
  StateMachine() : Node("StateMachine"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/sm", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&StateMachine::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "hey" + std::to_string(count_++);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world brain package\n");

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto subnode = std::make_shared<Subscriber>();
  auto smnode = std::make_shared<StateMachine>();
                                                   
  executor.add_node(subnode);
  executor.add_node(smnode);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
