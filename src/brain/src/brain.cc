#include <cstdint>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"

#include "state_machine.h"

class Brain : public rclcpp::Node {
  public:
    Brain() : Node("Brain") {
      printf("Brain constructor\n");
      StateMachine sm;
    }

    uint8_t get_state() {
      return this->sm.get_state();
    }

  private:
    StateMachine sm;
};


int main(int argc, char ** argv) {
  printf("hello world brain package\n");
  // this works but look at this for improvements:
  // https://github.com/ros2/examples/blob/master/rclcpp/executors/multithreaded_executor/multithreaded_executor.cpp
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto subnode = std::make_shared<Subscriber>();
  auto pubnode = std::make_shared<Publisher>();
  auto smnode = std::make_shared<StateMachine>();

  executor.add_node(subnode);
  executor.add_node(pubnode);
  executor.add_node(smnode);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
