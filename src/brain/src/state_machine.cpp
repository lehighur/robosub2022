#include <cstdio>
#include <chrono>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>

//#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"

// should add headers and reimplement instead
#include "subscriber.cpp"
#include "publisher.cpp"
#include "thread_safe_queue.cpp"

using namespace std;

// use this as base class and implement
// state machines for each task
// look into this too:
// https://github.com/ros2/examples/blob/master/rclcpp/services/async_client/main.cpp
// also,
// https://design.ros2.org/articles/realtime_background.html#multithreaded-programming-and-synchronization
// and,
// https://orenbell.com/?p=436
class StateMachine : public rclcpp::Node {
  public:
    StateMachine() : Node("StateMachine") {
      cout << "StateMachine constructor\n";
    }

    get_state() {
      return current_state;
    }

  private:
    utin8_t current_state;

    enum States {
      IDLE,
      PREQUAL,
      COINFLIP,
      GATE,
      PATH,
      BUOY,
      BIN,
      TORPEDO,
      OCTAGON
    };

};


int main(int argc, char ** argv) {
  (void) argc;
  (void) argv;

  printf("hello world brain package\n");
  ThreadSafeQueue q;

  int x = 10;
  q.enqueue(x);
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
