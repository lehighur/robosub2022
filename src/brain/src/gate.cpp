#include <iostream>

using namespace std;

// use this as base class and implement
// state machines for each task
// look into this too:
// https://github.com/ros2/examples/blob/master/rclcpp/services/async_client/main.cpp
// also,
// https://design.ros2.org/articles/realtime_background.html#multithreaded-programming-and-synchronization
// and,
// https://orenbell.com/?p=436
class GateStateMachine : public rclcpp::Node {
  public:
    // might not want this
    GateStateMachine() : Node("GateStateMachine") {
      cout << "GateStateMachine constructor\n";
    }
    
    get_state() {
      return current_state;
    }

  private:
    utin8_t current_state;

    enum States {
      IDLE,
      LOOKING,
      FOUND,
      GATEDONE,
    };

};

int main(int argc, char ** argv) {
  (void) argc;
  (void) argv;

  printf("hello world gate state machine package\n");
  GateStateMachine gsm;
  while (gsm.get_state() != GATEDONE) {
  }
  return 0;
}
