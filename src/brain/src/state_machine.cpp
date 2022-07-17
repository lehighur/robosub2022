#include <cstdio>

#include "state_machine.h"
#include "qual.h"
// use this as base class and implement
// state machines for each task
// look into this too:
// https://github.com/ros2/examples/blob/master/rclcpp/services/async_client/main.cpp
// also,
// https://design.ros2.org/articles/realtime_background.html#multithreaded-programming-and-synchronization
// and,
// https://orenbell.com/?p=436
StateMachine::StateMachine() : current_state(0) { printf("StateMachine constructor\n"); }

uint8_t StateMachine::get_state() { return this->current_state; }

void StateMachine::set_state(STATE s) { this->current_state = s; }

int StateMachine::run() {
  while (this->current_state != STATE::DONE) {
    switch (this->current_state) {
      case 0:
        // STATE::IDLE
        break;
      case 1:
        // STATE::TEST
        break;
      case 2:
        // STATE::QUAL
        Qual qual;
        qual.run();
        break;
    }
  }
  return 0;
}
