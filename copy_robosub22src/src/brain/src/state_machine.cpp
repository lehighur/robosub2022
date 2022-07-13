#include <cstdio>

#include "state_machine.h"
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
