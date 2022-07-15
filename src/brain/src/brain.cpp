#include <cstdio>
#include <cstdint>
#include <queue>
#include <fstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

// how do I handle double dependencies or
// does the compiler handle that?
#include "state_machine.h"

using namespace std;

typedef std_msgs::msg::Header RHeader;
typedef std_msgs::msg::String RString;
typedef sensor_msgs::msg::BatteryState RBatteryState;
typedef mavros_msgs::msg::State RState;
typedef mavros_msgs::msg::ManualControl RManualControl; 
typedef sensor_msgs::msg::Imu RIMU; 

class Brain : public rclcpp::Node {
  public:
    Brain() : Node("Brain"), count(0) {
      printf("Brain constructor\n");
      mc_pub = this->create_publisher<RManualControl>("/mavros/manual_control/send", 10);
      brain_pub = this->create_publisher<RString>("/brain", 10);
      q = queue<RManualControl>(); 
      //battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", 10, battery_callback);
      //run_state_machine();
      timer = this->create_wall_timer(500ms, std::bind(&Brain::timer_callback, this));
    }

    //void run_state_machine() {
    //  while (sm.get_state() != STATE::DONE) {
    //    Event e = eq.dequeue();
    //    process_event(e);
    //    // do inside each from above ^^
    //    sm.process_event(e);
    //  }
    //}

    RManualControl create_manual_msg(int x, int y, int z, int r, int buttons) {
      RManualControl msg;
      msg.x = x; msg.y = y; msg.z = z;
      msg.r = r; msg.buttons = buttons;

      //set header
      RHeader head;
      head.frame_id = "VECTORED_6DOF";
      msg.header = head;

      return msg;
    }

    bool publish_manual_msg(RManualControl *msg) {
      mc_pub->publish(*msg);
      return true;
    }

    // check and return 1 if added
    int enq(RManualControl msg) {
      q.push(msg);
      return 0;
    }

    RManualControl get_front() {
      return q.front();
    }
    RManualControl get_back() {
      return q.back();
    }

  private:
    rclcpp::TimerBase::SharedPtr timer;
    StateMachine sm;
    std::size_t count;
    EventQueue eq;
    // temporary
    queue<RManualControl> q;

    // Publishers
    rclcpp::Publisher<RString>::SharedPtr brain_pub;
    rclcpp::Publisher<RManualControl>::SharedPtr mc_pub;

    // Subscribers
    //rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<RIMU>::SharedPtr imu_sub;

    void timer_callback() {
      if (!q.empty()) {
        RManualControl msg;
        msg = q.front();
        q.pop();
        //auto message = std_msgs::msg::String();
        //message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing");
        mc_pub->publish(msg);
      }
      else printf("queue empty\n");
    }
};

int main(int argc, char ** argv) {
  printf("hello world brain package\n");
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto brain_node = std::make_shared<Brain>();
  //RManualControl man_msg;
  brain_node->enq(brain_node->create_manual_msg(0, 0, 500, 0, 0));

  ifstream man_file("/home/lur/man_test.txt");
  if (man_file.is_open()) {
    string line;
    array<int, 5> arr;
    int index = 0;
    while (getline (man_file, line)) {
      stringstream ss(line);
      string word;
      index = 0;
      while (ss >> word) {
        if (word == "#") {
          int time;
          ss >> time;
          for (int i = 0; i < time * 2; ++i) {
            brain_node->enq(brain_node->get_back());
          }
          break;
        }
        arr[index++] = stoi(word);
      }
      brain_node->enq(brain_node->create_manual_msg(arr[0], arr[1], arr[2], arr[3], arr[4]));
    }
    man_file.close();
  }
  else printf("Unable to open file `man_test.txt`\n");
  // maybe pass all 0s? not sure why it gets disarmed
  brain_node->enq(brain_node->create_manual_msg(0, 0, 500, 0, 0));

  executor.add_node(brain_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
