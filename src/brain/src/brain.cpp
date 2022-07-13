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
      //head.seq = count_;
      head.stamp.sec = 1;
      msg.header = head;

      return msg;
    }

    RManualControl create_wait_msg(uint32_t time) {
      RManualControl msg;
      //set header
      RHeader head;
      msg.header = head;
      //set header
      RHeader head;
      head.frame_id = "VECTORED_6DOF";
      head.stamp.sec = 0;
      head.stamp.nsec = time;
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

    void timer_callback() {
      if (!q.empty()) {
        RManualControl msg;
        msg = q.front();
        q.pop();
        //auto message = std_msgs::msg::String();
        //message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        if (msg.header.stamp.sec == 0) {
                RCLCPP_INFO(this->get_logger(), "Waiting");
          std::this_thread::sleep_for(std::chrono::seconds(msg.header.stamp.sec));
        }
        else {
                RCLCPP_INFO(this->get_logger(), "Publishing");
                mc_pub->publish(msg);
        }
      }
          else printf("queue empty\n");
    }
};

int main(int argc, char ** argv) {
  printf("hello world brain package\n");
  // this works but look at this for improvements:
  // https://github.com/ros2/examples/blob/master/rclcpp/executors/multithreaded_executor/multithreaded_executor.cpp
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto brain_node = std::make_shared<Brain>();
  RManualControl man_msg;
  man_msg.x = 0; man_msg.y = 0; man_msg.z = 0;
  man_msg.r = 0; man_msg.buttons = 0;
  brain_node->enq(brain_node->create_manual_msg(0, 0, 0, 0, 0));

  ifstream man_file("/home/lur/robosub22/test/man_test.txt");
  if (man_file.is_open())
  {
    string line;
    array<int, 5> arr;
    int index = 0;
    bool check = true;
    while ( getline (man_file, line) )
    {
      stringstream ss(line);
      string word;
      index = 0;
      while (ss >> word) {
        if (word == "#") {
	  uint32_t time;
	  ss >> time;
	  brain_node->enq(brain_node->create_wait_msg(time));
	  check = false;
	  break;
	}
        arr[index++] = stoi(word);
      }
      if (check) brain_node->enq(brain_node->create_manual_msg(arr[0], arr[1], arr[2], arr[3], arr[4]));
    }
    man_file.close();
  }
  else printf("Unable to open file `man_test.txt`\n");
  brain_node->enq(brain_node->create_manual_msg(0, 0, 0, 0, 0));

  executor.add_node(brain_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
