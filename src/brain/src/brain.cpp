#include <cstdio>
#include <cstdint>
#include <queue>
#include <fstream>
#include <chrono>
#include <thread>

// how do I handle double dependencies or
// does the compiler handle that?
#include "state_machine.h"
#include "ts_queue.h"
#include "common.h"

using namespace std;

class Brain : public rclcpp::Node {
  public:
    TSQueue<lur::RManualControl> q;
    StateMachine sm;

    Brain() : Node("Brain"), q(), count(0) {
      printf("Brain constructor\n");
      mc_pub = this->create_publisher<lur::RManualControl>("/mavros/manual_control/send", 10);
      brain_pub = this->create_publisher<lur::RString>("/brain", 10);
      //battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", 10, battery_callback);
      //imu_sub = this->create_subscription<lur::RImu>("/mavros/imu/data", 10, imu_callback);
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

    // @@ use custom messages and services?
    lur::RManualControl create_manual_msg(int x, int y, int z, int r, int buttons) {
      lur::RManualControl msg;
      msg.x = x; msg.y = y; msg.z = z;
      msg.r = r; msg.buttons = buttons;

      //set header
      lur::RHeader head;
      head.frame_id = "VECTORED_6DOF";
      msg.header = head;

      return msg;
    }

    bool publish_manual_msg(lur::RManualControl *msg) {
      mc_pub->publish(*msg);
      return true;
    }

  private:
    rclcpp::TimerBase::SharedPtr timer;
    std::size_t count;
    // temporary
    //queue<lur::RManualControl> q;

    // Publishers
    rclcpp::Publisher<lur::RString>::SharedPtr brain_pub;
    rclcpp::Publisher<lur::RManualControl>::SharedPtr mc_pub;

    // Subscribers
    //rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<lur::RImu>::SharedPtr imu_sub;

    void timer_callback() {
      if (!q.empty()) {
        lur::RManualControl msg;
        msg = q.dequeue();
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
  brain_node->q.enqueue(brain_node->create_manual_msg(0, 0, 500, 0, 0));

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
            brain_node->q.enqueue(brain_node->q.back());
          }
          break;
        }
        arr[index++] = stoi(word);
      }
      brain_node->q.enqueue(brain_node->create_manual_msg(arr[0], arr[1], arr[2], arr[3], arr[4]));
    }
    man_file.close();
  }
  else printf("Unable to open file `man_test.txt`\n");
  // maybe pass all 0s? not sure why it gets disarmed
  brain_node->q.enqueue(brain_node->create_manual_msg(0, 0, 500, 0, 0));

  executor.add_node(brain_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
