#include <cstdio>
#include <array>
#include <queue>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

using namespace std::chrono_literals;
using namespace std;
 
typedef mavros_msgs::msg::ManualControl manualMsg;
typedef mavros_msgs::msg::OverrideRCIn rcMsg;

class RCPublisher : public rclcpp::Node {
  public:
    RCPublisher() : Node("RCPublisher"), count_(0) {
      publisher_ = this->create_publisher<rcMsg>("/mavros/rc/override", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&RCPublisher::timer_callback, this));
      q_ = queue<rcMsg>();
    }

    // check and return 1 if added
    int enq(rcMsg msg) {
      q_.push(msg);
      return 0;
    }
    private:
      void timer_callback() {
        if (!q_.empty()) {
          rcMsg msg;
          msg = q_.front();
          q_.pop();
          auto message = std_msgs::msg::String();
          message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
          publisher_->publish(msg);
        }
        else printf("queue empty\n");
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rcMsg>::SharedPtr publisher_;
    size_t count_;
    queue<rcMsg> q_;
};

class ManPublisher : public rclcpp::Node {
  public:
    ManPublisher() : Node("ManPublisher"), count_(0)
    {
      publisher_ = this->create_publisher<manualMsg>("/mavros/manual_control/control", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&ManPublisher::timer_callback, this));
      q_ = queue<manualMsg>();
    }

    // check and return 1 if added
    int enq(manualMsg msg) {
      q_.push(msg);
      return 0;
    }

  private:
    void timer_callback() {
      if (!q_.empty()) {
        manualMsg msg;
        msg = q_.front();
        q_.pop();
        auto message = std_msgs::msg::String();
        message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(msg);
      }
      else printf("queue empty\n");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<manualMsg>::SharedPtr publisher_;
  size_t count_;
  queue<manualMsg> q_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("starting test\n");
  rclcpp::init(argc, argv);

  //auto man_pub = std::make_shared<ManPublisher>();
  //manualMsg man_msg;
  //man_msg.x = 0; man_msg.y = 0; man_msg.z = 0;
  //man_msg.r = 0; man_msg.buttons = 0;
  //man_pub->enq(man_msg);

  //ifstream man_file("test/src/man_test.txt");
  //if (man_file.is_open())
  //{
  //  string line;
  //  array<int, 5> arr;
  //  int index = 0;
  //  while ( getline (man_file, line) )
  //  {
  //    stringstream ss(line);
  //    string word;
  //    index = 0;
  //    while (ss >> word) {
  //      arr[index++] = stoi(word);
  //    }
  //    man_msg.x = arr[0]; man_msg.y = arr[1]; man_msg.z = arr[2];
  //    man_msg.r = arr[3]; man_msg.buttons = arr[4];
  //    man_pub->enq(man_msg);
  //  }
  //  man_file.close();
  //}
  //else printf("Unable to open file `man_test.txt`\n");
  //
  //man_msg.x = 0; man_msg.y = 0; man_msg.z = 0;
  //man_msg.r = 0; man_msg.buttons = 0;
  //man_pub->enq(man_msg);
  //rclcpp::spin(man_pub);

  auto rc_pub = std::make_shared<RCPublisher>();
  rcMsg rc_msg;
  printf("%d\n", rc_msg.channels.size());
  rc_msg.channels[0] = 0; rc_msg.channels[1] = 0; rc_msg.channels[2] = 0; rc_msg.channels[3] = 0;
  rc_msg.channels[4] = 0; rc_msg.channels[5] = 0; rc_msg.channels[6] = 0; rc_msg.channels[7] = 0;
  for (int i = 8; i < 18; ++i) {
    rc_msg.channels[i] = 0;
  }
  rc_pub->enq(rc_msg);
  ifstream rc_file("test/src/rc_test.txt");
  if (rc_file.is_open())
  {
    string line;
    array<int, 8> arr;
    int index = 0;
    while ( getline (rc_file, line) )
    {
      stringstream ss(line);
      string word;
      index = 0;
      while (ss >> word) {
        arr[index++] = stoi(word);
      }
      rc_msg.channels[0] = arr[0]; rc_msg.channels[1] = arr[1]; rc_msg.channels[2] = arr[2]; rc_msg.channels[3] = arr[3];
      rc_msg.channels[4] = arr[4]; rc_msg.channels[5] = arr[5]; rc_msg.channels[6] = arr[6]; rc_msg.channels[7] = arr[7];
      rc_pub->enq(rc_msg);
    }
    rc_file.close();
  }
  else printf("Unable to open file: `rc_test.txt`\n");

  rc_msg.channels[0] = 0; rc_msg.channels[1] = 0; rc_msg.channels[2] = 0; rc_msg.channels[3] = 0;
  rc_msg.channels[4] = 0; rc_msg.channels[5] = 0; rc_msg.channels[6] = 0; rc_msg.channels[7] = 0;
  rc_pub->enq(rc_msg);
  rclcpp::spin(rc_pub);

  rclcpp::shutdown();
  return 0;
}
