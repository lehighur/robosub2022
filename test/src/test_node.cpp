#include <cstdio>
#include <array>
#include <queue>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

using namespace std::chrono_literals;
using namespace std;
 
typedef mavros_msgs::msg::ManualControl manualMsg;
class Publisher : public rclcpp::Node {
  public:
    Publisher() : Node("Publisher"), count_(0)
    {
      publisher_ = this->create_publisher<manualMsg>("/mavros/manual_control/control", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
      q_ = queue<manualMsg>();
    }

    // check and return 1 if added
    int enq(manualMsg msg) {
      q_.push(msg);
      return 0;
    }

  private:
    void timer_callback() {
      manualMsg msg;
      if (!q_.empty()) {
        msg = q_.front();
        printf("popping\n");
        q_.pop();
      }
      auto message = std_msgs::msg::String();
      message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(msg);
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
  auto pub = std::make_shared<Publisher>();

  manualMsg msg;
  msg.x = 0; msg.y = 0; msg.z = 0;
  msg.r = 0; msg.buttons = 0;
  pub->enq(msg);
  ifstream file("test/src/test.txt");
  if (file.is_open())
  {
    string line;
    array<int, 5> arr;
    int index = 0;
    while ( getline (file, line) )
    {
      stringstream ss(line);
      string word;
      index = 0;
      while (ss >> word) {
        arr[index++] = stoi(word);
      }
      msg.x = arr[0]; msg.y = arr[1]; msg.z = arr[2];
      msg.r = arr[3]; msg.buttons = arr[4];
      pub->enq(msg);
    }
    file.close();
  }
  else printf("Unable to open file\n");
  
  msg.x = 0; msg.y = 0; msg.z = 0;
  msg.r = 0; msg.buttons = 0;
  pub->enq(msg);

  rclcpp::spin(pub);
  rclcpp::shutdown();
  return 0;
}
