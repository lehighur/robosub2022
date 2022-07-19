#include <cstdio>
#include <array>
#include <queue>
#include <fstream>
#include <sstream>

#include "common.h"

using namespace std;

class Test : public rclcpp::Node {
  public:
    Test() : Node("Test"), q() {
      test_mc_pub = this->create_publisher<lur::RManualControl>("/lur/test/manual_control", 10);
      timer = this->create_wall_timer(500ms, std::bind(&Test::publish_queue, this));
    }

    // make an internal service poss
    bool set_mode(string mode) {
      //# set FCU mode
      //#
      //# Known custom modes listed here:
      //# http://wiki.ros.org/mavros/CustomModes
      //
      //# basic modes from MAV_MODE
      //uint8 MAV_MODE_PREFLIGHT = 0
      //uint8 MAV_MODE_STABILIZE_DISARMED = 80
      //uint8 MAV_MODE_STABILIZE_ARMED = 208
      //uint8 MAV_MODE_MANUAL_DISARMED = 64
      //uint8 MAV_MODE_MANUAL_ARMED = 192
      //uint8 MAV_MODE_GUIDED_DISARMED = 88
      //uint8 MAV_MODE_GUIDED_ARMED = 216
      //uint8 MAV_MODE_AUTO_DISARMED = 92
      //uint8 MAV_MODE_AUTO_ARMED = 220
      //uint8 MAV_MODE_TEST_DISARMED = 66
      //uint8 MAV_MODE_TEST_ARMED = 194
      //
      //uint8 base_mode # filled by MAV_MODE enum value or 0 if custom_mode != ''
      //string custom_mode # string mode representation or integer
      //---
      //bool mode_sent # Mode known/parsed correctly and SET_MODE are sent

      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Set Mode");
      rclcpp::Client<lur::RSetMode>::SharedPtr client = node->create_client<lur::RSetMode>("Set Mode");
      auto request = std::make_shared<lur::RSetMode::Request>();
      //request->custom_mode = "MAV_MODE_MANUAL_ARMED";
      request->custom_mode = mode;
      //request->a = atoll(argv[1]);
      //request->b = atoll(argv[2]);
      while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          printf("Interrupted while waiting for the service. Exiting.");
          return false;
        }
        // might want to break here instead
        printf("service not available, waiting again...");
      }

      auto result = client->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        printf("%s", result.get()->mode_sent ? "true" : "false");
        return result.get()->mode_sent;
      }
      else {
        printf("Failed to call service set_mode");
      }
      return false;
    }

    void run_man_test(string path) {
      q.enqueue(lur::create_manual_msg(0, 0, 500, 0, 0));
      read_man_test_file(path);
      // maybe pass all 0s? not sure why it gets disarmed
      q.enqueue(lur::create_manual_msg(0, 0, 500, 0, 0));
    }

    void publish_queue() {
      while (!q.empty()) {
        printf("publishing\n");
        test_mc_pub->publish(q.dequeue());
      }
    }

    void read_man_test_file(string path) {
      //ifstream man_file("/home/lur/man_test.txt");
      ifstream man_file(path);
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
                q.enqueue(q.back());
              }
              break;
            }
            arr[index++] = stoi(word);
          }
          q.enqueue(lur::create_manual_msg(arr[0], arr[1], arr[2], arr[3], arr[4]));
        }
        man_file.close();
      }
      else printf("Unable to open file `man_test.txt`\n");
    }
  private:
    TSQueue<lur::RManualControl> q;
    rclcpp::Publisher<lur::RManualControl>::SharedPtr test_mc_pub;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("test node\n");
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<Test>();
      //uint8 MAV_MODE_STABILIZE_DISARMED = 80
      //uint8 MAV_MODE_STABILIZE_ARMED = 208
      //uint8 MAV_MODE_MANUAL_DISARMED = 64
      //uint8 MAV_MODE_MANUAL_ARMED = 192
      //uint8 MAV_MODE_GUIDED_DISARMED = 88
      //uint8 MAV_MODE_GUIDED_ARMED = 216
  bool set_check = test_node->set_mode("MAV_MODE_STABILIZE_ARMED");
  //test_node->set_mode("MAV_MODE_MANUAL_ARMED");
  //test_node->set_mode("MAV_MODE_GUIDED_ARMED");
  test_node->run_man_test("/robosub/src/test/src/man_test.txt");
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}
