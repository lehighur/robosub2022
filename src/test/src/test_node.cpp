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
  test_node->run_man_test("/robosub/src/test/src/man_test.txt");
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}
