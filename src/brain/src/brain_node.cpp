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
using std::placeholders::_1;

class Brain : public rclcpp::Node {
  public:
    Brain() : Node("Brain"), q(), count(0) {
      this->declare_parameter("timeout");
      rclcpp::Parameter p = this->get_parameter("timeout");
      this->timeout = p.as_int();

      msg = lur::create_manual_msg(0, 0, 500, 0, 0);
      mc_pub = this->create_publisher<lur::RManualControl>("/mavros/manual_control/send", 10);
      //brain_pub = this->create_publisher<lur::RString>("/brain", 10);
      imu_sub = this->create_subscription<lur::RImu>("/mavros/imu/data", 10, std::bind(&Brain::imu_callback, this, _1));
      cam_sub = this->create_subscription<lur::Cam>("/camera", 10, std::bind(&Brain::cam_callback, this, _1));
      test_mc_sub = this->create_subscription<lur::RManualControl>("/lur/test/manual_control", 10, std::bind(&Brain::test_mc_callback, this, _1));
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

    // could make this an action server or a service
    bool publish_manual_msg(lur::RManualControl::SharedPtr msg) {
      mc_pub->publish(*msg);
      return true;
    }

  private:
    // need state stuff
    // like position and what not
    // back forward 
    // -1000 1000
    int x;
    // left right
    // -1000 1000
    int y;
    // up down
    // 0 1000
    int z;
    // clock counter
    // -1000 1000
    int r;
    int diff_x;
    int diff_y;
    int diff_z;
    int diff_r;
    lur::RManualControl msg;

    int timeout;


    TSQueue<lur::RManualControl::SharedPtr> q;
    StateMachine sm;

    rclcpp::TimerBase::SharedPtr timer;
    std::size_t count;

    // Publishers
    rclcpp::Publisher<lur::RString>::SharedPtr brain_pub;
    rclcpp::Publisher<lur::RManualControl>::SharedPtr mc_pub;

    // Subscribers
    //rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<lur::RImu>::SharedPtr            imu_sub;
    rclcpp::Subscription<lur::Cam>::SharedPtr         cam_sub;
    rclcpp::Subscription<lur::RManualControl>::SharedPtr  test_mc_sub;

    void imu_callback(const lur::RImu::SharedPtr msg) {
      // geometry_messages/Quaternion
      // x y z w
      // # This represents an orientation in free space in quaternion form.
      // float64 x 0
      // float64 y 0
      // float64 z 0
      // float64 w 1
      //msg->orientation

      // geometry_msgs/Vector3
      // x y z
      // # This represents a vector in free space.
      // # This is semantically different than a point.
      // # A vector is always anchored at the origin.
      // # When a transform is applied to a vector, only the rotational component is applied.
      // float64 x
      // float64 y
      // float64 z
      //msg->angular_velocity

      // geometry_msgs/Vector3
      // x y z
      //msg->linear_acceleration
    }

    void cam_callback(const lur::Cam::SharedPtr msg) {
      // lur::Cam message
      //std_msgs/Header header
      //uint32 class_id
      //float64 confidence
      //uint32 x
      //uint32 y
      // frame is 600x800
      
      // classes
      // 0 gmain_bouys
      // 1 bootlegger_bouys
      // 2 gate
      // 3 gman
      // 4 bootlegger
      // 5 path

      int col = msg->x - 400;
      int row = msg->y - 300;
      diff_r = col;
      diff_z = row;
      //diff_x = 1;
    }

    void mag_callback(const lur::RMag::SharedPtr msg) {
      // # Measurement of the Magnetic Field vector at a specific location.
      //
      // # If the covariance of the measurement is known, it should be filled in
      // # (if all you know is the variance of each measurement, e.g. from the datasheet,
      // #just put those along the diagonal)
      // # A covariance matrix of all zeros will be interpreted as "covariance unknown",
      // # and to use the data a covariance will have to be assumed or gotten from some
      // # other source
      //
      //
      // Header header                        # timestamp is the time the
      //                                      # field was measured
      //                                      # frame_id is the location and orientation
      //                                      # of the field measurement
      //
      // geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
      //                                      # field vector in Tesla
      //                                      # If your sensor does not output 3 axes,
      //                                      # put NaNs in the components not reported.
      //
      // float64[9] magnetic_field_covariance # Row major about x, y, z axes
      //                                      # 0 is interpreted as variance unknown
    }

    void temp_callback(const lur::RTemperature::SharedPtr msg) {
      //# Single temperature reading.

      //Header header           # timestamp is the time the temperature was measured
      //                        # frame_id is the location of the temperature reading

      //float64 temperature     # Measurement of the Temperature in Degrees Celsius

      //float64 variance        # 0 is interpreted as variance unknown
    }

    void test_mc_callback(const lur::RManualControl::SharedPtr msg) {
      q.enqueue(msg);
    }

    void timer_callback() {
      ++this->count;
      // use message headers?
      if (this->count > this->timeout * 2) exit(1);

      // for test node
      //if (!q.empty()) {
      //  //auto message = std_msgs::msg::String();
      //  //message.data = "Message " + std::to_string(count_++);// + " (x,y,z,r,b): (" + msg.x + "," + msg.y + "," + msg.z + "," + msg.r + "," + msg.buttons + ")";
      //  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //  RCLCPP_INFO(this->get_logger(), "Publishing");
      //  mc_pub->publish(*q.dequeue());
      //}
      //else printf("queue empty\n");
      
      float scaling_factor = 0.5;
      msg.r += diff_r * (2000 / 800) * scaling_factor;
      //msg.z += diff_z * (1000 / 600) * scaling_factor;
      msg.x = 500;
      mc_pub->publish(msg);  
      msg.r += diff_r * (2000 / 800) * scaling_factor;
      //msg.z += diff_z * (1000 / 600) * scaling_factor;
      msg.x = 0;
      diff_r = 0;
      //diff_z = 0;
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto brain_node = std::make_shared<Brain>();
  executor.add_node(brain_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
