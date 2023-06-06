#include <cstdio>
#include <cstdint>
#include <queue>
#include <fstream>
#include <chrono>
#include <thread>
#include <unordered_map>

// how do I handle double dependencies or
// does the compiler handle that?
#include "state_machine.h"
#include "ts_queue.h"
#include "common.h"

using namespace std;
using std::placeholders::_1;

class Brain : public rclcpp::Node {
  public:
    Brain() : Node("Brain"), q(), count(0), detections(), index(0), det_check(false), det_count(0) {
      RCLCPP_INFO(this->get_logger(), "constructors");
      this->declare_parameter("timeout");
      rclcpp::Parameter p = this->get_parameter("timeout");
      this->timeout = p.as_int();

      msg = lur::create_manual_msg(0, 0, 500, 0, 0);
      mc_pub = this->create_publisher<lur::RManualControl>("/mavros/manual_control/send", 10);
      // create custom message type
      //brain_pub = this->create_publisher<lur::RString>("/brain", 10);
      imu_sub = this->create_subscription<lur::RImu>("/mavros/imu/data", 10, std::bind(&Brain::imu_callback, this, _1));
      cam_sub = this->create_subscription<lur::Cam>("/lur/camera", 10, std::bind(&Brain::cam_callback, this, _1));
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
    bool det_check;
    int det_count;
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
    // counter clock
    // -1000 1000
    int r;
    int diff_x;
    int diff_y;
    int diff_z;
    int diff_r;
    lur::RManualControl msg;
    unordered_map<int, lur::Cam::SharedPtr> detections;
    int index;

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
      //
      //
      //RCLCPP_INFO(this->get_logger(), "IMU");
      //float x = msg->orientation.x;
      //float y = msg->orientation.y;
      //float z = msg->orientation.z;
      //float w = msg->orientation.w;
      //RCLCPP_INFO(this->get_logger(), "Orientation (x,y,z,w): %.4f, %.4f, %.4f, %.4f \n", x, y, z, w);
      //
      //
      //
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
      //if (index == 10) index = 0;
      //else ++index;
      //detections[index] = msg;
      // lur::Cam message
      //std_msgs/Header header
      //uint32 class_id
      //float64 confidence
      //uint32 x
      //uint32 y
      //uint32 width
      //uint32 height
      // frame is 600x800
      
      // classes
      // 0 gmain_bouys
      // 1 bootlegger_bouys
      // 2 gate
      // 3 gman
      // 4 bootlegger
      // 5 path
      if (msg->class_id == 0 || msg->class_id == 1) {
        this->det_check = true;
        diff_y = msg->x - 400;
        diff_z = msg->y - 300;
      }
      //exit(1);
      //if (msg->class_id == 3) {
      //  if (msg->x < 300 || msg->x > 500) {
      //    diff_r = msg->x - 400;
      //  }
      //  if (msg->y > 300) {
      //    diff_z = msg->y - 300;
      //  }
      //}
      //RCLCPP_INFO(this->get_logger(), "cam_callback");
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

    // move
    float r_scaling_factor = .50;
    float z_scaling_factor = .75;
    float y_scaling_factor = .50;
    void timer_callback() {
      //mc_pub->publish(lur::create_manual_msg(0, 0, 500, 0, 0b0000000001001000));
      ++this->count;
      if (this->count > this->timeout * 2) exit(1);

      if (this->count <= 85) {
        if (!q.empty()) {
	  msg = *q.dequeue();
	}
      }
      else if (!q.empty()) {
        msg = *q.dequeue();
	if (det_check) {
	  if (diff_y < -150 || diff_y > 150) {
            msg.x = 0;
	    msg.y += diff_y * y_scaling_factor;
	    msg.z = 380;
	    msg.r = 0;
	  }
	  else if (diff_y < -75 || diff_y > 75) {
            msg.x = 0;
	    msg.y = 0;
	    msg.z = 380;
	    msg.r += diff_y * y_scaling_factor;
	  }
	  else if (diff_z < -150 || diff_z > 150) {
            msg.x = 0;
	    msg.y = 0;
	    msg.z -= diff_z * z_scaling_factor;
	    msg.r = 0;
	  }
	  else {
            ++det_count;
	    msg.x = 350;
	    msg.y = 0;
	    msg.z = 380;
	    msg.r = 0;
	  }
          ////msg.r += diff_r * r_scaling_factor;
          //// might happen to often to use this simply
          ////msg.z -= diff_z * z_scaling_factor;
	  //msg.y += diff_y * y_scaling_factor;
	}
	//if (det_count > 12) {
	//  msg.x = 0;
	//  msg.z = 600;
	//  msg.r = -500;
	//}
	//if (det_count > 20) exit(1);
        //RCLCPP_INFO(this->get_logger(), "Publishing");
      }
      else {
	if (det_check) {
	  if (diff_y < -150 || diff_y > 150) {
            msg.x = 0;
	    msg.y += diff_y * y_scaling_factor;
	    msg.z = 380;
	    msg.r = 0;
	  }
	  else if (diff_y < -75 || diff_y > 75) {
            msg.x = 0;
	    msg.y = 0;
	    msg.z = 380;
	    msg.r -= diff_y * y_scaling_factor;
	  }
	  else if (diff_z < -100 || diff_z > 100) {
            msg.x = 0;
	    msg.y = 0;
	    msg.z -= diff_z * z_scaling_factor;
	    msg.r = 0;
	  }
	  else {
            ++det_count;
	    msg.x = 350;
	    msg.y = 0;
	    msg.z = 380;
	    msg.r = 0;
	  }
          ////msg.r += diff_r * r_scaling_factor;
          //// might happen to often to use this simply
          ////msg.z -= diff_z * z_scaling_factor;
	  //msg.y += diff_y * y_scaling_factor;
	}
      }
      if (det_count > 56 && det_count < 64) {
        ++det_count;
        msg.x = 0;
	msg.y = 0;
	msg.z = 650;
	msg.r = 0;
      }
      else if (det_count >= 64) exit(1);
      mc_pub->publish(msg);
      //msg.x = 0;
      msg.y = 0;
      msg.z = 380;
      msg.r = 0;
      diff_r = 0;
      diff_z = 0;
      diff_y = 0;
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
