#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

using namespace std;
using namespace cv;

typedef std_msgs::msg::Header RHeader;
typedef std_msgs::msg::String RString;
typedef sensor_msgs::msg::BatteryState RBatteryState;
typedef mavros_msgs::msg::State RState;
typedef mavros_msgs::msg::ManualControl RManualControl; 

class CameraNode : public rclcpp::Node {
  public:
    CameraNode() : Node("Camera") {
      printf("CameraNode constructor\n");
      auto callback = [this](RString::SharedPtr msg) {
      	this->brain_sub_callback(msg);
      };
      camera_pub = this->create_publisher<RString>("/camera", 10);
      brain_sub = this->create_subscription<RString>("/brain", 10, callback);
    }
  private:
    rclcpp::Publisher<RString>::SharedPtr camera_pub;
    rclcpp::Subscription<RString>::SharedPtr brain_sub;

    void brain_sub_callback(const RString::SharedPtr msg) {
      printf("brain_sub_callback\n");
      printf("String: %s\n", msg->data.c_str());
    }
};

class Camera {
  public:
    Camera(int id, int w=640, int h=640) {
      VideoCapture c;
      Mat f;
      if (!c.open(id)) {
        printf("ERROR: Unable to open camera with id: %d\n", id);
      }
      capture = c;
      frame = f;
      width = w;
      height = h;
    }
    void record_to_file(string path, int frames=900) {
      // Default resolutions of the frame are obtained.The default resolutions are system dependent.
      int frame_width = this->capture.get(cv::CAP_PROP_FRAME_WIDTH);
      int frame_height = this->capture.get(cv::CAP_PROP_FRAME_HEIGHT);
      VideoWriter writer(path, cv::VideoWriter::fourcc('M','J','P','G'), 30, Size(frame_width, frame_height));
      for (int i = 0; i < frames; ++i) {
        this->capture >> this->frame;
        if (this->frame.empty()) break; // maybe be safer here
        writer.write(frame); 
      }
    }
  private:
    VideoCapture capture;
    Mat frame;
    int width;
    int height;
};

int main(int argc, char **argv) {
  // hides unused parameter warnings from compiler 
  (void) argc;
  (void) argv;
  printf("hello world camera package\n");
  Camera front(0);
  Camera bottom(4);
  front.record_to_file("/home/lur/test.mp4");

  //for(int i = 0; i < 1; ++i)
  ////for(;;)
  //{
  //  cap >> frame;
  //  if( frame.empty() ) break; // end of video stream
  //  imshow("camera capture", frame);

  //  cap2 >> frame2;
  //  if( frame2.empty() ) break; // end of video stream
  //  imshow("camera capture2", frame2);
  //  if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
  //}

  // the camera will be closed automatically upon exit
  // cap.close();
  return 0;
}
