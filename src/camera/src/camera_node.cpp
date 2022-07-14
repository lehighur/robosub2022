#include <cstdio>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
// don't think this is needed for just the reader
// only the writer
#include "opencv2/videoio.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;

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

    // Constants.
    const float INPUT_WIDTH = 640.0;
    const float INPUT_HEIGHT = 640.0;
    const float SCORE_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.45;
    const float CONFIDENCE_THRESHOLD = 0.45;

    void post_process(Mat &input_image, vector<Mat> &outputs, const vector<string> &class_name) {
      // Initialize vectors to hold respective outputs while unwrapping     detections.
      vector<int> class_ids;
      vector<float> confidences;
      // Resizing factor.
      //float x_factor = input_image.cols / INPUT_WIDTH;
      //float y_factor = input_image.rows / INPUT_HEIGHT;
      float *data = (float *)outputs[0].data;
      const int dimensions = 6;
      // 25200 for default size 640.
      const int rows = 25200;
      // Iterate through 25200 detections.
      for (int i = 0; i < rows; ++i) {
        float confidence = data[4];
        // Discard bad detections and continue.
        if (confidence >= CONFIDENCE_THRESHOLD) {
          printf("found yay\n");
          float * classes_scores = data + 5;
          // Create a 1x11 Mat and store class scores of 6 classes.
          Mat scores(1, class_name.size(), CV_32FC1, classes_scores);
          // Perform minMaxLoc and acquire the index of best class  score.
          Point class_id;
          double max_class_score;
          minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
          // Continue if the class score is above the threshold.
          cout << class_id.x << "\n";
          cout << confidence << "\n";
          if (max_class_score > SCORE_THRESHOLD) {
            // Store class ID and confidence in the pre-defined respective vectors.
            confidences.push_back(confidence);
            class_ids.push_back(class_id.x);
            // Center.
            float cx = data[0];
            float cy = data[1];
            // Box dimension.
            float w = data[2];
            float h = data[3];
          }
        }
        else {
          printf("not found\n");
        }
        // Jump to the next row.
        data += 11;
      }
    }

    vector<Mat> pre_process(Mat &frame, Net &net) {
      Mat blob;
      blobFromImage(frame, blob, 1./255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);

      net.setInput(blob);

      // Forward propagate.
      vector<Mat> outputs;
      net.forward(outputs, net.getUnconnectedOutLayersNames());

      return outputs;
    }

    int detect_frames(int frames, Net &net, vector<string> &class_list) {
      // use frame queue
      for (int i = 0; i < frames; ++i) {
        this->capture >> this->frame;
        if (this->frame.empty()) break; // maybe be safer here
        vector<Mat> detections;     // Process the image.
        detections = pre_process(this->frame, net);
        post_process(this->frame, detections, class_list);
      }
      return 0;
    }

    void detect() {
    }

    void detect_one(Mat &frame) {
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

vector<string> load_class_list() {
  vector<string> class_list;
  ifstream ifs("/home/lur/model/obj.names");
  string line;
  while (getline(ifs, line)) {
    class_list.push_back(line);
  }
  return class_list;
}

// check if loaded
void load_net(Net &net) {
  net = readNet("/home/lur/model/yolo-obj_final.weights",
      "/home/lur/model/yolo-obj.cfg");
  //net.setPreferableTarget(DNN_TARGET_CUDA);
  //net.setPreferableBackend(DNN_BACKEND_CUDA);
  //net.setPreferableBackend(DNN_BACKEND_OPENCV);
  //net.setPreferableTarget(DNN_TARGET_CPU);
}

int main(int argc, char **argv) {
  // hides unused parameter warnings from compiler 
  (void) argc;
  (void) argv;
  printf("hello world camera package\n");

  vector<string> class_list = load_class_list();
  Net net;
  load_net(net);

  Camera front(0);
  Camera bottom(4);

  front.detect_frames(90, net, class_list);
  //front.record_to_file("/home/lur/test.mp4");

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
