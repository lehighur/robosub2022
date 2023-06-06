#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
// don't think this is needed for just the reader
// only the writer
#include "opencv2/videoio.hpp"

#include "common.h"

using namespace std;
using namespace cv;
using namespace cv::dnn;
using std::placeholders::_1;

class Camera : public rclcpp::Node {
  public:
    Net net;
    vector<string> class_list;
    bool front_detected;
    bool bottom_detected;
    bool front_capturing;
    bool bottom_capturing;
    rclcpp::Publisher<lur::Cam>::SharedPtr camera_pub;
    rclcpp::Subscription<lur::RString>::SharedPtr brain_sub;

    Camera() : 
      Node("Camera"),
      count(0),
      front_detected(false),
      bottom_detected(false),
      front_capturing(false),
      bottom_capturing(false) {

      this->declare_parameter("timeout");
      rclcpp::Parameter p = this->get_parameter("timeout");
      this->timeout = p.as_int();

      this->declare_parameter("front_camera");
      this->declare_parameter("bottom_camera");
      rclcpp::Parameter front_camera = this->get_parameter("front_camera");
      rclcpp::Parameter bottom_camera = this->get_parameter("bottom_camera");
      int front_camera_id = front_camera.as_int();
      int bottom_camera_id = bottom_camera.as_int();

      // Load class list
      ifstream ifs("/home/lur/model/obj.names");
      string line;
      while (getline(ifs, line)) {
        class_list.push_back(line);
      }
      // Load model
      net = readNet("/home/lur/model/yolo-obj_final.weights",
          "/home/lur/model/yolo-obj.cfg");
      net.setPreferableTarget(DNN_TARGET_CUDA);
      net.setPreferableBackend(DNN_BACKEND_CUDA);
      //net.setPreferableBackend(DNN_BACKEND_OPENCV);
      //net.setPreferableTarget(DNN_TARGET_CPU);
      camera_pub = this->create_publisher<lur::Cam>("/lur/camera", 10);
      //brain_sub = this->create_subscription<lur::RString>("/brain", 10, std::bind(&Camera::brain_sub_callback, this, _1));
      VideoCapture fc;
      VideoCapture bc;
      Mat ff;
      Mat bf;
      if (!fc.open(front_camera_id)) {
        printf("ERROR: Unable to open front camera with id: %d\n", front_camera_id);
      }
      if (!bc.open(bottom_camera_id)) {
        printf("ERROR: Unable to open bottom camera with id: %d\n", bottom_camera_id);
      }
      front_capture = fc;
      bottom_capture = bc;
      front_frame = ff;
      bottom_frame = bf;
      timer = this->create_wall_timer(500ms, std::bind(&Camera::timer_callback, this));
    }

    // move
    //
  // Initialize the parameters
  float confThreshold = 0.5; // Confidence threshold
  float nmsThreshold = 0.4;  // Non-maximum suppression threshold
  int inpWidth = 416;        // Width of network's input image
  int inpHeight = 416;       // Height of network's input image
    // Constants.
    const float INPUT_WIDTH = 640.0;
    const float INPUT_HEIGHT = 640.0;

    void post_process(Mat &input_image, vector<Mat> &outputs, const vector<string> &class_name, bool front) {
      vector<int> classIds;
      vector<float> confidences;
      vector<Rect> boxes;
      
      for (size_t i = 0; i < outputs.size(); ++i) {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outputs[i].data;
        for (int j = 0; j < outputs[i].rows; ++j, data += outputs[i].cols) {
          Mat scores = outputs[i].row(j).colRange(5, outputs[i].cols);
          Point classIdPoint;
          double confidence;
          // Get the value and location of the maximum score
          minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
          if (confidence > confThreshold) {
            // MAKE SURE THESE ARE 640x800
            int centerX = (int)(data[0] * 800);
            int centerY = (int)(data[1] * 640);
            int width = (int)(data[2] * 800);
            int height = (int)(data[3] * 640);
            int left = centerX - width / 2;
            int top = centerY - height / 2;
            
            classIds.push_back(classIdPoint.x);
            confidences.push_back((float)confidence);
            boxes.push_back(Rect(left, top, width, height));
          }
        }
      }
    
      // Perform non maximum suppression to eliminate redundant overlapping boxes with
      // lower confidences
      vector<int> indices;
      NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
      if (front) this->front_detected = true;
      if (!front) this->bottom_detected = true;
      for (size_t i = 0; i < indices.size(); ++i) {
        lur::Cam msg;
        msg.x = boxes[indices[i]].x + (boxes[indices[i]].width / 2);
        msg.y = boxes[indices[i]].y + (boxes[indices[i]].height / 2);
        msg.width = boxes[indices[i]].width;
        msg.height = boxes[indices[i]].height;
        msg.class_id = classIds[indices[i]];
        RCLCPP_INFO(this->get_logger(), "class: %d\nx: %d, y: %d\n", msg.class_id, msg.x, msg.y);
        msg.confidence = confidences[indices[i]];
        this->camera_pub->publish(msg);
      }
    }

    vector<Mat> pre_process(Mat &frame) {
      Mat blob;
      blobFromImage(frame, blob, 1./255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);

      // do we need to worry about locking this?
      this->net.setInput(blob);

      // Forward propagate.
      vector<Mat> outputs;
      this->net.forward(outputs, this->net.getUnconnectedOutLayersNames());

      return outputs;
    }

    void detect_front() {
      this->front_capturing = true;
      while (!this->front_detected) {
        this->front_capture >> this->front_frame;
        if (this->front_frame.empty()) break; // maybe be safer here
        vector<Mat> detections;     // Process the image.
        detections = pre_process(this->front_frame);
        post_process(this->front_frame, detections, this->class_list, true);
        // print time for a frame
        //perf();
      }
      this->front_capturing = false;
    }

    void detect_bottom() {
      this->bottom_capturing = true;
      while (!this->bottom_detected) {
        this->bottom_capture >> this->bottom_frame;
        if (this->bottom_frame.empty()) break; // maybe be safer here
        vector<Mat> detections;     // Process the image.
        detections = pre_process(this->bottom_frame);
        post_process(this->bottom_frame, detections, this->class_list, false);
        // print time for a frame
        //perf();
      }
      this->bottom_capturing = false;
    }

    void front_record_to_file(string path, int frames=900) {
      // Default resolutions of the frame are obtained.The default resolutions are system dependent.
      int frame_width = this->front_capture.get(cv::CAP_PROP_FRAME_WIDTH);
      int frame_height = this->front_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
      VideoWriter writer(path, cv::VideoWriter::fourcc('M','J','P','G'), 30, Size(frame_width, frame_height));
      for (int i = 0; i < frames; ++i) {
        this->front_capture >> this->front_frame;
        if (this->front_frame.empty()) break; // maybe be safer here
        writer.write(front_frame); 
      }
    }

    void bottom_record_to_file(string path, int frames=900) {
      // Default resolutions of the frame are obtained.The default resolutions are system dependent.
      int frame_width = this->bottom_capture.get(cv::CAP_PROP_FRAME_WIDTH);
      int frame_height = this->bottom_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
      VideoWriter writer(path, cv::VideoWriter::fourcc('M','J','P','G'), 30, Size(frame_width, frame_height));
      for (int i = 0; i < frames; ++i) {
        this->bottom_capture >> this->bottom_frame;
        if (this->bottom_frame.empty()) break; // maybe be safer here
        writer.write(bottom_frame); 
      }
    }

  private:
    VideoCapture front_capture;
    VideoCapture bottom_capture;
    Mat front_frame;
    Mat bottom_frame;
    int front_width;
    int front_height;
    int bottom_width;
    int bottom_height;
    int timeout;
    rclcpp::TimerBase::SharedPtr timer;
    std::size_t count;

    void timer_callback() {
      ++this->count;
      // use message headers?
      if (this->count > this->timeout * 2) exit(1);

      // make this better
      // might just want to read in frames every 500ms
      if (!this->front_capturing) {
        if (!this->front_detected) detect_front();
        this->front_detected = false;
      }
      if (!this->bottom_capturing) {
        if (!this->bottom_detected) detect_bottom();
        this->bottom_detected = false;
      }
    }

    void brain_sub_callback() {
      printf("brain_sub_callback\n");
    }
};

int main(int argc, char **argv) {
  // hides unused parameter warnings from compiler 
  (void) argc;
  (void) argv;
  printf("hello world camera package\n");

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  // uses a bunch of memory, could be redesigned

  auto camera_node = std::make_shared<Camera>();
  //auto bottom = std::make_shared<Camera(0, model)>;

  //front.detect_frames(900, net, class_list);
  //front.detect(net, class_list);
  //front.record_to_file("/home/lur/test.mp4");

  executor.add_node(camera_node);
  executor.spin();

  return 0;
}
