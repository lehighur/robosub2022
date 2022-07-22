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

class ModelObject {
  public:
    Net net;
    vector<string> class_list;

    ModelObject() {
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
    }
};

class Camera : public rclcpp::Node {
  public:
    ModelObject *model;
    rclcpp::Publisher<lur::Cam>::SharedPtr camera_pub;
    rclcpp::Subscription<lur::RString>::SharedPtr brain_sub;

    Camera(int id, ModelObject &m) : Node("Camera") {
      model = m;
      camera_pub = this->create_publisher<lur::Cam>("/camera", 10);
      brain_sub = this->create_subscription<lur::RString>("/brain", 10, std::bind(&Camera::brain_sub_callback, this, _1));
      VideoCapture c;
      Mat f;
      if (!c.open(id)) {
        printf("ERROR: Unable to open camera with id: %d\n", id);
      }
      capture = c;
      frame = f;
      timer = this->create_wall_timer(500ms, std::bind(&Camera::timer_callback, this));
    }

  // Initialize the parameters
  float confThreshold = 0.5; // Confidence threshold
  float nmsThreshold = 0.4;  // Non-maximum suppression threshold
  int inpWidth = 416;        // Width of network's input image
  int inpHeight = 416;       // Height of network's input image
    // Constants.
    const float INPUT_WIDTH = 640.0;
    const float INPUT_HEIGHT = 640.0;

    void post_process(Mat &input_image, vector<Mat> &outputs, const vector<string> &class_name) {
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
            int centerX = (int)(data[0] * frame.cols);
            int centerY = (int)(data[1] * frame.rows);
            int width = (int)(data[2] * frame.cols);
            int height = (int)(data[3] * frame.rows);
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
      for (size_t i = 0; i < indices.size(); ++i) {
          lur::Cam msg;
          msg.x = boxes[indices[i]].x + (boxes[indices[i]].width / 2);
          msg.y = boxes[indices[i]].y + (boxes[indices[i]].height / 2);
          msg.class_id = classIds[indices[i]];
          msg.confidence = confidences[indices[i]];
          this->camera_pub->publish(msg);
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
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time for a frame : %.2f ms", t);
        cout << label << "\n";
      }
      return 0;
    }

    void detect(Net &net, vector<string> &class_list) {
      while (1) {
        this->capture >> this->frame;
        if (this->frame.empty()) break; // maybe be safer here
        vector<Mat> detections;     // Process the image.
        detections = pre_process(this->frame, net);
        post_process(this->frame, detections, class_list);
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time for a frame : %.2f ms", t);
        cout << label << "\n";
      }
    }

    void detect_one(Mat &frame, Net &net, vector<string> class_list) {
      this->capture >> this->frame;
      if (this->frame.empty()) return; // maybe be safer here
      vector<Mat> detections;     // Process the image.
      detections = pre_process(this->frame, net);
      post_process(this->frame, detections, class_list);
      // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
      vector<double> layersTimes;
      double freq = getTickFrequency() / 1000;
      double t = net.getPerfProfile(layersTimes) / freq;
      string label = format("Inference time for a frame : %.2f ms", t);
      cout << label << "\n";
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
    void timer_callback() {
      printf("timer_callback\n");
    }
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

  ModelObject model;

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto front = std::make_shared<Camera(0, &model)>;
  //auto bottom = std::make_shared<Camera(0, &model)>;

  //front.detect_frames(900, net, class_list);
  //front.detect(net, class_list);
  //front.record_to_file("/home/lur/test.mp4");

  executor.add_node(front);
  //executor.add_node(bottom);
  executor.spin();

  return 0;
}
