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
      brain_sub = this->create_subscription<RString>("/brain", 10, std::bind(&MinimalSubscriber::brain_sub_callback, this, _1));
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

  // Initialize the parameters
  float confThreshold = 0.5; // Confidence threshold
  float nmsThreshold = 0.4;  // Non-maximum suppression threshold
  int inpWidth = 416;        // Width of network's input image
  int inpHeight = 416;       // Height of network's input image
    // Constants.
    const float INPUT_WIDTH = 640.0;
    const float INPUT_HEIGHT = 640.0;
    const float SCORE_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.45;
    const float CONFIDENCE_THRESHOLD = 0.45;

    void post_process(Mat &input_image, vector<Mat> &outputs, const vector<string> &class_name) {
      vector<int> classIds;
      vector<float> confidences;
      vector<Rect> boxes;
      
      for (size_t i = 0; i < outputs.size(); ++i) {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outputs[i].data;
        for (int j = 0; j < outputs[i].rows; ++j, data += outputs[i].cols)
        {
            Mat scores = outputs[i].row(j).colRange(5, outputs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                printf("found %d with confidence %.2f\n", classIdPoint.x, confidence);
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
        int idx = indices[i];
        Rect box = boxes[idx];
        //drawPred(classIds[idx], confidences[idx], box.x, box.y,
        //         box.x + box.width, box.y + box.height, frame);
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

  front.detect_frames(900, net, class_list);
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
