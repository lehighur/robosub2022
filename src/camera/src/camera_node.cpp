#include <cstdio>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

auto timeFuncInvocation = 
    [](auto&& func, auto&&... params) {
        // get time before function invocation
        const auto& start = std::chrono::high_resolution_clock::now();
        // function invocation using perfect forwarding
        std::forward<decltype(func)>(func)(std::forward<decltype(params)>(params)...);
        // get time after function invocation
        const auto& stop = std::chrono::high_resolution_clock::now();
        return stop - start;
     };

int main(int argc, char **argv) {
  printf("hello world camera package\n");
  VideoCapture cap;
  // open the default camera, use something different from 0 otherwise;
  // Check VideoCapture documentation.
  if(!cap.open(0))
    return 0;

  //VideoCapture cap2;
  //if(!cap2.open(4))
  //  return 0;
  //int frame2_count = 0;
  //Mat frame2;

  int frame_count = 0;

  const auto& start = std::chrono::high_resolution_clock::now();
  Mat frame;
  for(int i = 0; i < 1; ++i)
  //for(;;)
  {
    cap >> frame;
    //cap2 >> frame2;
    if( frame.empty() ) break; // end of video stream
    //if( frame2.empty() ) break; // end of video stream
    ++frame_count;
    //++frame2_count;
    cout << frame.at<int>(0,0) << "\n";
    cout << "rows: " << frame.rows << "\n";
    cout << "cols: " << frame.cols << "\n";
    cout << "channels: " << frame.channels() << "\n";
    imshow("camera capture", frame);
    //imshow("camera capture2", frame2);
    if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
  }
  const auto& stop = std::chrono::high_resolution_clock::now();
  auto ms = chrono::duration_cast<chrono::milliseconds>(stop - start).count();
  cout << "Frames:  " << frame_count << "\n";
  cout << "Seconds: " << ms/1000 << "\n";
  cout << "FPS:     " << (double)frame_count / ((double)ms/1000) << "\n";
  //cout << "Frames2:  " << frame2_count << "\n";
  //cout << "FPS2:     " << (double)frame2_count / ((double)ms/1000) << "\n";
  // the camera will be closed automatically upon exit
  // cap.close();
  return 0;
}
