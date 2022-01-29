#include <iostream>
#include <pthread.h>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


void openCameraStream(std::string path, std::string dirName, std::string title)
{
  cv::VideoCapture cap;
  cap.open(path, cv::CAP_GSTREAMER);
  if(!cap.isOpened()){std::cerr << "Unable to open the camera " << std::endl;}

  std::cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << " " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

  std::string windowName = title;
  std::string outputDir = dirName;

  bool frameStFg = false;
  while(!frameStFg)
  {
    cv::Mat frame;
    cap.read(frame);
    if(frame.empty()){std::cerr << "frame buffer empty " << std::endl;}
    else
    {
      frameStFg = true;
    }
  }

  cv::TickMeter timer;
  timer.start();
  int frameCount = 0;

  while(1)
  {
    cv::Mat frame;
    cap.read(frame);

    if(frame.empty()){std::cerr << "frame buffer empty " << std::endl;}
    frameCount++;

//    std::thread th(writeFrametoDisk, &frame, outputDir, frameCount, windowName);
//    th.join();
    //// a simple wayto exit the loop
    if(frameCount > 500)
    {
      break;
    }
  }
  timer.stop();
  std::cout << "Device id " << title << " Capture ran for " << timer.getTimeSec() << " seconds" << std::endl;
  std::cout << "Device id " << title << " Number of frames to be capture should be " << timer.getTimeSec() * 30 << std::endl;
  std::cout << "Device id " << title << " Number of frames captured " << frameCount << std::endl;
  cap.release();
}

int main(int argc, char * argv[])
{
  std::string outputDir1 = "left/";
  std::string outputDir2 = "right/";
  std::string cam0 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  std::string cam1 = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  std::string windowTitle0 = "Left";
  std::string windowTitle1 = "Right";

  std::thread camera1Thread(openCameraStream, cam0, outputDir1, windowTitle0);
  std::thread camera2Thread(openCameraStream, cam1, outputDir2, windowTitle1);
  camera1Thread.join();
  camera2Thread.join();
}

