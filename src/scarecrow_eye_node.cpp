#include <iostream>
#include <pthread.h>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include "IMURead/ICM20948.h"


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

  ros::init(argc, argv, "stereo_imu");
  ros::NodeHandle n;

  sensor_msgs::Imu imu_msg;

  ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("stereo_imu", 20);

  IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;

	imuInit(&enMotionSensorType);

	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
		printf("Motion sersor is ICM-20948\n" );
	}
	else
	{
		printf("Motion sersor NULL\n");
	}

  ros::Rate loop_rate(200);

	while(ros::ok())
	{
		imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
		printf("\r\n /-------------------------------------------------------------/ \r\n");
		printf("\r\n Angle Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",stAngles.fRoll, stAngles.fPitch, stAngles.fYaw);
		printf("\r\n Acceleration(g): X: %.3f     Y: %.3f     Z: %.3f \r\n",stAccelRawData.fX, stAccelRawData.fY, stAccelRawData.fZ);
		printf("\r\n Gyroscope(dps): X: %.3f     Y: %.3f     Z: %.3f \r\n",stGyroRawData.fX, stGyroRawData.fY, stGyroRawData.fZ);
		printf("\r\n Magnetic(uT): X: %.3f     Y: %.3f     Z: %.3f \r\n",stMagnRawData.fX, stMagnRawData.fY, stMagnRawData.fZ);
		// usleep(100*1000);

    imu_msg.header.frame_id = "/stereo_imu";
    imu_msg.header.stamp = ros::Time::now();

    tf2::Quaternion quat;
    quat.setRPY(stAngles.fRoll, stAngles.fPitch, stAngles.fYaw);

    imu_msg.orientation.x = quat[0];
    imu_msg.orientation.y = quat[1];
    imu_msg.orientation.z = quat[2];
    imu_msg.orientation.w = quat[3];

    imu_msg.linear_acceleration.x = stAccelRawData.fX;
    imu_msg.linear_acceleration.y = stAccelRawData.fY;
    imu_msg.linear_acceleration.z = stAccelRawData.fZ;

    imu_msg.angular_velocity.x = stGyroRawData.fX;
    imu_msg.angular_velocity.y = stGyroRawData.fY;
    imu_msg.angular_velocity.z = stGyroRawData.fZ;

    IMU_pub.publish(imu_msg);

    ros::spinOnce();
    loop_rate.sleep();
	}
	return 0;

}

