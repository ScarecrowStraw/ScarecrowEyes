cmake_minimum_required(VERSION 3.0.2)
project(scarecrow_eye)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package( OpenCV REQUIRED)
find_package( Boost REQUIRED )

catkin_package()


include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_subdirectory(src/CameraDisplay)
add_subdirectory(src/CameraSync)
# add_subdirectory(src/IMURead)
add_subdirectory(src/StereoCamerasCalibration)

catkin_install_python(PROGRAMS 
      scripts/camerasync.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${PROJECT_NAME}_node src/scarecrow_eye_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
