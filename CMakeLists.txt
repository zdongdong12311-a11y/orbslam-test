cmake_minimum_required(VERSION 3.10)
project(d455_stereo_slam)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge message_filters)
find_package(OpenCV 4.2 REQUIRED)
find_package(Pangolin QUIET)
set(ORB_DIR $ENV{HOME}/ORB_SLAM3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${ORB_DIR}
  ${ORB_DIR}/include
  ${ORB_DIR}/include/CameraModels
  ${ORB_DIR}/Thirdparty/Sophus
  ${ORB_DIR}/Thirdparty/Pangolin/include
  /usr/include/eigen3)

add_executable(d455_stereo_slam src/d455_stereo_slam.cc)
target_link_libraries(d455_stereo_slam
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${ORB_DIR}/lib/libORB_SLAM3.so)
