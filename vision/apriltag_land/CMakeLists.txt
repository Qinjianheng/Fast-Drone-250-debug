cmake_minimum_required(VERSION 3.0.2)
project(apriltag_land)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    nav_msgs
    sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge nav_msgs sensor_msgs
)

include_directories(apriltag)
aux_source_directory(apriltag/common COMMON_SRC)
set(APRILTAG_SRCS apriltag/apriltag.c apriltag/apriltag_pose.c apriltag/apriltag_quad_thresh.c)
file(GLOB TAG_FILES apriltag/tag*.c)
add_library(apriltag_detector ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})
target_link_libraries(apriltag_detector PUBLIC Threads::Threads m)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${apriltag_INCLUDE_DIRS}
)
add_executable(apriltag_land src/apriltag_land.cpp)
target_link_libraries(apriltag_land ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} apriltag_detector)
