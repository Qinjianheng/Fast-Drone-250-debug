cmake_minimum_required(VERSION 3.0.2)
project(rtk_converter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rtk_converter src/rtk_converter.cpp)
target_link_libraries(rtk_converter ${catkin_LIBRARIES})
