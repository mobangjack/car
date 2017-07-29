cmake_minimum_required(VERSION 2.8.3)
project(vel2odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(vel2odom src/vel2odom.cpp src/vel2odom_node.cpp)
target_link_libraries(vel2odom ${catkin_LIBRARIES})
