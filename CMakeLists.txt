cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_to_odom
  src/gps_to_odom.cpp
)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})



