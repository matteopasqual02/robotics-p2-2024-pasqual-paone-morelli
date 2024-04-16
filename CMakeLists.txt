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

add_executable(node1
  src/node1.cpp
)


add_executable(node2
  src/node1.cpp
)


add_executable(node3
  src/node1.cpp
)

