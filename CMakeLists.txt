cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  nav_msgs
  dynamic_reconfigure
  roscpp
  tf
        )

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_to_odom
  src/gps_to_odom.cpp
)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(lidar_remap
  src/lidar_remap.cpp
)
add_dependencies(lidar_remap ${PROJECT_NAME}_gencfg)
target_link_libraries(lidar_remap ${catkin_LIBRARIES})

add_executable(odom_to_tf 
  src/odom_to_tf.cpp
)
add_dependencies(odom_to_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

