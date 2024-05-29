cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        nav_msgs
        std_msgs
        roscpp
        tf
        pointcloud_to_laserscan
        rviz
        )


catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_tf 
  src/odom_to_tf.cpp
)
add_dependencies(odom_to_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

install(FILES cfg/rviz_config_file.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


