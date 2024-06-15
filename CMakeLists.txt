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
        move_base
        )


catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_remap src/lidar_remap.cpp src/lidar_remap.cpp)
add_dependencies(lidar_remap  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_remap  ${catkin_LIBRARIES})

add_executable(odom_to_base_link src/odom_to_base_link.cpp src/odom_to_base_link.cpp)
add_dependencies(odom_to_base_link  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_to_base_link  ${catkin_LIBRARIES})

install(FILES cfg/config.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


