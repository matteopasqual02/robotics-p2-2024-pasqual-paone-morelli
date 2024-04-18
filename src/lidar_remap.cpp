#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void fixCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");
    ros::NodeHandle nodeHandle;

    ros::Publisher publ = nodeHandle.advertise<nav_msgs::Odometry>("/pointcloud_remapped", 1);
    ros::Subscriber subscr = nodeHandle.subscribe("/os_cloud_node/points", 1, fixCallback);

    ros::Rate loop_rate(1);


    return 0;
}