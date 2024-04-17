#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

double lat = 0;
double lon = 0;
double alt = 0;

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nodeHandle;

    ros::Publisher publ = nodeHandle.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    ros::Subscriber subscr = nodeHandle.subscribe("/fix", 1, fixCallback);

    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        nav_msgs::Odometry private_message;
        private_message.pose.pose.position.x = lat;
        private_message.pose.pose.position.y = lon;
        private_message.pose.pose.position.z = alt;

        publ.publish(private_message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
