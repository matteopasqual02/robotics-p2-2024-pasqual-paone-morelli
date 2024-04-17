#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cstdlib>
#include <cmath>

double lat = 0;
double lon = 0;
double alt = 0;
double const a = 6378137;
double const b = 6356752;
double const e_square = 1- (b*b)/(a*a);

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
}
double Ntheta(){
    return a/( sqrt( 1-e_square * sin(lat) * sin(lat) ) );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nodeHandle;

    ros::Publisher publ = nodeHandle.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    ros::Subscriber subscr = nodeHandle.subscribe("/fix", 1, fixCallback);

    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        nav_msgs::Odometry private_message;

        double lat_lon_rad[2];
        lat_lon_rad[0] = M_PI * lat / 180;
        lat_lon_rad[1] = M_PI * lon / 180;

        double ECEF[3];
        ECEF[0] = ( Ntheta() + alt ) * cos(lat_lon_rad[0]) * cos(lat_lon_rad[1]);
        ECEF[1] = ( Ntheta() + alt ) * cos(lat_lon_rad[0]) * sin(lat_lon_rad[1]);
        ECEF[2] = ( Ntheta() * (1-e_square) + alt ) * cos(lat_lon_rad[0]) * sin(lat_lon_rad[1]);

        double ENU[3];

        private_message.pose.pose.position.x = ECEF[0];
        private_message.pose.pose.position.y = ECEF[1];
        private_message.pose.pose.position.z = ECEF[2];

        /*
        private_message.pose.pose.position.x = lat;
        private_message.pose.pose.position.y = lon;
        private_message.pose.pose.position.z = alt;
*/
        publ.publish(private_message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
