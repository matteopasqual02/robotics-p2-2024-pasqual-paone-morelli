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
    return a/( sqrt( 1-e_square * sin(lat*M_PI/180) * sin(lat*M_PI/180) ) );
}
double Ntheta_r(double lat_r){
    return a/( sqrt( 1-e_square * sin(lat_r*M_PI/180) * sin(lat_r*M_PI/180) ) );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nodeHandle;

    ros::Publisher publ = nodeHandle.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    ros::Subscriber subscr = nodeHandle.subscribe("/fix", 1, fixCallback);

    ros::Rate loop_rate(10);

    double lat_r,lon_r,alt_r;   //parametri INPUT
    double lat_r_lon_r_rad[2];  //transformazione in radianti (parametri)
    double lat_lon_rad[2];      //trasformazione in radianti
    double reference_ECEF[3];   //trasformazione parametri in ECEF
    double ECEF[3];             //ECEF
    double ENU[3];              //ENU

    nodeHandle.param("lat_r", lat_r, 0.0); 
    nodeHandle.param("lon_r", lon_r, 0.0);
    nodeHandle.param("alt_r", alt_r, 0.0);

    lat_r_lon_r_rad[0] = lat_r*M_PI/180;
    lat_r_lon_r_rad[1] = lon_r*M_PI/180;

    reference_ECEF[0] = ( Ntheta_r(lat_r) + alt_r ) * cos(lat_r_lon_r_rad[0]) * cos(lat_r_lon_r_rad[1]);
    reference_ECEF[1] = ( Ntheta_r(lat_r) + alt_r ) * cos(lat_r_lon_r_rad[0]) * sin(lat_r_lon_r_rad[1]);
    reference_ECEF[2] = ( Ntheta_r(lat_r) * (1-e_square) + alt_r ) * sin(lat_r_lon_r_rad[0]);

    ROS_INFO("lat_r: %f", reference_ECEF[0]);
    ROS_INFO("lon_r: %f", reference_ECEF[1]);
    ROS_INFO("alt_r: %f", reference_ECEF[2]);

    while(ros::ok()) {
        nav_msgs::Odometry private_message;

        lat_lon_rad[0] = M_PI * lat / 180;
        lat_lon_rad[1] = M_PI * lon / 180;

        ECEF[0] = ( Ntheta() + alt ) * cos(lat_lon_rad[0]) * cos(lat_lon_rad[1]);
        ECEF[1] = ( Ntheta() + alt ) * cos(lat_lon_rad[0]) * sin(lat_lon_rad[1]);
        ECEF[2] = ( Ntheta() * (1-e_square) + alt ) * sin(lat_lon_rad[0]);

        ENU[0] = (-sin(lat_r_lon_r_rad[1]))*(ECEF[0] - reference_ECEF[0]) 
            + (cos(lat_r_lon_r_rad[1]))*(ECEF[1] - reference_ECEF[1]) 
            + 0;
        ENU[1] = (-sin(lat_r_lon_r_rad[0])*cos(lat_r_lon_r_rad[1]))*(ECEF[0] - reference_ECEF[0]) 
            + (-sin(lat_r_lon_r_rad[0])*sin(lat_r_lon_r_rad[1]))*(ECEF[1] - reference_ECEF[1]) 
            + (cos(lat_r_lon_r_rad[0]))*(ECEF[2] - reference_ECEF[2]);
        ENU[2] = (cos(lat_r_lon_r_rad[0])*cos(lat_r_lon_r_rad[1]))*(ECEF[0] - reference_ECEF[0]) 
            + (cos(lat_r_lon_r_rad[0])*sin(lat_r_lon_r_rad[1]))*(ECEF[1] - reference_ECEF[1]) 
            + (sin(lat_r_lon_r_rad[0]))*(ECEF[2] - reference_ECEF[2]);

        private_message.pose.pose.position.x = ENU[0];
        private_message.pose.pose.position.y = ENU[1];
        private_message.pose.pose.position.z = ENU[2];

        publ.publish(private_message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
