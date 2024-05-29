#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class LidarRemap {
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher pub;
    ros::Subscriber sub;

public:
    LidarRemap() {
        sub = nodeHandle.subscribe("/scan", 1, &LidarRemap::pointCloudCallback, this);
        pub = nodeHandle.advertise<sensor_msgs::LaserScan>("/scan_tf", 1);
    }

    void pointCloudCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan modified_msg = *msg;
        modified_msg.header.frame_id = "wheel_odom";
        modified_msg.header.stamp = ros::Time::now();
   
        pub.publish(modified_msg);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");

    LidarRemap lidar_remap;
    ros::spin();

    return 0;
    
}



