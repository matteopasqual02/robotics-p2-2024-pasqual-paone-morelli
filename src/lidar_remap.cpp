#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class LidarRemap {
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher pub;
    ros::Subscriber sub;
    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    std::string frame_id;

public:
    LidarRemap() {
        server.setCallback(boost::bind(&LidarRemap::reconfigCallback, this, _1, _2));
        sub = nodeHandle.subscribe("/os_cloud_node/points", 1, &LidarRemap::pointCloudCallback, this);
        pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);
    }

    void reconfigCallback(first_project::parametersConfig &config, uint32_t level) {
        frame_id = config.topic_choice;
        ROS_INFO("Frame ID: %s", frame_id.c_str());

    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud2 modified_msg = *msg;
        modified_msg.header.frame_id = frame_id;
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


