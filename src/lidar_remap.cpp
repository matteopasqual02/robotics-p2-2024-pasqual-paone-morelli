#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class LidarRemap {
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher pub;
    ros::Subscriber sub;
    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    std::string frame_id;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform stamped;

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
        
        listener.lookupTransform(frame_id, msg->header.frame_id, ros::Time(0), stamped);

        broadcaster.sendTransform(tf::StampedTransform(stamped, ros::Time::now(), "/pointcloud_remapped", msg->header.frame_id));
        
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");

    LidarRemap lidar_remap;
    ros::spin();

    return 0;
}


