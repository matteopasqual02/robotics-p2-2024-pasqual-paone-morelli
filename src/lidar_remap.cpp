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
        //we set a callback function that handles changes in tha dynamic parameters
        server.setCallback(boost::bind(&LidarRemap::reconfigCallback, this, _1, _2));
        //we subscribe to the topc /os_cloud_node/points and call the pointCloudCallback to handle the data we recieve from it
        sub = nodeHandle.subscribe("/os_cloud_node/points", 1, &LidarRemap::pointCloudCallback, this);
        //we define the publisher that will be called at the end of the callback function that remaps the frame_id
        pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);
    }

    void reconfigCallback(first_project::parametersConfig &config, uint32_t level) {
        //we set the variable frame_id to the dynamic parameter's current value
        frame_id = config.topic_choice;
        //we print the new value to check its new value
        ROS_INFO("Frame ID: %s", frame_id.c_str());

    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        //we reset the frame_id of the input message to the value stored in our variable frame_id. We then republish it through the publish function.
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


