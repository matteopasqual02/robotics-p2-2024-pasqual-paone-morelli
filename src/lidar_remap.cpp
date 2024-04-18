#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <parameter_test/parametersConfig.h>

class LidarRemap{
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher publ;
    ros::Subscriber subscr; 
    dynamic_reconfigure::Server<parameter_test::parametersConfig> server;
    std::string frame_id;     

public:
    LidarRemap() {
        subscr = nodeHandle.subscribe("/os_cloud_node/points", 1, &LidarRemap::pointCloudCallback, this);
        publ = nodeHandle.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped",1);

        server.setCallback(boost::bind(&LidarRemap::reconfigCallback, this, _1, _2));
    }

    void reconfigCallback(parameter_test::parametersConfig &config, uint32_t level) {
        frame_id = config.frame_id;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

        sensor_msgs::PointCloud2 modified_msg = *msg;
        modified_msg.header.frame_id = frame_id;

        publ.publish(modified_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");

    LidarRemap lidar_remap;
    ros::spin();

    return 0;
}
