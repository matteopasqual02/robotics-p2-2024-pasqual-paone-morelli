#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

/*
 * NODE 2 first project
 */
class tf_sub_pub{
public:
    tf_sub_pub() {
        nh_private = ros::NodeHandle("~");
        nh_private.getParam("child_frame",child_frame);
        nh_private.getParam("root_frame",root_frame);

        ROS_INFO("child_frame: %s", child_frame.c_str());
        ROS_INFO("root_frame: %s", root_frame.c_str());


        sub = nodeHandle.subscribe("input_odom",1,&tf_sub_pub::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
    }

private:
    ros::NodeHandle nodeHandle;
    ros::NodeHandle nh_private;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
    std::string root_frame;
    std::string child_frame;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_base_link");

    tf_sub_pub my_tf_sub_pub;
    ros::spin();

    return 0;
}

