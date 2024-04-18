#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

class tf_sub_pub{
public:
    tf_sub_pub(){
        sub = nodeHandle.subscribe("input_odom",1,&tf_sub_pub::callback, this);
    }
void callback(const nav_msgs::Odometry::ConstPtr& msg){
        std::string root_frame;
        std::string child_frame;
        nodeHandle.getParam("child_frame",child_frame); //mette il parametro child_frame in child_frame
        nodeHandle.getParam("root_frame",root_frame);
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        /*tf::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        transform.setRotation(q);*/
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));

}
private:
    ros::NodeHandle nodeHandle;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
    /*std::string root_frame;
    std::string child_frame;
    nodeHandle.getParam("child_frame",child_frame); //mette il parametro child_frame in child_frame
    nodeHandle.getParam("root_frame",root_frame);*/
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    tf_sub_pub my_tf_sub_pub;

    ros::spin();
    return 0;
}