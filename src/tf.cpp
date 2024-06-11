#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

class tf_sub_pub{
public:
    tf_sub_pub() {

        /**
         * we use the private nodeHandle to get the private parameters set in the launchfile,
         * then we print the values from the parameters to see if they are correct.
         * By subscribing to input_odom, remapped in the launchfile, we get the input data and call the callback function, which publishes the tf.
         */
        nh_private = ros::NodeHandle("~");

        sub = nodeHandle.subscribe("input_odom",1,&tf_sub_pub::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){

        /**
         * We create the tf transformation by setting the translation and rotation using the input data. We then publish the tf transform.
         */
         //first transformation: map to odom
        tf::Transform transform1;
        transform1.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tf::Quaternion q1(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        transform1.setRotation(q1);
        ros::Time current_time = ros::Time::now();
        ROS_INFO("Publishing TF odom_to_base_link with timestamp: %f", current_time.toSec());
        br.sendTransform(tf::StampedTransform(transform1, current_time, "map", "/odom"));

        //delay for the other transformation
        ros::Duration(0.001).sleep();

        //second transformation: odom to base_link
        tf::Transform transform2;
        transform2.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tf::Quaternion q2(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        transform2.setRotation(q2);
        current_time = ros::Time::now();
        ROS_INFO("Publishing TF odom_to_base_link with timestamp: %f", current_time.toSec());
        br.sendTransform(tf::StampedTransform(transform2, current_time, "/odom", "/base_link"));

        ros::Duration(0.001).sleep();

    }

private:
    ros::NodeHandle nodeHandle;
    ros::NodeHandle nh_private;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "tf");

    tf_sub_pub my_tf_sub_pub;
    ros::spin();

    return 0;
}

