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
        nh_private.getParam("child_frame",child_frame);
        nh_private.getParam("root_frame",root_frame);

        ROS_INFO("child_frame: %s", child_frame.c_str());
        ROS_INFO("root_frame: %s", root_frame.c_str());


        sub = nodeHandle.subscribe("input_odom",1,&tf_sub_pub::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){

        /**
         * We create the tf transformation by setting the translation and rotation using the input data. We then publish the tf transform.
         */
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        transform.setRotation(q);

        //ENABLE to debug timestamp
        //ros::Time current_time = ros::Time::now();
        //ROS_INFO("Publishing TF odom_to_base_link with timestamp: %f", current_time.toSec());

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

