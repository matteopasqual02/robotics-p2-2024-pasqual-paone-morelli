#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "node1");

    ros::NodeHandle nodeH;
    ros::Subscriber subscr;

    subscr.subscribe();

    ros::spin();

    return 0;
}