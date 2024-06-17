#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x;
    double y;
    double yaw;
};

std::vector<Goal> readGoalsFromCSV(const std::string& file_path) {
    std::vector<Goal> goals;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open file: " << file_path);
        return NULL;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::getline(ss, token, ',');
        double x = std::stod(token);
        std::getline(ss, token, ',');
        double y = std::stod(token);
        std::getline(ss, token, ',');
        double yaw = std::stod(token);

        Goal goal;
        goal.x = x;
        goal.y = y;
        goal.yaw = yaw;
        goals.push_back(goal);
    }

    file.close();
    return goals;
}

void sendGoals(const std::vector<Goal>& goals) {
    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for (const auto& goal : goals) {
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map"; 
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = goal.x;
        goal_msg.target_pose.pose.position.y = goal.y;

        tf::Quaternion q;
        q.setRPY(0, 0, goal.yaw);
        tf::quaternionTFToMsg(q, goal_msg.target_pose.pose.orientation);

        ROS_INFO("Sending goal: (%.2f, %.2f, %.2f)", goal.x, goal.y, goal.yaw);
        ac.sendGoal(goal_msg);

        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal reached successfully.");
            } else {
                ROS_WARN("Goal failed: %s", state.toString().c_str());
            }
        } else {
            ROS_WARN("Action did not finish before the time out.");
        }
    }

    ROS_INFO("All goals have been sent.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_node");
    ros::NodeHandle nh("~");

    std::string file_path;
    nh.param<std::string>("file_path", file_path, "goals.csv"); // Default file path

    std::vector<Goal> goals = readGoalsFromCSV(file_path);
    if (goals.empty()) {
        ROS_ERROR("No goals to send. Exiting.");
        return 1;
    }

    sendGoals(goals);

    return 0;
}
