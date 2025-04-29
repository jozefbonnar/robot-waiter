#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose initial_pose;
bool got_initial_pose = false;

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!got_initial_pose) {
        initial_pose = msg->pose.pose;
        got_initial_pose = true;
        ROS_INFO("Initial pose saved.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_and_return");
    ros::NodeHandle nh;

    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, amclCallback);
    MoveBaseClient ac("move_base", true);

    while (!got_initial_pose && ros::ok()) {
        ros::spinOnce();
        ROS_INFO_THROTTLE(2, "Waiting for initial pose...");
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();

    // Send forward goal (relative to base_link)
    move_base_msgs::MoveBaseGoal forward_goal;
    forward_goal.target_pose.header.frame_id = "base_link";
    forward_goal.target_pose.header.stamp = ros::Time::now();
    forward_goal.target_pose.pose.position.x = 9.0;
    forward_goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending forward goal...");
    ac.sendGoal(forward_goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Reached forward goal.");
    else
        ROS_WARN("Failed to reach forward goal.");

    ros::Duration(2.0).sleep();

    // Return to initial pose (in map frame)
    move_base_msgs::MoveBaseGoal return_goal;
    return_goal.target_pose.header.frame_id = "map";  // or "odom" if that's more stable
    return_goal.target_pose.header.stamp = ros::Time::now();
    return_goal.target_pose.pose = initial_pose;

    ROS_INFO("Returning to initial pose...");
    ac.sendGoal(return_goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Returned to initial pose.");
    else
        ROS_WARN("Failed to return to initial pose.");

    return 0;
}
