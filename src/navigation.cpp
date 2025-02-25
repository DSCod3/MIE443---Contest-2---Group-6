#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){
    // Set up and wait for the action client.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // Convert phi from degrees to radians.
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(-phiGoal * M_PI / 180.0);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = xGoal;
    goal.target_pose.pose.position.y = yGoal;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = phi;
    ROS_INFO("Sending goal: (%.2f, %.2f, %.2f)", xGoal, yGoal, phiGoal);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Goal reached successfully.");
        return true;
    } else {
        ROS_INFO("Failed to reach goal: %s", ac.getState().toString().c_str());
        return false;
    }
}
