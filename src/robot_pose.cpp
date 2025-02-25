#include <robot_pose.h>
#include <tf/transform_datatypes.h>

RobotPose::RobotPose(float x, float y, float phi) {
	this->x = x;
	this->y = y;
	this->phi = phi;
}

void RobotPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
	phi = tf::getYaw(msg.pose.pose.orientation);
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
}
