#include <robot_pose.h>
#include <tf/transform_datatypes.h>
#include <cmath>  // For M_PI

#define DEG2RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD2DEG(rad) ((rad) * (180.0 / M_PI))

RobotPose::RobotPose(float x, float y, float phi) {
	this->x = x;
	this->y = y;
	this->phi = phi;
	this->phid = RAD2DEG(phi);
}

void RobotPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
	phi = tf::getYaw(msg.pose.pose.orientation);
	phid = RAD2DEG(phi);
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
}
