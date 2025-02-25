#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotPose {
	public:
		float x;
		float y;
		float phi;
	public:
		RobotPose(float x, float y, float phi);
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};
