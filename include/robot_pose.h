#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotPose {
    public:
        float x;
        float y;
        float phi;
        bool pose_received; // New flag to indicate a valid pose has been received

    public:
        // Constructor: initializes pose values and sets pose_received to false since default is no pose.
        RobotPose(float x, float y, float phi)
            : x(x), y(y), phi(phi), pose_received(false) {}

        // Callback function to update the pose values and set the flag.
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};
