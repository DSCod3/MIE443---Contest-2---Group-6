#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/spinner.h>  // For AsyncSpinner
#include <cmath>          // For cos(), sin()

// Define an offset distance (in meters) so the robot stops at a safe distance from the object.
const float STOP_OFFSET = 0.3;  

// Helper: Adjust target coordinate based on object's orientation.
void adjustGoalCoordinates(float originalX, float originalY, float objectPhi,
                           float &adjustedX, float &adjustedY) {
    // Convert orientation to radians.
    float phiRad = objectPhi * M_PI / 180.0;
    // Offset the goal backwards along the object's orientation.
    adjustedX = originalX - STOP_OFFSET * cos(phiRad);
    adjustedY = originalY - STOP_OFFSET * sin(phiRad);
}

int main(int argc, char** argv) {
    // Initialize ROS node.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Use an AsyncSpinner for concurrent callback processing.
    ros::AsyncSpinner spinner(4); // Use 4 threads.
    spinner.start();

    // Create and subscribe to AMCL pose updates.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Load object coordinates and template images.
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coordinates or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;

    // Initialize Navigation and ImagePipeline modules.
    Navigation nav;
    ImagePipeline imagePipeline(n);

    // Allow some time for the AMCL pose to converge.
    ros::Duration(0.5).sleep();
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;

    // Prepare a vector to store the detection results for each object.
    std::vector<int> detectedTags(boxes.coords.size(), -1);

// Wait until an initial pose estimate is received
ros::Rate waitRate(10);
while(ros::ok() && !robotPose.pose_received) {
    ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
    waitRate.sleep();
}
ROS_INFO("Initial pose estimate received: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);

    // Iterate through each object coordinate.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        float objX = boxes.coords[i][0];
        float objY = boxes.coords[i][1];
        float objPhi = boxes.coords[i][2]; // object's orientation in degrees
        
        // Adjust goal coordinates to ensure a safe stop and correct view of tag.
        float goalX, goalY;
        adjustGoalCoordinates(objX, objY, objPhi, goalX, goalY);

        // Set the goal orientation to be the same as object's, so robot faces the tag.
        float goalPhi = objPhi;

        ROS_INFO("Navigating to object %zu at adjusted goal (%.2f, %.2f, %.2f)...", 
                 i, goalX, goalY, goalPhi);

        // Use move_base (via Navigation module) to go to the adjusted target.
        bool reached = nav.moveToGoal(goalX, goalY, goalPhi);
        if (!reached) {
            ROS_WARN("Failed to reach object %zu. Skipping tag detection.", i);
            detectedTags[i] = -1;
            continue;
        }
        // Allow a brief pause for stabilization.
        ros::Duration(0.5).sleep();

        // Use SURF-based image processing to detect the tag.
        int tagID = imagePipeline.getTemplateID(boxes);
        ROS_INFO("Detected tag %d for object %zu", tagID, i);
        detectedTags[i] = tagID;
    }

    // After processing all objects, return to the starting position.
    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f)...", startX, startY, startPhi);
    bool returned = nav.moveToGoal(startX, startY, startPhi);
    if (!returned) {
        ROS_WARN("Failed to return to starting position.");
    }

    // Write the detection results to an output file.
    std::ofstream outfile("contest2_results.txt");
    if (!outfile.is_open()) {
        ROS_ERROR("Could not open output file for writing results.");
    } else {
        outfile << "Contest 2 Object Detection Results\n";
        for (size_t i = 0; i < detectedTags.size(); i++) {
            outfile << "Object " << i << ": ";
            if (detectedTags[i] == -1)
                outfile << "No tag detected\n";
            else
                outfile << "Tag " << detectedTags[i] << "\n";
        }
        outfile.close();
        ROS_INFO("Results written to contest2_results.txt");
    }
    
    spinner.stop();
    return 0;
}
