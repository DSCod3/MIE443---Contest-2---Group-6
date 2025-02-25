#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <cmath>

// Offset to position robot safely away from object
const float STOP_OFFSET = 0.4;  

// Helper to adjust goal coordinates so that the robot stops away from the object.
void adjustGoalCoordinates(float origX, float origY, float objPhi, float &adjX, float &adjY) {
    float phiRad = objPhi * M_PI / 180.0;
    adjX = origX - STOP_OFFSET * cos(phiRad);
    adjY = origY - STOP_OFFSET * sin(phiRad);
}

// Wait for AMCL convergence by checking that the pose remains nearly constant over a given time.
void waitForConvergence(RobotPose &robotPose) {
    ros::Rate rate(10);
    // Initialize with the current pose.
    float prevX = robotPose.x, prevY = robotPose.y, prevPhi = robotPose.phi;
    ros::Time stableStart = ros::Time::now();
    while (ros::ok()) {
        rate.sleep();
        // Compute changes since last check.
        float dx = fabs(robotPose.x - prevX);
        float dy = fabs(robotPose.y - prevY);
        float dphi = fabs(robotPose.phi - prevPhi);
        // If the change is below thresholds, check how long it's been stable.
        if (dx < 0.01 && dy < 0.01 && dphi < 0.5) { // adjust thresholds as needed
            if ((ros::Time::now() - stableStart).toSec() > 2.0) {
                ROS_INFO("AMCL has converged: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
                break;
            }
        } else {
            // Not yet stable: reset the timer and update previous values.
            stableStart = ros::Time::now();
            prevX = robotPose.x;
            prevY = robotPose.y;
            prevPhi = robotPose.phi;
            ROS_INFO("Waiting for AMCL convergence... Current changes: dx=%.4f, dy=%.4f, dphi=%.4f", dx, dy, dphi);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    
    // Use an AsyncSpinner to process callbacks concurrently.
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // Subscribe to AMCL pose updates.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // Wait until an initial pose is received.
    ros::Rate waitRate(10);
    while (ros::ok() && !robotPose.pose_received) {
        ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
        waitRate.sleep();
    }
    // Extra delay to allow costmaps to stabilize.
    ros::Duration(2.0).sleep();
    ROS_INFO("Initial pose estimate: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
    
    // Load object coordinates and template images.
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cerr << "ERROR: could not load coordinates or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;
    
    // Initialize Navigation and ImagePipeline.
    Navigation nav;
    ImagePipeline imagePipeline(n);
    
    // Record starting pose.
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;
    
    std::vector<int> detectedTags(boxes.coords.size(), -1);
    
    // Iterate over each object coordinate.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        float objX = boxes.coords[i][0];
        float objY = boxes.coords[i][1];
        float objPhi = boxes.coords[i][2]; // Object's orientation in degrees.
        
        // Adjust goal so robot stops at a safe distance and faces the tag.
        float goalX, goalY;
        adjustGoalCoordinates(objX, objY, objPhi, goalX, goalY);
        float goalPhi = objPhi;
        
        ROS_INFO("Preparing to navigate to object %zu at adjusted goal (%.2f, %.2f, %.2f)...", i, goalX, goalY, goalPhi);
        
        // Ensure AMCL has converged before sending each goal.
        waitForConvergence(robotPose);
        
        // Use move_base (via Navigation module) to go to the adjusted target.
        bool reached = nav.moveToGoal(goalX, goalY, goalPhi);
        if (!reached) {
            ROS_WARN("Failed to reach object %zu. Skipping tag detection.", i);
            detectedTags[i] = -1;
            continue;
        }
        ros::Duration(0.5).sleep();  // Brief stabilization pause.
        
        // Perform SURF-based image processing to detect the tag.
        int tagID = imagePipeline.getTemplateID(boxes);
        ROS_INFO("Detected tag %d for object %zu", tagID, i);
        detectedTags[i] = tagID;
    }
    
    // Return to starting position.
    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f)...", startX, startY, startPhi);
    waitForConvergence(robotPose);
    bool returned = nav.moveToGoal(startX, startY, startPhi);
    if (!returned) {
        ROS_WARN("Failed to return to starting position.");
    }
    
    // Write detection results to output file.
    std::ofstream outfile("contest2_results.txt");
    if (!outfile.is_open()) {
        ROS_ERROR("Could not open output file for writing results.");
    } else {
        outfile << "Contest 2 Object Detection Results\n";
        for (size_t i = 0; i < detectedTags.size(); i++) {
            outfile << "Object " << i << ": ";
            outfile << ((detectedTags[i] == -1) ? "No tag detected" : "Tag " + std::to_string(detectedTags[i])) << "\n";
        }
        outfile.close();
        ROS_INFO("Results written to contest2_results.txt");
    }
    
    spinner.stop();
    return 0;
}
