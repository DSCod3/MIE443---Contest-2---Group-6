#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <laser.h>                // Include laser header for sensor data
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <cmath>

// Assume the laser module defines this structure and global variable:
extern DistancesStruct distances;

// Dynamic offset computation parameters.
const float MIN_OFFSET = 0.3;       // Minimum safe offset (meters)
const float MAX_OFFSET = 0.8;       // Maximum offset (meters)
const float SAFETY_MARGIN = 0.2;    // Margin to leave between the object and robot

// Dynamically compute an offset based on the robot's current front distance.
float getDynamicOffset() {
    float frontDistance = distances.frontRay;
    if (std::isnan(frontDistance) || frontDistance < (SAFETY_MARGIN + MIN_OFFSET)) {
        return MIN_OFFSET;
    }
    float computed = frontDistance - SAFETY_MARGIN;
    if (computed > MAX_OFFSET)
        computed = MAX_OFFSET;
    if (computed < MIN_OFFSET)
        computed = MIN_OFFSET;
    return computed;
}

// Adjust goal coordinates based on the object coordinate and dynamic offset.
void adjustGoalCoordinates(float origX, float origY, float objPhi, float &adjX, float &adjY) {
    float dynamicOffset = getDynamicOffset();
    float phiRad = objPhi * M_PI / 180.0;
    // Offset the goal along the object's orientation.
    adjX = origX - dynamicOffset * cos(phiRad);
    adjY = origY - dynamicOffset * sin(phiRad);
}

// Wait for AMCL convergence by ensuring the pose is stable.
void waitForConvergence(RobotPose &robotPose) {
    ros::Rate rate(10);
    float prevX = robotPose.x, prevY = robotPose.y, prevPhi = robotPose.phi;
    ros::Time stableStart = ros::Time::now();
    while (ros::ok()) {
        rate.sleep();
        float dx = fabs(robotPose.x - prevX);
        float dy = fabs(robotPose.y - prevY);
        float dphi = fabs(robotPose.phi - prevPhi);
        if (dx < 0.01 && dy < 0.01 && dphi < 0.5) {
            if ((ros::Time::now() - stableStart).toSec() > 2.0) {
                ROS_INFO("AMCL has converged: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
                break;
            }
        } else {
            stableStart = ros::Time::now();
            prevX = robotPose.x;
            prevY = robotPose.y;
            prevPhi = robotPose.phi;
            ROS_INFO("Waiting for AMCL convergence...");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // Subscribe to AMCL pose updates.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // Wait for a valid initial pose.
    ros::Rate waitRate(10);
    while (ros::ok() && !robotPose.pose_received) {
        ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
        waitRate.sleep();
    }
    // Extra delay for costmap stabilization.
    ros::Duration(2.0).sleep();
    ROS_INFO("Initial pose estimate: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
    
    // Load object coordinates and template images.
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cerr << "ERROR: could not load coordinates or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;
    
    Navigation nav;
    ImagePipeline imagePipeline(n);
    
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;
    
    std::vector<int> detectedTags(boxes.coords.size(), -1);
    
    // Iterate over each object coordinate.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        float objX = boxes.coords[i][0];
        float objY = boxes.coords[i][1];
        float objPhi = boxes.coords[i][2];  // Object orientation in degrees.
        
        // Dynamically compute the adjusted goal based on sensor data.
        float goalX, goalY;
        adjustGoalCoordinates(objX, objY, objPhi, goalX, goalY);
        float goalPhi = -objPhi;
        
        ROS_INFO("Navigating to object %zu at adjusted goal (%.2f, %.2f, %.2f)...", i, goalX, goalY, goalPhi);
        
        // Wait for AMCL to converge before sending each goal.
        waitForConvergence(robotPose);
        
        bool reached = nav.moveToGoal(goalX, goalY, goalPhi);
        if (!reached) {
            ROS_WARN("Failed to reach object %zu. Skipping tag detection.", i);
            detectedTags[i] = -1;
            continue;
        }
        ros::Duration(0.5).sleep();
        
        // Perform SURF-based image processing to detect the tag.
        int tagID = imagePipeline.getTemplateID(boxes);
        ROS_INFO("Detected tag %d for object %zu", tagID, i);
        detectedTags[i] = tagID;
    }
    
    // Return to the starting position.
    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f)...", startX, startY, startPhi);
    waitForConvergence(robotPose);
    bool returned = nav.moveToGoal(startX, startY, startPhi);
    if (!returned) {
        ROS_WARN("Failed to return to starting position.");
    }
    
    // Write detection results.
    std::ofstream outfile("contest2_results.txt");
    if (!outfile.is_open()) {
        ROS_ERROR("Could not open output file for writing results.");
    } else {
        outfile << "Contest 2 Object Detection Results\n";
        for (size_t i = 0; i < detectedTags.size(); i++) {
            outfile << "Object " << i << ": " 
                    << ((detectedTags[i] == -1) ? "No tag detected" : "Tag " + std::to_string(detectedTags[i])) << "\n";
        }
        outfile.close();
        ROS_INFO("Results written to contest2_results.txt");
    }
    
    spinner.stop();
    return 0;
}
