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

#define DEG2RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD2DEG(rad) ((rad) * (180.0 / M_PI))

// Global movement variables.
bool facingInwards = true;
float offsetFromTarget = 0.40;  // Base offset (can be updated dynamically)
uint8_t destinationNumber = 0;
float targetX, targetY, targetPhi;
float destX, destY, destPhi;

// --- Dynamic Offset Computation ---
// For now, return the constant offsetFromTarget.
float getDynamicOffset() {
    return offsetFromTarget;
}

// Helper function to adjust the goal coordinates.
// Adds 180° to the object's yaw (in degrees) so the robot approaches from the correct side.
// origX, origY: object's coordinates (from XML)
// objPhi: object's orientation in degrees
// adjX, adjY: computed adjusted goal coordinates.
void adjustGoalCoordinates(float origX, float origY, float objPhi, float &adjX, float &adjY) {
    float dynamicOffset = getDynamicOffset();
    // Add 180 degrees to flip the orientation.
    float phiAdjustedDeg = objPhi + 180.0f;
    // Convert to radians.
    float phiRad = DEG2RAD(phiAdjustedDeg);
    // Compute adjusted goal so the robot stops "dynamicOffset" meters before the object.
    adjX = origX - dynamicOffset * std::cos(phiRad);
    adjY = origY - dynamicOffset * std::sin(phiRad);
}

// Wait for AMCL convergence by checking that the robot's pose remains nearly constant over time.
void waitForConvergence(RobotPose &robotPose) {
    ros::Rate rate(10);
    float prevX = robotPose.x, prevY = robotPose.y, prevPhi = robotPose.phi;
    ros::Time stableStart = ros::Time::now();
    while (ros::ok()) {
        rate.sleep();
        float dx = std::fabs(robotPose.x - prevX);
        float dy = std::fabs(robotPose.y - prevY);
        float dphi = std::fabs(robotPose.phi - prevPhi);
        if (dx < 0.01 && dy < 0.01 && dphi < 0.5) {  // thresholds: adjust as needed
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
    // Initialize ROS node.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    
    // Start an AsyncSpinner so that callbacks (AMCL, image, etc.) run concurrently.
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // Subscribe to AMCL pose updates.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // Wait for an initial pose estimate.
    ros::Rate waitRate(10);
    while (ros::ok() && !robotPose.pose_received) {
        ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
        waitRate.sleep();
    }
    // Extra delay to allow costmap and AMCL to converge.
    ros::Duration(2.0).sleep();
    ROS_INFO("Initial pose estimate: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
    
    // Load object coordinates and template images.
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cerr << "ERROR: could not load coordinates or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;
    
    // Initialize Navigation and ImagePipeline modules.
    Navigation nav;
    ImagePipeline imagePipeline(n);
    
    // Record the starting pose.
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi; // in radians
    
    // Prepare a vector to store detection results.
    std::vector<int> detectedTags(boxes.coords.size(), -1);
    
    // Iterate over each object coordinate.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        // Retrieve the object coordinate.
        float objX = boxes.coords[i][0];
        float objY = boxes.coords[i][1];
        // For myhal_scene.xml, φ is provided in degrees.
        float objPhi_deg = boxes.coords[i][2];
        // We'll use the object orientation in degrees for goal adjustment.
        
        destX = objX;
        destY = objY;
        destPhi = objPhi_deg;
        
        // Compute target orientation.
        float targetPhi_deg;
        if (facingInwards) {
            targetPhi_deg = objPhi_deg + 180.0f;  // flip by 180° (in degrees)
            while (targetPhi_deg > 360.0f)
                targetPhi_deg -= 360.0f;
        } else {
            targetPhi_deg = objPhi_deg;
        }
        
        ROS_INFO("Moving to destination %zu: Object coords (%.2f, %.2f, %.2f deg)", i, destX, destY, objPhi_deg);
        
        // Compute adjusted goal coordinates using dynamic offset.
        adjustGoalCoordinates(destX, destY, objPhi_deg, targetX, targetY);
        ROS_INFO("Adjusted goal: (%.2f, %.2f) with target φ=%.2f deg", targetX, targetY, targetPhi_deg);
        
        // Wait for AMCL convergence before sending each goal.
        waitForConvergence(robotPose);
        
        // Send the goal via the Navigation module.
        if (!Navigation::moveToGoal(targetX, targetY, targetPhi_deg)) {
            ROS_WARN("moveToGoal failed for object %zu. Skipping tag detection.", i);
            detectedTags[i] = -1;
            continue;
        }
        ros::Duration(0.5).sleep();  // Brief stabilization.
        
        // Use SURF-based image processing to detect the tag.
        int tagID = imagePipeline.getTemplateID(boxes);
        ROS_INFO("Detected tag %d for object %zu", tagID, i);
        detectedTags[i] = tagID;
    }
    
    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f deg)...", startX, startY, RAD2DEG(startPhi));
    waitForConvergence(robotPose);
    if (!Navigation::moveToGoal(startX, startY, RAD2DEG(startPhi))) {
        ROS_WARN("Failed to return to starting position.");
    }
    
    // Write detection results to a file.
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
    
    ROS_INFO("------PROGRAM END------");
    spinner.stop();
    return 0;
}
