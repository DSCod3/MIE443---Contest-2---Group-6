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

// Macros for conversion.
#define DEG2RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD2DEG(rad) ((rad) * (180.0 / M_PI))

// Global movement constants.
bool facingInwards = true;
float baseOffset = 0.50;  // Base offset from target (meters)

// Helper function: Computes adjusted goal coordinates based on object's coordinate,
// by adding 180° to the object's orientation (if needed) then subtracting an offset.
// Here, we use the object's original orientation (in degrees) to compute the offset.
void adjustGoalCoordinates(float origX, float origY, float objPhi, float &adjX, float &adjY) {
    // We use a fixed offset (baseOffset) for now.
    float phiRad = DEG2RAD(objPhi + 180.0f);
    adjX = origX + baseOffset * std::cos(phiRad);
    adjY = origY + baseOffset * std::sin(phiRad);
}

void offsetCoordinates(float offset, float x, float y, float phi, float &offsetX, float &offsetY){
    offsetX = x+offset*std::cos(phi);
    offsetY = y+offset*std::sin(phi);
}

// Wait for AMCL convergence: repeatedly check that the robot's pose is nearly constant.
void waitForConvergence(RobotPose &robotPose) {
    ros::Rate rate(10);
    float prevX = robotPose.x, prevY = robotPose.y, prevPhi = robotPose.phi;
    ros::Time stableStart = ros::Time::now();
    while (ros::ok()) {
        rate.sleep();
        float dx = std::fabs(robotPose.x - prevX);
        float dy = std::fabs(robotPose.y - prevY);
        float dphi = std::fabs(robotPose.phi - prevPhi);
        if (dx < 0.01 && dy < 0.01 && dphi < 0.5) {
            if ((ros::Time::now() - stableStart).toSec() > 2.0) {
                ROS_INFO("AMCL converged: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
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

// Mapping from template indices to tag names.
std::string getTagName(int templateID) {
    if (templateID == 0)
        return "Raisin Bran";
    else if (templateID == 1)
        return "Cinnamon Toast Crunch";
    else if (templateID == 2)
        return "Rice Krispies";
    else
        return "Blank";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    
    // Use AsyncSpinner for concurrent callbacks.
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
    ros::Duration(2.0).sleep(); // Allow costmaps to stabilize.
    ROS_INFO("Initial pose: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
    
    // Load object coordinates and template images.
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cerr << "ERROR: could not load coordinates or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;
    
    // Initialize Navigation and ImagePipeline modules.
    // Note: Navigation::moveToGoal expects orientation in degrees.
    ImagePipeline imagePipeline(n);
    
    // Record starting pose.
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;  // convert to degrees
    
    // Vector for storing final detection results (one per object).
    std::vector<std::string> finalResults(boxes.coords.size(), "Blank");
    
    // To detect duplicates, keep track of already seen tag names.
    std::vector<std::string> seenTags;

    float objX;
    float objY;
    float objPhi;

    float goalX, goalY;
    float baseTargetPhi;
    
    // For each object in the coordinate list.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        // Get the object's coordinates and orientation.
        objX = boxes.coords[i][0];
        objY = boxes.coords[i][1];
        //float objPhi = boxes.coords[i][2];
        objPhi = DEG2RAD(boxes.coords[i][2]);  // from myhal_scene.xml (in degrees)
        
        // Determine the base target orientation.
        if (facingInwards) {
            // If facing inwards, approach from the opposite side.
            baseTargetPhi = objPhi - M_PI;
            while (baseTargetPhi < 2*M_PI){
                baseTargetPhi += 2*M_PI;}
        } else {
            baseTargetPhi = objPhi;
        }
        
        ROS_INFO("Processing object %zu: (%.2f, %.2f, %.2f deg)", i, objX, objY, RAD2DEG(objPhi));
        
        // We'll take 3 images: one straight on, one with +30° offset, one with -30° offset.
        // We'll store the detection results.
        std::vector<int> detections;
        std::vector<float> angleOffsets = {DEG2RAD(0.0), DEG2RAD(30.0), DEG2RAD(-30.0)};
        for (size_t j = 0; j < 1; j++) {
            //float currentGoalPhi = objPhi + angleOffsets[j];
            // Normalize angle.
            //while (currentGoalPhi > M_PI) currentGoalPhi -= 2*M_PI;
            //while (currentGoalPhi < -M_PI) currentGoalPhi += 2*M_PI;
            
            // Compute an adjusted goal position using a helper that uses the object's original orientation.
            // Use the original object's orientation (objPhi_deg) for offset calculation.
            // (Assumes that adding 180° in adjustGoalCoordinates is done inside that function.)
            offsetCoordinates(baseOffset, objX, objY, objPhi, goalX, goalY);
            
            ROS_INFO("Moving to object %zu, view %zu at goal (%.2f, %.2f, %.2f deg)", 
                     i, j, goalX, goalY, RAD2DEG(baseTargetPhi));
            
            // Wait for convergence before sending the goal.
            waitForConvergence(robotPose);
            
            // Send the goal using the Navigation module.
            if (!Navigation::moveToGoal(goalX, goalY, baseTargetPhi)) {
                ROS_WARN("Failed to reach object %zu, view %zu. Skipping this view.", i, j);
                detections.push_back(-1);
                continue;
            }
            // Allow brief stabilization.
            ros::Duration(0.5).sleep();
            
            // Use the SURF-based image pipeline to get a template match.
            int tagID = imagePipeline.getTemplateID(boxes);
            ROS_INFO("Detected tag %d for object %zu, view %zu", tagID, i, j);
            detections.push_back(tagID);
        }
        
        // For this object, choose the "straight-on" detection (view index 0) if valid.
        int finalTagID = detections[0];
        if(finalTagID == -1) {
            // Otherwise, if one of the side views is valid, use it.
            if(detections[1] != -1)
                finalTagID = detections[1];
            else if(detections[2] != -1)
                finalTagID = detections[2];
        }
        
        // Map the finalTagID to a tag name.
        std::string tagName = getTagName(finalTagID);
        
        // If this tag was seen before, mark it as duplicate.
        bool duplicate = false;
        for (const auto &name : seenTags) {
            if (name == tagName && tagName != "Blank") {
                duplicate = true;
                break;
            }
        }
        if (tagName != "Blank" && !duplicate) {
            seenTags.push_back(tagName);
        }
        
        finalResults[i] = tagName + (duplicate ? " -> duplicate" : "");
    }
    
    // Return to starting position.
    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f deg)...", startX, startY, RAD2DEG(startPhi));
    waitForConvergence(robotPose);
    if (!Navigation::moveToGoal(startX, startY, startPhi)) {
        ROS_WARN("Failed to return to starting position.");
    }
    
    // Write the detection results to the output file in the required format.
    std::ofstream outfile("contest2_results.txt");
    if (!outfile.is_open()) {
        ROS_ERROR("Could not open output file for writing results.");
    } else {
        for (size_t i = 0; i < finalResults.size(); i++) {
            outfile << "Tag " << finalResults[i] << " : Coordinates " 
                    << "(" << boxes.coords[i][0] << ", " << boxes.coords[i][1] 
                    << ", " << boxes.coords[i][2] << ")\n";
        }
        outfile.close();
        ROS_INFO("Results written to contest2_results.txt");
    }
    
    ROS_INFO("------PROGRAM END------");
    spinner.stop();
    return 0;
}
