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

// Global settings.
bool facingInwards = true;
float offsetFromTarget = 0.50;  // in meters

// Global variables for destination indexing.
uint8_t destinationNumber = 0;
float targetX;
float targetY;
float targetPhi;
float destX;
float destY;
float destPhi;
uint64_t timeReference = 0;

// Helper: Offsets the coordinates by a fixed offset along the object's orientation.
// Note: Here, phi is assumed to be in radians.
void offsetCoordinates(float offset, float x, float y, float phi, float &offsetX, float &offsetY) {
    offsetX = x + offset * std::cos(phi);
    offsetY = y + offset * std::sin(phi);
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
        if (dx < 0.01 && dy < 0.01 && dphi < 0.5) {  // thresholds (adjust as needed)
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

// Helper: Map a template index to a tag name.
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
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Use an AsyncSpinner for concurrent callbacks.
    ros::AsyncSpinner spinner(4);
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
    // For debugging, print the loaded coordinates.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        ROS_INFO("Object %zu: x=%.2f, y=%.2f, φ=%.2f deg", i, boxes.coords[i][0], boxes.coords[i][1],
                 RAD2DEG(boxes.coords[i][2]));  // assuming boxes.coords stores angles in radians?
    }
    
    // Initialize image processing.
    ImagePipeline imagePipeline(n);

    // Timer setup.
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t reportingInterval = 1; // seconds
    uint64_t lastReportTimestamp = 0;

    // Wait for a valid initial pose (set via RViz's 2D Pose Estimate).
    ros::Rate waitRate(10);
    while (ros::ok() && !robotPose.pose_received) {
        ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
        waitRate.sleep();
    }
    ros::Duration(2.0).sleep();  // Allow costmaps to stabilize.
    ROS_INFO("Initial pose: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);

    // Record starting position.
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi_deg = RAD2DEG(robotPose.phi);  // Convert to degrees for move_base.

    // Prepare vector for detection results.
    std::vector<std::string> finalResults(boxes.coords.size(), "Blank");
    std::vector<std::string> seenTags; // To track duplicates.

    // Execute strategy: iterate over each destination.
    while (ros::ok() && secondsElapsed <= 300 && destinationNumber < boxes.coords.size()) {
        // Update timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - startTime).count();

        // Get current destination from Boxes.
        destX = boxes.coords[destinationNumber][0];
        destY = boxes.coords[destinationNumber][1];
        // For myhal_scene.xml, the object's angle is in degrees. Convert it to radians.
        destPhi = DEG2RAD(boxes.coords[destinationNumber][2]);

        // Compute the target orientation.
        float targetPhi; 
        if (facingInwards) {
            targetPhi = destPhi - M_PI; // subtract 180° (in radians)
            while (targetPhi < -2 * M_PI) {
                targetPhi += 2 * M_PI;
            }
        } else {
            targetPhi = destPhi;
        }

        ROS_INFO("----------------------------------------------------");
        ROS_INFO("Moving to destination %u at x=%.2f, y=%.2f, φ=%.2f deg", destinationNumber, destX, destY, RAD2DEG(destPhi));

        // Compute adjusted goal position.
        offsetCoordinates(offsetFromTarget, destX, destY, destPhi, targetX, targetY);
        ROS_INFO("Adjusted goal: (%.2f, %.2f) with target φ=%.2f deg", targetX, targetY, RAD2DEG(targetPhi));

        // Wait for AMCL convergence.
        waitForConvergence(robotPose);

        // Use the Navigation module to move to the target.
        if (!Navigation::moveToGoal(targetX, targetY, RAD2DEG(targetPhi))) {
            ROS_INFO("moveToGoal error for destination %u.", destinationNumber);
            destinationNumber++;
            continue;
        } else {
            ROS_INFO("Destination %u reached!", destinationNumber);
            ros::Duration(5.0).sleep();  // Wait for stabilization and image capture.
            destinationNumber++;
        }

        // Capture and compare the image using SURF-based image processing.
        int tagID = imagePipeline.getTemplateID(boxes);
        ROS_INFO("Detected tag %d at destination %u", tagID, destinationNumber - 1);
        std::string tagName = getTagName(tagID);

        // Check for duplicates (if tagName is not "Blank").
        bool duplicate = false;
        for (const auto &existing : seenTags) {
            if (existing == tagName && tagName != "Blank") {
                duplicate = true;
                break;
            }
        }
        if (tagName != "Blank" && !duplicate) {
            seenTags.push_back(tagName);
        }
        finalResults[destinationNumber - 1] = tagName + (duplicate ? " -> duplicate" : "");
    }

    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f deg)...", startX, startY, startPhi_deg);
    waitForConvergence(robotPose);
    if (!Navigation::moveToGoal(startX, startY, startPhi_deg)) {
        ROS_WARN("Failed to return to starting position.");
    }
    
    // Write results to file.
    std::ofstream outfile("contest2_results.txt");
    if (!outfile.is_open()) {
        ROS_ERROR("Could not open output file for writing results.");
    } else {
        for (size_t i = 0; i < finalResults.size(); i++) {
            outfile << "Tag " << finalResults[i] << " : Coordinates ("
                    << boxes.coords[i][0] << ", " 
                    << boxes.coords[i][1] << ", " 
                    << boxes.coords[i][2] << ")\n";
        }
        outfile.close();
        ROS_INFO("Results written to contest2_results.txt");
    }
    
    ROS_INFO("------PROGRAM END------");
    return 0;
}
