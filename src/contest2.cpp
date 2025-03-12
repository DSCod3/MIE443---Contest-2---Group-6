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

// Global settings
bool facingInwards = true;
float offsetFromTarget = 0.50;  // Offset from target (meters)

// Global variables for destination indexing
uint8_t destinationNumber = 0;
float targetX;
float targetY;
float targetPhi;
float destX;
float destY;
float destPhi;
uint64_t timeReference = 0;

// Helper function: Offsets the coordinates by a fixed offset along the object's orientation.
// (Original Chinese comment: "计算偏移后的坐标" -> "Calculate offset coordinates")
void offsetCoordinates(float offset, float x, float y, float phi, float &offsetX, float &offsetY) {
    offsetX = x + offset * std::cos(phi);
    offsetY = y + offset * std::sin(phi);
}

// Wait for AMCL convergence by ensuring that the robot’s pose remains nearly constant over a short time.
// (Chinese: "等待AMCL收敛" -> "Wait for AMCL convergence")
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
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Clear the results file (overwrite old file each run).
    std::ofstream clearFile("contest.txt", std::ios::trunc);
    clearFile.close();

    // Use an AsyncSpinner for concurrent callback processing.
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Create RobotPose object and subscribe to AMCL pose updates.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Load object coordinates and template images.
    Boxes boxes; 
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;
    for (int i = 0; i < boxes.coords.size(); ++i) {
        ROS_INFO("Object %d: x=%.2f, y=%.2f, φ=%.2f deg", i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // Initialize ImagePipeline.
    ImagePipeline imagePipeline(n);

    // Timer setup.
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t reportingInterval = 1;
    uint64_t lastReportTimestamp = 0;

    // Map to keep track of results per destination.
    std::vector<std::string> finalResults(boxes.coords.size(), "Blank");

    // Map to count occurrences of each template.
    std::map<int, int> templateCounts;

    // Wait for initial pose estimate.
    ros::Rate loopRate(10);
    while (ros::ok() && !robotPose.pose_received) {
        ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
        loopRate.sleep();
    }
    ros::Duration(2.0).sleep();
    ROS_INFO("Initial pose: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);

    // Record starting position.
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi_deg = RAD2DEG(robotPose.phi);

    // Execute strategy: iterate through each destination.
    while (ros::ok() && secondsElapsed <= 300 && destinationNumber < boxes.coords.size()) {
        // Update timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - startTime).count();

        // Get destination from Boxes.
        destX = boxes.coords[destinationNumber][0];
        destY = boxes.coords[destinationNumber][1];
        // For myhal_scene.xml, φ is in degrees; convert to radians.
        destPhi = DEG2RAD(boxes.coords[destinationNumber][2]);

        // Compute target orientation.
        float targetPhi;
        if (facingInwards) {
            targetPhi = destPhi - M_PI;  // subtract 180° (in radians)
            while (targetPhi < -2 * M_PI) {
                targetPhi += 2 * M_PI;
            }
        } else {
            targetPhi = destPhi;
        }

        ROS_INFO("------------------------------------------------");
        ROS_INFO("Moving to destination %d at x=%.2f, y=%.2f, φ=%.2f deg", destinationNumber, destX, destY, RAD2DEG(destPhi));

        // Compute adjusted goal using offsetCoordinates.
        offsetCoordinates(offsetFromTarget, destX, destY, destPhi, targetX, targetY);
        ROS_INFO("Adjusted goal: (%.2f, %.2f) with target φ=%.2f deg", targetX, targetY, RAD2DEG(targetPhi));

        // Wait for AMCL convergence.
        waitForConvergence(robotPose);

        // Send goal using Navigation module.
        if (!Navigation::moveToGoal(targetX, targetY, RAD2DEG(targetPhi))) {
            ROS_INFO("moveToGoal error for destination %d.", destinationNumber);
            destinationNumber++;
            continue;
        } else {
            ROS_INFO("Destination %d reached!", destinationNumber);
            ros::Duration(5.0).sleep();  // Pause for stabilization and image capture.
            destinationNumber++;
        }

        // Use SURF-based image processing to detect the tag.
        int templateID = imagePipeline.getTemplateID(boxes);
        if (templateID != -1) {
            templateCounts[templateID]++; // Update global count
            ROS_INFO("Detected template %d at destination %d", templateID, destinationNumber - 1);
        } else {
            ROS_INFO("No template detected at destination %d", destinationNumber - 1);
        }
        // Save result with mapping.
        std::string tagName = getTagName(templateID);
        // If this tag has been seen before (and is not Blank), mark as duplicate.
        bool duplicate = false;
        for (const auto &pair : templateCounts) {
            if (pair.first == templateID && pair.second > 1 && tagName != "Blank") {
                duplicate = true;
                break;
            }
        }
        finalResults[destinationNumber - 1] = tagName + (duplicate ? " -> duplicate" : "");

        // Append intermediate results to contest.txt (for debugging/logging).
        std::ofstream outFile("contest.txt", std::ios::app);
        if (outFile.is_open()) {
            outFile << "===== Timestamp: " << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << " =====\n";
            outFile << "Destination " << destinationNumber - 1 << ": " << finalResults[destinationNumber - 1] 
                    << " (Coordinates: " << destX << ", " << destY << ", " << boxes.coords[destinationNumber - 1][2] << ")\n\n";
            outFile.close();
        } else {
            ROS_ERROR("Failed to write to contest.txt");
        }
    }

    // Return to starting position.
    ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f deg)...", startX, startY, startPhi_deg);
    waitForConvergence(robotPose);
    if (!Navigation::moveToGoal(startX, startY, startPhi_deg)) {
        ROS_WARN("Failed to return to starting position.");
    }
    
    // Write final results to output file in required format.
    std::ofstream finalFile("contest.txt", std::ios::app);
    if (finalFile.is_open()) {
        finalFile << "===== FINAL RESULTS =====\n";
        for (size_t i = 0; i < finalResults.size(); i++) {
            finalFile << "Tag " << finalResults[i] << " : Coordinates ("
                      << boxes.coords[i][0] << ", " << boxes.coords[i][1] << ", " 
                      << boxes.coords[i][2] << ")\n";
        }
        finalFile.close();
        ROS_INFO("Final results written to contest.txt");
    } else {
        ROS_ERROR("Failed to write final results to contest.txt");
    }
    
    ROS_INFO("------PROGRAM END------");
    return 0;
}
