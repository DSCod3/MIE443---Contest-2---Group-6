#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    // Initialize ROS node.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Robot pose object and subscriber.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Load object coordinates and image templates.
    Boxes boxes; 
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    std::cout << "Loaded " << boxes.coords.size() << " object coordinates." << std::endl;

    // Initialize Navigation and ImagePipeline.
    Navigation nav;
    ImagePipeline imagePipeline(n);

    // Give time for the robot pose to update.
    ros::Duration(1.0).sleep();
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;

    // Vector to store detection results for each object.
    std::vector<int> tagResults(boxes.coords.size(), -1);

    // Main algorithm: navigate to each object and perform tag detection.
    for (size_t i = 0; i < boxes.coords.size(); i++) {
        float targetX = boxes.coords[i][0];
        float targetY = boxes.coords[i][1];
        float targetPhi = boxes.coords[i][2];
        std::cout << "Navigating to object " << i 
                  << " at (" << targetX << ", " << targetY << ", " << targetPhi << ")" << std::endl;

        // Use the Navigation module to send a goal to move_base.
        bool reached = nav.moveToGoal(targetX, targetY, targetPhi);
        if (!reached) {
            std::cout << "Failed to reach object " << i << ". Skipping image detection." << std::endl;
            tagResults[i] = -1;
            continue;
        }
        // Allow the robot to settle.
        ros::Duration(1.0).sleep();

        // Perform image processing to detect the feature tag.
        int detectedTag = imagePipeline.getTemplateID(boxes);
        std::cout << "Detected tag " << detectedTag << " for object " << i << std::endl;
        tagResults[i] = detectedTag;
    }

    // After visiting all objects, return to the starting position.
    std::cout << "Returning to starting position (" << startX << ", " << startY << ", " << startPhi << ")" << std::endl;
    bool returned = nav.moveToGoal(startX, startY, startPhi);
    if (!returned) {
        std::cout << "Failed to return to starting position." << std::endl;
    }

    // Output the detection results to a file.
    std::ofstream outfile("contest2_results.txt");
    if (!outfile.is_open()) {
        std::cout << "ERROR: Unable to open output file." << std::endl;
    } else {
        outfile << "Contest 2 Object Detection Results\n";
        for (size_t i = 0; i < tagResults.size(); i++) {
            outfile << "Object " << i << ": ";
            if (tagResults[i] == -1)
                outfile << "No tag detected\n";
            else
                outfile << "Tag " << tagResults[i] << "\n";
        }
        outfile.close();
        std::cout << "Results written to contest2_results.txt" << std::endl;
    }
    
    return 0;
}
