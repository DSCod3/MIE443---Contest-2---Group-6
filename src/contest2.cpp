// REMEMBER TO CHANGE THE BOXES COORDINATE FILE AND THE PHI CALCULATION (DEG/RAD) BEFORE RUNNING
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <fstream>
#include <map>
#include <algorithm>

#define DEG2RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD2DEG(rad) ((rad) * (180.0 / M_PI))

bool facingInwards = true;
float offsetFromTarget = 0.50;

// Initialize box coordinates and templates
uint8_t destinationIndex = 1;
int destinationNumber;
float targetX;
float targetY;
float targetPhi;
float destX;
float destY;
float destPhi;
uint64_t timeReference;

// waitForConvergence: Wait until the robot's AMCL pose remains nearly constant over a short period.
void waitForConvergence(RobotPose &robotPose) {
    ros::Rate loopRate(10);
    float prevX = robotPose.x, prevY = robotPose.y, prevPhi = robotPose.phi;
    ros::Time stableStart = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
        float dx = std::fabs(robotPose.x - prevX);
        float dy = std::fabs(robotPose.y - prevY);
        float dphi = std::fabs(robotPose.phi - prevPhi);
        if (dx < 0.01 && dy < 0.01 && dphi < 0.5) { // Thresholds; adjust as needed
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

void addNextDestinations(std::vector<std::vector<int>> currentPaths, int nDestinations, std::vector<std::vector<int>> &outputPaths){
    std::vector<std::vector<int>> newPaths = {};
    std::vector<int> thisPath;
    std::vector<int> thisPathCopy;

    // For each current vector in the list
    for(int i = 0; i < currentPaths.size(); i++){
        thisPath = currentPaths[i];
        // See if any of the destination numbers are not in that list, if not then append to a copy of that list and add that list to newPaths
        for(int j = 0; j < nDestinations; j++){
            thisPathCopy = std::vector<int>(thisPath);
            if(std::find(thisPath.begin(), thisPath.end(), j) == thisPath.end()){
                thisPathCopy.push_back(j);
                newPaths.push_back(thisPathCopy);
            }
            else{
                continue;
            }
        }
    }

    outputPaths = std::vector<std::vector<int>>(newPaths);
}

float distanceBetween(float x1, float y1, float x2, float y2){
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int main(int argc, char** argv) {
    
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Clear file content (overwrite old file on each run)
    std::ofstream clearFile("contest.txt", std::ios::trunc);
    clearFile.close();

    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        ROS_INFO("x/y/phi: %.2f/%.2f/%.2f", boxes.coords[i][0], boxes.coords[i][1], RAD2DEG(boxes.coords[i][2]));
    }
    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n);

    // Timer setup and Settings
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t reportingInterval = 0.5;
    uint64_t lastReportTimestamp = 0;

    std::map<int, int> templateCounts; // Template occurrence count
    std::map<int, int> bestMatchPerDestination; // Best matching template for each destination

    #pragma region Get Starting Pose
    ros::Rate loopRate(10);
    while (ros::ok() && !robotPose.pose_received) {
        ros::spinOnce();
        ROS_WARN("Waiting for initial pose estimate. Please use RViz's '2D Pose Estimate' tool.");
        loopRate.sleep();
    }
    ros::Duration(2.0).sleep();
    ROS_INFO("Initial pose: (%.2f, %.2f, %.2f)", robotPose.x, robotPose.y, robotPose.phi);
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;
    #pragma endregion

    #pragma region Crude Fixes
    // Initialize OpenCL runtime (not sure if this causes failure or not)
    imagePipeline.getTemplateID(boxes);
    ros::Duration(0.01).sleep();
    // Crude fix for destination writing condition ... 
    std::ofstream outFile("contest.txt", std::ios::app);
    #pragma endregion
    
    #pragma region TSP Brute Force Algo
    std::vector<std::vector<float>> TSPDestinations = {{startX, startY, startPhi}};

    for(int i = 0; i < boxes.coords.size(); i++){
        float tempX = boxes.coords[i][0];
        float tempY = boxes.coords[i][1];
        float tempPhi = boxes.coords[i][2];

        offsetCoordinates(offsetFromTarget, tempX, tempY, tempPhi, tempX, tempY);
        TSPDestinations.push_back({tempX, tempY, tempPhi});
    }

    TSPDestinations.push_back({startX, startY, startPhi});

    std::vector<std::vector<int>> AllTSPPaths = {{0}};
    const int nDestinations = TSPDestinations.size();

    for(int i = 0; i < nDestinations-2; i ++){
        addNextDestinations(AllTSPPaths, nDestinations-1, AllTSPPaths);
    }

    for(int i = 0; i < AllTSPPaths.size(); i ++){
        AllTSPPaths[i].push_back(0);
    }

    for(int i = 0; i < AllTSPPaths.size(); i++){
        for(int j = 0; j < AllTSPPaths[0].size(); j++){
            std::cout << AllTSPPaths[i][j];
            std::cout << ",";
        }
        std::cout << std::endl;
    }

    // Fill distance matrix
    std::vector<std::vector<int>> distancesArray(nDestinations - 1, std::vector<int>(nDestinations - 1, 0));
    for(int i = 0; i < nDestinations-1; i++){
        for(int j = 0; j < nDestinations-1; j++){
            distancesArray[i][j] = distanceBetween(TSPDestinations[i][0], TSPDestinations[i][1], TSPDestinations[j][0], TSPDestinations[j][1]);
        }
    }

    // Find path with shortest distance
    float distanceSum;
    float minDistance = INFINITY;
    int shortestPathIndex = 0;

    for(int i = 0; i < AllTSPPaths.size(); i++){
        distanceSum = 0;
        for(int j = 0; j < AllTSPPaths[0].size()-1; j++){
            distanceSum += distancesArray[AllTSPPaths[i][j]][AllTSPPaths[i][j+1]];
        }
        if(distanceSum < minDistance){
            minDistance = distanceSum;
            shortestPathIndex = i;
        }

    }

    std::vector<int> shortestPath = AllTSPPaths[shortestPathIndex];

    ROS_INFO("Shortest path index: %d", shortestPathIndex);




    #pragma endregion

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();

        #pragma region Status Printing
        if(secondsElapsed > lastReportTimestamp + reportingInterval){
            // Print additional report info here if needed.
            lastReportTimestamp = secondsElapsed;
        }
        #pragma endregion

        #pragma region Target Position Calculation
        destinationNumber = shortestPath[destinationIndex];
        targetX = TSPDestinations[destinationNumber][0];
        targetY = TSPDestinations[destinationNumber][1];
        destPhi = TSPDestinations[destinationNumber][2];
        // If facing inwards, subtract PI to flip orientation. If it's destination 0 (origin), don't flip the angle
        if(facingInwards && destinationNumber != 0){
            targetPhi = destPhi - M_PI;
            while(targetPhi < -2*M_PI){
                targetPhi += 2*M_PI;
            }
        }
        else{
            targetPhi = destPhi;
        }
        ROS_INFO("V-----------------------------------------------V");
        // ROS_INFO("Moving to destination %u of %lu at x/y/phi: %.2f/%.2f/%.2f", destinationIndex, boxes.coords.size()-1, destX, destY, RAD2DEG(destPhi));
        ROS_INFO("Adjusted position x/y/phi: %.2f/%.2f/%.2f", targetX, targetY, RAD2DEG(targetPhi));       
        #pragma endregion

        #pragma region Movement Execution
        // Wait for AMCL convergence for every destination.
        waitForConvergence(robotPose);
        if(!Navigation::moveToGoal(targetX, targetY, targetPhi)){
            ROS_INFO("moveToGoal error.");
        }
        else{
            ROS_INFO("Destination reached!");
            ros::Duration(1).sleep();
        }
        #pragma endregion

        #pragma region Image Processing
        if(shortestPath[destinationIndex] != 0){            
            imagePipeline.getTemplateID(boxes);
            ros::Duration(1).sleep();
            ros::spinOnce();
            // Image processing section.
            int templateID = imagePipeline.getTemplateID(boxes);
            if (templateID != -1) {
                templateCounts[templateID]++; // Update global statistics
                bestMatchPerDestination[destinationIndex] = templateID; // Update best match for current destination
                ROS_INFO("Detected template %d at destination %d", templateID, shortestPath[destinationIndex]);
            }
            else{
                ROS_INFO("Detected no template.");
            }

            // Append statistics every second.
            // Initialize template names array.
            std::string templateNames[] = {"Raisin Bran", "Cinnamon Toast Crunch", "Rice Krispies", "Blank"};

            auto currentTime = std::chrono::system_clock::now();
            while(!outFile.is_open());
            if (outFile.is_open()) {
                outFile << "===== Timestamp: " << std::chrono::system_clock::to_time_t(currentTime) << " =====" << std::endl;
                if (bestMatchPerDestination.find(destinationIndex) != bestMatchPerDestination.end()) {
                    int bestMatch = bestMatchPerDestination[destinationIndex];

                    // 根据模板ID获取模板名称
                    std::string templateName;
                    if (bestMatch == -1) {
                        templateName = templateNames[3]; // 使用 'blank'
                    } else if (bestMatch >= 0 && bestMatch < 3) {
                        templateName = templateNames[bestMatch]; // 使用对应的模板名称
                    } else {
                        templateName = "unknown"; // 处理意外情况
                    }

                    // 获取模板出现次数
                    int count = (bestMatch == -1) ? 0 : templateCounts[bestMatch];

                    // 写入当前目标点的最佳匹配信息和机器人位置
                    outFile << "Destination " << std::to_string(shortestPath[destinationIndex])
                            << " | Robot Position: ( X: " << targetX << ", Y: " << targetY << ", Phi: " << targetPhi << ")" 
                            << templateName 
                            << " (appeared " << count << " times), "
                            << std::endl;
                    } else {
                        // 如果当前目标点没有匹配 safe procaution
                        outFile << "Destination " << std::to_string(shortestPath[destinationIndex]) << ": blank (appeared 0 times), "
                                << " | Robot Position: ( X: " << targetX << ", Y: " << targetY << ", Phi:" << targetPhi << ")" << std::endl;
                    }

                outFile << std::endl;
            } else {
                ROS_ERROR("Failed to write to contest.txt");
            }
        }
        #pragma endregion
        
        // End condition.
        destinationIndex++;
        if(destinationIndex >= shortestPath.size()){
            break;
        }
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    }

    // #pragma region Return to Starting Position
    // ROS_INFO("Returning to starting position (%.2f, %.2f, %.2f deg)...", startX, startY, startPhi);
    // waitForConvergence(robotPose);
    // if (!Navigation::moveToGoal(startX, startY, startPhi)) {
    //     ROS_INFO("...failed to return to starting position.");
    // }
    // else{
    //     ROS_INFO("...successfully returned to starting position.");
    // }
    // #pragma endregion

    // Write final statistics before program ends.
    std::ofstream finalFile("contest.txt", std::ios::app);
    if (finalFile.is_open()) {
        finalFile << "===== FINAL STATISTICS =====" << std::endl;
        for (const auto& pair : bestMatchPerDestination) {
            int dest = pair.first;
            int bestMatch = pair.second;
            finalFile << "Destination " << shortestPath[dest] << ": Template " << bestMatch 
                      << " (appeared " << templateCounts[bestMatch] << " times)" << std::endl;
        }
    }
 
    ROS_INFO("------PROGRAM END------");
    return 0;
}
