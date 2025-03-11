#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>


#define DEG2RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD2DEG(rad) ((rad) * (180.0 / M_PI))

bool facingInwards = true;
float offsetFromTarget = 0.50;


// Initialize box coordinates and templates
uint8_t destinationNumber = 0;
float targetX;
float targetY;
float targetPhi;
float destX;
float destY;
float destPhi;
uint64_t timeReference;

int main(int argc, char** argv) {
    
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;


    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        //std::cout << "Box coordinates: " << std::endl;
        //std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
        //          << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Timer setup and Settings
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t reportingInterval = 0.5;
    uint64_t lastReportTimestamp = 0;
    
    for(int i=0; i < boxes.coords.size(); i++){
        ROS_INFO("x/y/phi: %.2f/%.2f/%.2f", boxes.coords[i][0], boxes.coords[i][1], RAD2DEG(boxes.coords[i][2]));
    }
    
    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();


        // Status Printing
        if(secondsElapsed > lastReportTimestamp + reportingInterval){
            // Print stuff to report here
           // ROS_INFO("AbsX/AbsY/AbsYaw %.2f, %.2f, %.2f", robotPose.x, robotPose.y, robotPose.phid);


            // End of printing
            lastReportTimestamp = secondsElapsed;
        }

        destX = boxes.coords[destinationNumber][0];
        destY = boxes.coords[destinationNumber][1];
        //destPhi = boxes.coords[destinationNumber][2];
        destPhi = DEG2RAD(boxes.coords[destinationNumber][2]);

        if(facingInwards){
            targetPhi = destPhi - M_PI;
            while(targetPhi < -2*M_PI){
                targetPhi += 2*M_PI;
            }
        }
        else{
            targetPhi = destPhi;
        }

        ROS_INFO("V-----------------------------------------------V");
        ROS_INFO("Moving to destination %u at x/y/phi: %.2f/%.2f/%.2f", destinationNumber, destX, destY, RAD2DEG(destPhi));

        offsetCoordinates(offsetFromTarget, destX, destY, destPhi, targetX, targetY);
        ROS_INFO("Adjusted position x/y/phi: %.2f/%.2f/%.2f", targetX, targetY, RAD2DEG(targetPhi));       
        
        if(!Navigation::moveToGoal(targetX, targetY, targetPhi)){
            ROS_INFO("moveToGoal error.");
            destinationNumber++;
        }

        else{
            
            ROS_INFO("Destination reached!");
            ros::Duration(5).sleep();
            destinationNumber++;

            if(destinationNumber > boxes.coords.size()){
                break;
            }
        }





        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();7
    
    }

    ROS_INFO("------PROGRAM END------");
    return 0;
}
