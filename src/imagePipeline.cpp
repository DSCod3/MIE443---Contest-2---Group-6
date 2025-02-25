#include "imagePipeline.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "boxes.h"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // Kinect: "camera/rgb/image_raw", webcam: "camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
    // Create a SURF detector with a hessian threshold (tunable parameter).
    detector = cv::xfeatures2d::SURF::create(400);
    // Use a brute-force matcher with L2 norm (SURF descriptors are float).
    matcher = cv::BFMatcher::create(cv::NORM_L2);
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        img = cv_bridge::toCvShare(msg, IMAGE_TYPE)->image.clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Image conversion error: %s", e.what());
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int bestTemplateID = -1;
    int maxGoodMatches = 0;
    
    if (!isValid || img.empty()) {
        ROS_ERROR("Invalid image for SURF detection");
        return -1;
    }
    
    // Convert the current image to grayscale.
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    
    // Detect keypoints and compute descriptors for the current image.
    std::vector<cv::KeyPoint> keypointsImg;
    cv::Mat descriptorsImg;
    detector->detectAndCompute(gray, cv::noArray(), keypointsImg, descriptorsImg);
    
    if (descriptorsImg.empty()) {
        ROS_WARN("No descriptors found in current image");
        return -1;
    }
    
    // For each template in boxes.templates, perform SURF matching.
    for (size_t i = 0; i < boxes.templates.size(); i++) {
        cv::Mat templ = boxes.templates[i];
        if (templ.empty()) continue;
        
        cv::Mat templGray;
        if (templ.channels() == 3)
            cv::cvtColor(templ, templGray, cv::COLOR_BGR2GRAY);
        else
            templGray = templ;
        
        std::vector<cv::KeyPoint> keypointsTempl;
        cv::Mat descriptorsTempl;
        detector->detectAndCompute(templGray, cv::noArray(), keypointsTempl, descriptorsTempl);
        
        if (descriptorsTempl.empty()) continue;
        
        // Match descriptors between template and current image.
        std::vector< std::vector<cv::DMatch> > knnMatches;
        matcher->knnMatch(descriptorsTempl, descriptorsImg, knnMatches, 2);
        
        int goodMatches = 0;
        // Lowe's ratio test added (check https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html for documentation)
        for (size_t j = 0; j < knnMatches.size(); j++) {
            if (knnMatches[j].size() < 2) continue;
            if (knnMatches[j][0].distance < 0.75 * knnMatches[j][1].distance)
                goodMatches++;
        }
        
        if (goodMatches > maxGoodMatches) {
            maxGoodMatches = goodMatches;
            bestTemplateID = i;
        }
    }
    
    // Set a threshold for a valid match (tunable parameter (ideally larger than 10)).
    if (maxGoodMatches < 20) { 
        bestTemplateID = -1;
    }
    
    // Optional: Display the current grayscale image.
    cv::imshow("SURF Image", gray);
    cv::waitKey(10);
    
    return bestTemplateID;
}
