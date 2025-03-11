#include "imagePipeline.h"
#include "boxes.h"  // Include the full definition of Boxes.
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw"  // Kinect: "camera/rgb/image_raw", webcam: "camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
    // Create a SURF detector with a hessian threshold (tunable).
    detector = cv::xfeatures2d::SURF::create(400);
    // Create a BFMatcher with L2 norm (SURF descriptors are float).
    matcher = cv::BFMatcher::create(cv::NORM_L2);
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        img = cv_bridge::toCvShare(msg, IMAGE_TYPE)->image.clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cerr << "Image conversion error: " << e.what() << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int bestTemplateID = -1;
    int maxGoodMatches = 0;
    
    if (!isValid || img.empty()) {
        std::cerr << "Invalid image for SURF detection" << std::endl;
        return -1;
    }
    
    // Crop the image to a central region.
    int cropWidth = img.cols / 2;
    int cropHeight = img.rows / 2;
    int startX = (img.cols - cropWidth) / 2;
    int startY = (img.rows - cropHeight) / 2;
    cv::Rect roi(startX, startY, cropWidth, cropHeight);
    cv::Mat cropped;
    try {
        cropped = img(roi).clone();
    } catch (const cv::Exception &e) {
        std::cerr << "Error cropping image: " << e.what() << std::endl;
        return -1;
    }
    
    // Convert cropped image to grayscale.
    cv::Mat gray;
    cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
    
    // Detect keypoints and compute descriptors in the cropped image.
    std::vector<cv::KeyPoint> keypointsImg;
    cv::Mat descriptorsImg;
    detector->detectAndCompute(gray, cv::noArray(), keypointsImg, descriptorsImg);
    
    if (descriptorsImg.empty()) {
        std::cerr << "No descriptors found in current image" << std::endl;
        return -1;
    }
    
    // Iterate through each template loaded in Boxes.
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
        
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descriptorsTempl, descriptorsImg, knnMatches, 2);
        
        int goodMatches = 0;
        // Apply Lowe's ratio test.
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
    
    // Set a threshold for detection (e.g., at least 10 good matches).
    if (maxGoodMatches < 10)
        bestTemplateID = -1;
    
    // Optionally display the cropped image.
    cv::imshow("Cropped Image", gray);
    cv::waitKey(10);
    
    return bestTemplateID;
}
