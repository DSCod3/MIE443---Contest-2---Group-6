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

    cv::Mat cropped;
    try {
        // ===== ADAPTIVE CROPPING USING EDGE DETECTION =====
        cv::Mat grayFull;
        cv::cvtColor(img, grayFull, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grayFull, grayFull, cv::Size(5, 5), 0);

        // Edge detection with Canny
        cv::Mat edges;
        cv::Canny(grayFull, edges, 50, 150);

        // Find contours in the edge map
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour by area
            int maxIdx = -1;
            double maxArea = 0;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area > maxArea) {
                    maxArea = area;
                    maxIdx = i;
                }
            }

            if (maxIdx != -1) {
                // Get bounding rect with 10% padding
                cv::Rect boundingRect = cv::boundingRect(contours[maxIdx]);
                int padX = boundingRect.width * 0.1;
                int padY = boundingRect.height * 0.1;
                
                // Expand ROI with boundary checks
                boundingRect.x = std::max(0, boundingRect.x - padX);
                boundingRect.y = std::max(0, boundingRect.y - padY);
                boundingRect.width = std::min(img.cols - boundingRect.x, 
                                           boundingRect.width + 2 * padX);
                boundingRect.height = std::min(img.rows - boundingRect.y,
                                            boundingRect.height + 2 * padY);

                cropped = img(boundingRect).clone();
            }
        }

        // Fallback to center crop if no contours found
        if (cropped.empty()) {
            int cropWidth = img.cols / 2;
            int cropHeight = img.rows / 2;
            cv::Rect roi((img.cols - cropWidth)/2, (img.rows - cropHeight)/2,
                       cropWidth, cropHeight);
            cropped = img(roi).clone();
        }

    } catch (const cv::Exception &e) {
        std::cerr << "Cropping error: " << e.what() << std::endl;
        return -1;
    }

    // ===== REST OF SURF PROCESSING =====
    cv::Mat gray;
    cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
    
    std::vector<cv::KeyPoint> keypointsImg;
    cv::Mat descriptorsImg;
    detector->detectAndCompute(gray, cv::noArray(), keypointsImg, descriptorsImg);

    // ... (rest of your existing SURF matching code) ...

    // Display adaptive crop instead of center crop
    cv::imshow("Adaptive Crop", gray);
    cv::waitKey(10);

    return bestTemplateID;
}