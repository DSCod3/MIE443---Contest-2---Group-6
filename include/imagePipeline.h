#ifndef IMAGEPIPELINE_H
#define IMAGEPIPELINE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

class Boxes; // Forward declaration to avoid circular dependency.

class ImagePipeline {
public:
    ImagePipeline(ros::NodeHandle& n);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    // Uses SURF to detect a tag from the current image against the provided templates.

    // Returns the index of the best matching template (or -1 if no good match is found).
    int getTemplateID(Boxes& boxes);

    cv::Rect adaptiveCropping() ;
    
    bool isValid;
    cv::Mat img;
    
private:
    image_transport::Subscriber sub;
    cv::Ptr<cv::xfeatures2d::SURF> detector;
    cv::Ptr<cv::BFMatcher> matcher;
};

#endif