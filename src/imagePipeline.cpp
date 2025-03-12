#include "imagePipeline.h"
#include "boxes.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <fstream>  // Added for file output

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw"
#define DEBUG_MODE true  // 启用调试显示

// 轮廓筛选参数
constexpr double MIN_AREA_RATIO = 0.02;   // 最小区域占比
constexpr double MAX_AREA_RATIO = 0.98;    // 最大区域占比
constexpr double MIN_ASPECT_RATIO = 0.3;   // 最小宽高比
constexpr double MAX_ASPECT_RATIO = 3.0;   // 最大宽高比
constexpr double CENTER_WEIGHT = 0.6;      // 中心位置权重

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
    detector = cv::xfeatures2d::SURF::create(500);  // 提高Hessian阈值
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

cv::Rect ImagePipeline::adaptiveCropping() {
    cv::Mat processingImg = img.clone();
    
    // Stage 1: Preprocessing
    cv::Mat gray, blurred;
    cv::cvtColor(processingImg, gray, cv::COLOR_BGR2GRAY);
    
    // Adaptive Gaussian blur
    int blurKernel = static_cast<int>(gray.cols * 0.005) | 1;
    blurKernel = std::max(3, std::min(blurKernel, 11));
    cv::GaussianBlur(gray, blurred, cv::Size(blurKernel, blurKernel), 0);

    // Stage 2: Dynamic edge detection
    // Custom median calculation
    cv::Mat flat;
    blurred.reshape(1, 1).copyTo(flat); // Flatten the image into a single row
    cv::sort(flat, flat, cv::SORT_ASCENDING); // Sort the flattened array
    double medVal = flat.at<uchar>(flat.total() / 2); // Get the middle value

    double lowerThresh = std::max(0.0, 0.67 * medVal);
    double upperThresh = std::min(255.0, 1.33 * medVal);
    cv::Mat edges;
    cv::Canny(blurred, edges, lowerThresh, upperThresh);

    // Stage 3: Contour analysis
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    cv::Rect bestROI;
    double maxScore = 0.0;

    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        
        // Area filtering
        double areaRatio = cv::contourArea(contour) / (gray.cols * gray.rows);
        if (areaRatio < MIN_AREA_RATIO || areaRatio > MAX_AREA_RATIO) continue;

        // Aspect ratio filtering
        double aspect = static_cast<double>(rect.width) / rect.height;
        if (aspect < MIN_ASPECT_RATIO || aspect > MAX_ASPECT_RATIO) continue;

        // Position scoring
        cv::Point center(rect.x + rect.width/2, rect.y + rect.height/2);
        double centerDist = cv::norm(center - cv::Point(gray.cols/2, gray.rows/2));
        double positionScore = 1.0 - (centerDist / (gray.cols * 0.5));

        // Total score
        double totalScore = (CENTER_WEIGHT * positionScore) + 
                          ((1 - CENTER_WEIGHT) * areaRatio);

        if (totalScore > maxScore) {
            maxScore = totalScore;
            bestROI = rect;
        }
    }

    // Stage 4: Dynamic ROI adjustment
    if (maxScore > 0.3 && !bestROI.empty()) {
        double sizeFactor = static_cast<double>(bestROI.area()) / img.size().area();
        double padding = 0.1 + 0.15 * (1.0 - sizeFactor);

        bestROI.x -= bestROI.width * padding;
        bestROI.y -= bestROI.height * padding;
        bestROI.width *= (1 + 2 * padding);
        bestROI.height *= (1 + 2 * padding);

        // Boundary constraints
        bestROI.x = std::max(0, bestROI.x);
        bestROI.y = std::max(0, bestROI.y);
        bestROI.width = std::min(img.cols - bestROI.x, bestROI.width);
        bestROI.height = std::min(img.rows - bestROI.y, bestROI.height);
        
        return bestROI;
    }

    // Stage 5: Fallback strategy
    const std::vector<double> cropRatios{0.7, 0.5, 0.3};
    for (double ratio : cropRatios) {
        int size = static_cast<int>(img.cols * ratio);
        cv::Rect roi((img.cols - size)/2, (img.rows - size)/2, size, size);
        
        cv::Mat testCrop = img(roi);
        std::vector<cv::KeyPoint> kp;
        detector->detect(testCrop, kp);
        if (kp.size() > 15) return roi;
    }

    // Final fallback: center 50%
    return cv::Rect(
        img.cols/4, img.rows/4, 
        img.cols/2, img.rows/2
    );
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    if (!isValid || img.empty()) {
        std::cerr << "Invalid image for processing" << std::endl;
        return -1;
    }

    // 执行自适应裁剪
    cv::Rect roi = adaptiveCropping();
    cv::Mat cropped;
    try {
        cropped = img(roi).clone();
        
        // 尺寸保障机制
        if (cropped.rows < 50 || cropped.cols < 50) {
            cv::resize(cropped, cropped, cv::Size(100, 100), 0, 0, cv::INTER_LANCZOS4);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "Cropping failed: " << e.what() << std::endl;
        return -1;
    }

    // SURF特征处理
    cv::Mat gray;
    cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
    
    std::vector<cv::KeyPoint> kpImage;
    cv::Mat descImage;
    detector->detectAndCompute(gray, cv::noArray(), kpImage, descImage);

    if (descImage.empty()) {
        if (DEBUG_MODE) {
            std::cout << "No features detected in cropped image" << std::endl;
        }
        return -1;
    }

    // 模板匹配优化
    int bestMatch = -1;
    int maxMatches = 0;
    const int MIN_MATCHES = 15;

    for (size_t i = 0; i < boxes.templates.size(); ++i) {
        if (boxes.templates[i].empty()) continue;

        cv::Mat templGray;
        if (boxes.templates[i].channels() == 3) {
            cv::cvtColor(boxes.templates[i], templGray, cv::COLOR_BGR2GRAY);
        } else {
            templGray = boxes.templates[i];
        }

        std::vector<cv::KeyPoint> kpTemplate;
        cv::Mat descTemplate;
        detector->detectAndCompute(templGray, cv::noArray(), kpTemplate, descTemplate);

        if (descTemplate.empty()) continue;

        // 双向匹配验证
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(descTemplate, descImage, matches, 2);
        
        std::vector<cv::DMatch> validMatches;
        for (const auto& m : matches) {
            if (m.size() >= 2 && m[0].distance < 0.7 * m[1].distance) {
                if (m[0].queryIdx >= 0 && m[0].queryIdx < kpTemplate.size() &&
                    m[0].trainIdx >= 0 && m[0].trainIdx < kpImage.size()) {
                    validMatches.push_back(m[0]);
                }
            }
        }

        // 反向验证
        matcher->knnMatch(descImage, descTemplate, matches, 2);
        for (const auto& m : matches) {
            if (m.size() >= 2 && m[0].distance < 0.7 * m[1].distance) {
                if (m[0].queryIdx >= 0 && m[0].queryIdx < kpImage.size() &&
                    m[0].trainIdx >= 0 && m[0].trainIdx < kpTemplate.size()) {
                    validMatches.push_back(cv::DMatch(m[0].trainIdx, m[0].queryIdx, m[0].distance));
                }
            }
        }

        // 调试：显示每个模板的匹配结果
        if (DEBUG_MODE && !validMatches.empty()) {
            cv::Mat matchImg;
            cv::drawMatches(templGray, kpTemplate, gray, kpImage, validMatches, matchImg);
            cv::putText(matchImg, "Template " + std::to_string(i) + " Matches: " + std::to_string(validMatches.size()),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Template Matches", matchImg);
            cv::waitKey(10);
        }

        if (validMatches.size() > maxMatches && validMatches.size() >= MIN_MATCHES) {
            maxMatches = validMatches.size();
            bestMatch = i;
        }
    }
    
    return (maxMatches >= 50) ? bestMatch : -1;
}