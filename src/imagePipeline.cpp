#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        // Convert the incoming image to grayscale.
        cv::Mat grayImg;
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

        double bestScore = -1.0;
        int bestIndex = -1;
        // Loop over each template in boxes.templates.
        for (size_t i = 0; i < boxes.templates.size(); i++) {
            cv::Mat templ = boxes.templates[i];
            if(templ.empty())
                continue;
            cv::Mat result;
            // Perform template matching using normalized correlation.
            cv::matchTemplate(grayImg, templ, result, cv::TM_CCOEFF_NORMED);
            double minVal, maxVal;
            cv::minMaxLoc(result, &minVal, &maxVal, nullptr, nullptr);
            if(maxVal > bestScore) {
                bestScore = maxVal;
                bestIndex = i;
            }
        }
        // Display the current grayscale image (for debugging).
        cv::imshow("view", grayImg);
        cv::waitKey(10);

        // Set a threshold for a "good" match.
        double threshold = 0.8;  // Adjust this value as needed.
        if(bestScore >= threshold) {
            template_id = bestIndex;
        } else {
            template_id = -1;
        }
    }
    return template_id;
}
