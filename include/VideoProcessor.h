#ifndef AIRDEV_VISUAL_NAVIGATION_INCLUDE_VIDEO_PROCESSOR_H
#define AIRDEV_VISUAL_NAVIGATION_INCLUDE_VIDEO_PROCESSOR_H

#include "constant_parameters.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace Air {

    class VideoProcessor
    {
    public:
        explicit VideoProcessor(const std::string& filePath);
        VideoProcessor(int cameraID);
        ~VideoProcessor();

    public:
        void processFrames();
        void displayResults(const cv::Mat& frame);

    private:
        cv::VideoCapture         cap;
        cv::Mat                  prevFrame;
        cv::Mat                  currFrame;
        std::vector<cv::Point2f> points[2];
        float                    total_X;
        float                    total_Y;
        double                   yavRes;
        double                   abs_total_X;
        double                   abs_total_y;
    };
} // namespace Air

#endif // AIRDEV_VISUAL_NAVIGATION_INCLUDE_VIDEO_PROCESSOR_H