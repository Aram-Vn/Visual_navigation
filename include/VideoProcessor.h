#ifndef AIRDEV_VISUAL_NAVIGATION_INCLUDE_VIDEO_PROCESSOR_H
#define AIRDEV_VISUAL_NAVIGATION_INCLUDE_VIDEO_PROCESSOR_H

#include "constant_parameters.h"
#include "utilities.h"

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
        explicit VideoProcessor(const std::string& filePath, float altitude = 150.0);
        VideoProcessor(int cameraID = 0);
        ~VideoProcessor();

    public:
        void processFrames(void);
        void displayResults(const cv::Mat& frame);

    private:
        cv::VideoCapture         m_cap;
        cv::Mat                  m_prev_frame;
        cv::Mat                  m_curr_frame;
        std::vector<cv::Point2f> m_points[2];
        float                    m_total_X;
        float                    m_total_Y;
        double                   m_yav_res;
        double                   m_abs_total_X;
        double                   m_abs_total_Y;
        const float              m_displacementThreshold;
        droneMockData            m_drone_mock_data;
    };
} // namespace Air

#endif // AIRDEV_VISUAL_NAVIGATION_INCLUDE_VIDEO_PROCESSOR_H