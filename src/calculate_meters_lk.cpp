#include "../include/constant_parameters.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
// #include <chrono>
// #include <thread>

double getYawFromRotationMatrix(const cv::Mat& Mat)
{
    if (!(Mat.rows == 3) && !(Mat.cols == 3))
    {
        std::cout << "NO!";
    }

    // Extract yaw (rotation around the z-axis)
    double yaw_d = std::atan2(Mat.at<double>(1, 0), Mat.at<double>(0, 0)); // atan2(r21, r11)

    return yaw_d;
}

// clang-format off
cv::Mat createRotationMatrix(float pitch_val, float yaw_val, float roll_val)
{
    // Convert degrees to radians
    float rollRad  = roll_val * (M_PI / 180.0f);  // Roll: X-axis rotation
    float pitchRad = pitch_val * (M_PI / 180.0f); // Pitch: Y-axis rotation
    float yawRad   = yaw_val * (M_PI / 180.0f);   // Yaw: Z-axis rotation

    // Z-axis (Yaw)
    cv::Mat yawMatrix = (cv::Mat_<float>(3, 3) << 
        std::cos(yawRad),  -std::sin(yawRad),      0,
        std::sin(yawRad),  std::cos(yawRad),       0,
        0,                    0,                        1);

    // Y-axis (Pitch)
    cv::Mat pitchMatrix = (cv::Mat_<float>(3, 3) << 
        std::cos(pitchRad),   0,   std::sin(pitchRad),
        0,                      1,    0,
        -std::sin(pitchRad),  0,   std::cos(pitchRad));

    // X-axis (Roll)
    cv::Mat rollMatrix = (cv::Mat_<float>(3, 3) << 
        1,  0,                      0,
        0,  std::cos(rollRad),   -std::sin(rollRad),
        0,  std::sin(rollRad),   std::cos(rollRad));

    cv::Mat rotationMatrix = rollMatrix * pitchMatrix * yawMatrix;

    return rotationMatrix;
}
// clang-format on

std::vector<cv::Point2f> filterPointsNearCenter(const std::vector<cv::Point2f>& points, const cv::Size& frameSize,
                                                float maxDistance)
{
    std::vector<cv::Point2f> filteredPoints;
    cv::Point2f              center(frameSize.width / 2.0f,
                                    frameSize.height / 2.0f); // Center of the frame

    for (const auto& point : points)
    {
        float distance = cv::norm(point - center);

        if (distance < maxDistance)
        {
            filteredPoints.push_back(point);
        }
    }

    return filteredPoints;
}

// Function to calculate the median of a vector
float calculateMedian(std::vector<float>& vec)
{
    size_t size = vec.size();
    if (size == 0)
        return 0;

    std::sort(vec.begin(), vec.end());
    if (size % 2 == 0)
    {
        return (vec[size / 2 - 1] + vec[size / 2]) / 2.0;
    }
    else
    {
        return vec[size / 2];
    }
}

// Function to calculate the pixel size
float calculatePixelSize(float altitude, float fov, int resolution)
{
    return (2.0f * altitude * std::tan(fov * (M_PI / 180.0f) / 2.0f)) / resolution;
}

int main()
{
    float abs_total_X = 0.0f;
    float abs_total_y = 0.0f;

    float  total_X = 0.0;
    float  total_Y = 0.0;

    double yav_res = 0.0;

    // Open the video file
    // cv::VideoCapture cap("/home/aram/Downloads/straight_down_tville.mp4");
    // cv::VideoCapture cap("/home/aram/Downloads/Aerial_Fall_Foliage_.mp4");
    // cv::VideoCapture
    // cap("/home/aram/Downloads/real_camera_test_114cm_out.mp4");
    // cv::VideoCapture
    // cap("/home/aram/Videos/Screencasts/out_test_1383h_608d.webm");

    // real life
    // cv::VideoCapture cap("/home/aram/Videos/Screencasts/output_flight.webm");
    cv::VideoCapture cap("/home/aram/Downloads/yav_test.mp4");

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open video file." << std::endl;
        return -1;
    }

    cv::Mat prevFrame;
    cv::Mat currFrame;
    cap >> prevFrame;
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> points[2]; // To hold points from previous and current frame
    std::vector<uchar>       status;    // Status vector for tracking points
    std::vector<float>       err;       // Error vector

    cv::goodFeaturesToTrack(prevFrame, points[0], 120, 0.01, 10);

    while (true)
    {
        cap >> currFrame; // Capture the current frame

        if (currFrame.empty())
        {
            std::cout << "Empty frame" << std::endl;
            std::cout << "\nabs_total_X: " << abs_total_X << std::endl;
            std::cout << "\nabs_total_y: " << abs_total_y << std::endl;
            std::cout << "\nyav_res (in degrees): " << yav_res << std::endl;
            break;
        }

        cv::Mat grayCurrFrame;
        cv::cvtColor(currFrame, grayCurrFrame,
                     cv::COLOR_BGR2GRAY); // Convert to grayscale

        // clang-format off
        cv::calcOpticalFlowPyrLK(prevFrame, grayCurrFrame, points[0], points[1], 
                                status,  err, cv::Size(15, 15), 3, 
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        // clang-format on

        std::vector<cv::Point2f> inliersOld, inliersNew;
        for (size_t i = 0; i < points[1].size(); i++)
        {
            if (status[i])
            {
                inliersOld.push_back(points[0][i]);
                inliersNew.push_back(points[1][i]);
            }
        }

        // Use RANSAC to estimate the homography
        cv::Mat inliersMask;
        cv::Mat homography = cv::findHomography(inliersOld, inliersNew, cv::RANSAC, 3.0, inliersMask);

        if (!homography.empty())
        {
            // clang-format off
            double yaw_angle = std::atan2(homography.at<double>(1, 0), 
                                          homography.at<double>(0, 0)) * 180.0 / CV_PI;
            // clang-format on

            std::cout << "Estimated Yaw Angle: " << yaw_angle << " degrees" << std::endl;

            yav_res += yaw_angle;
        }

        std::vector<float> totalDisplacementVec;
        std::vector<float> totalAngleVec;

        const float displacementThreshold = 0.1f;

        for (int i = 0; i < inliersMask.rows; i++)
        {
            if (inliersMask.at<uchar>(i))
            {
                float dx = inliersNew[i].x - inliersOld[i].x;
                float dy = inliersNew[i].y - inliersOld[i].y;

                double displacement = std::hypot(dx, dy);
                totalDisplacementVec.push_back(displacement);

                if (displacement < displacementThreshold)
                    continue;

                float angle = std::atan2(dy, dx);
                totalAngleVec.push_back(angle);

                // Draw green circle for tracked points
                cv::circle(currFrame, inliersNew[i], 3, cv::Scalar(0, 255, 0), -1);
            }
        }

        if (!totalDisplacementVec.empty())
        {
            float medianDisplacement = calculateMedian(totalDisplacementVec);
            float medianAngle        = calculateMedian(totalAngleVec);
            float medianAngleDegrees = medianAngle * (180.0 / CV_PI); // Convert angle to degrees

            std::cout << "Median Displacement: " << medianDisplacement << ", "
                      << "Median Angle: " << medianAngleDegrees << " degrees." << std::endl;

            float pixelSizeX = calculatePixelSize(droneMockData::Altitude, CameraParameters::Horizontal_FOV,
                                                  CameraParameters::Resolution_width);
            float pixelSizeY = calculatePixelSize(droneMockData::Altitude, CameraParameters::Vertical_FOV,
                                                  CameraParameters::Resolution_height);

            float displacementX = medianDisplacement * std::cos(medianAngle);
            float displacementY = medianDisplacement * std::sin(medianAngle);

            float displacementZ = 0.0f;

            // Create the displacement vector
            cv::Mat displacementVec = (cv::Mat_<float>(3, 1) << displacementX, displacementY, displacementZ);

            cv::Mat rotationMatrix =
                createRotationMatrix(droneMockData::pitch, droneMockData::yaw, droneMockData::roll);

            cv::Mat transformedDisplacement = rotationMatrix * displacementVec;
            total_X += transformedDisplacement.at<float>(0) * pixelSizeX;
            total_Y += transformedDisplacement.at<float>(1) * pixelSizeY;

            abs_total_X += std::abs(transformedDisplacement.at<float>(0) * pixelSizeX);
            abs_total_y += std::abs(transformedDisplacement.at<float>(1) * pixelSizeY);

            // total_X += displacementX * pixelSizeX;
            // total_Y += displacementY * pixelSizeY;

            // abs_total_X += std::abs(displacementX * pixelSizeX);
            // abs_total_y += std::abs(displacementY * pixelSizeY);

            std::cout << "Real-world total_X in X: " << total_X << " m" << std::endl;
            std::cout << "Real-world total_Y in Y: " << total_Y << " m" << std::endl;
        }

        // Prepare for the next iteration
        points[0] = points[1];
        prevFrame = std::move(grayCurrFrame);

        cv::goodFeaturesToTrack(prevFrame, points[0], 120, 0.01,
                                5); // Reinitialize points if needed

        // cv::cornerSubPix(prevFrame, points[0], cv::Size(5, 5), cv::Size(-1, -1),
        //                  cv::TermCriteria(cv::TermCriteria::EPS +
        //                  cv::TermCriteria::COUNT, 30, 0.01));

        cv::imshow("Tracked Points", currFrame);
        if (cv::waitKey(30) >= 0)
            break; // Break on key press
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
