#include "../include/utilities.h"

namespace utilities {

    //*************************** RotationUtils ***************************//

    double RotationUtils::getYawFromRotationMatrix(const cv::Mat& Mat)
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
    cv::Mat RotationUtils::createRotationMatrix(float pitch_val, float yaw_val, float roll_val)
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

    //*********************** MathUtils ***********************//

    // Function to calculate the median of a vector
    float MathUtils::calculateMedian(std::vector<float>& vec)
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

    //*************************** PointUtils ***************************//

    std::vector<cv::Point2f> PointUtils::filterPointsNearCenter(const std::vector<cv::Point2f>& m_points,
                                                                const cv::Size& frameSize, float maxDistance)
    {
        std::vector<cv::Point2f> filteredPoints;
        cv::Point2f              center(frameSize.width / 2.0f,
                                        frameSize.height / 2.0f); // Center of the frame

        for (const auto& point : m_points)
        {
            float distance = cv::norm(point - center);

            if (distance < maxDistance)
            {
                filteredPoints.push_back(point);
            }
        }

        return filteredPoints;
    }

} // namespace utilities