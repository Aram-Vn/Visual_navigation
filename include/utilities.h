#ifndef AIRDEV_VISUAL_NAVIGATION_INCLUDE_UTILITIES_H
#define AIRDEV_VISUAL_NAVIGATION_INCLUDE_UTILITIES_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace utilities {
    class RotationUtils
    {
    public:
        static double  getYawFromRotationMatrix(const cv::Mat& mat);
        static cv::Mat createRotationMatrix(float pitch_val, float yaw_val, float roll_val);
    };

    class MathUtils
    {
    public:
        static float           calculateMedian(std::vector<float>& vec);
        static constexpr float calculatePixelSize(float altitude, float fov, int resolution)
        {
            return (2.0f * altitude * std::tan(fov * (M_PI / 180.0f) / 2.0f)) / resolution;
        }
    };

    class PointUtils
    {
    public:
        static std::vector<cv::Point2f> filterPointsNearCenter(const std::vector<cv::Point2f>& m_points,
                                                               const cv::Size& frameSize, float maxDistance);
    };
} // namespace utilities

#endif // AIRDEV_VISUAL_NAVIGATION_INCLUDE_UTILITIES_H
