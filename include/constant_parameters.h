#ifndef AIRDEV_VISUAL_NAVIGATION_INCLUDE_CONSTANT_PARAMETERS_H
#define AIRDEV_VISUAL_NAVIGATION_INCLUDE_CONSTANT_PARAMETERS_H

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

struct droneMockData
{
    float altitude; // meters
    float pitch;    // degrees
    float yaw;      // degrees
    float roll;     // degrees

    droneMockData(float alt = 150.0f, float p = 0.0f, float y = 0.0f, float r = 0.0f)
        : altitude(alt),
          pitch(p),
          yaw(y),
          roll(r)
    {
    }
};

namespace CameraParameters {

    constexpr int   Resolution_width  = 640;
    constexpr int   Resolution_height = 480;
    constexpr float Horizontal_FOV    = 62.2f; // degrees
    constexpr float Vertical_FOV      = 48.8f; // degrees

    constexpr float Principal_point_width  = 1.53; // mm;
    constexpr float Principal_point_height = 1.15; // mm;

    constexpr float focal_length_in_mm = 3.04;    // mm
    constexpr float pixel_size         = 0.00112; // mm per pixel

    constexpr float sensor_width_in_mm  = Resolution_width * pixel_size;  // mm
    constexpr float sensor_height_in_mm = Resolution_height * pixel_size; // mm

    // Camera intrinsic parameters
    constexpr float fx = (focal_length_in_mm * Resolution_width) / sensor_width_in_mm;       // Focal length in pixels
                                                                                             // (X-axis)
    constexpr float fy = (focal_length_in_mm * Resolution_height) / sensor_height_in_mm;     // Focal length in pixels
                                                                                             // (Y-axis)
    constexpr float cx = (Principal_point_width * Resolution_width) / sensor_width_in_mm;    // Principal point (X-axis)
    constexpr float cy = (Principal_point_height * Resolution_height) / sensor_height_in_mm; // Principal point (Y-axis)

    // clang-format off
    const cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0, cx,
                                                                      0, fy, cy,
                                                                      0,  0, 1);
    // clang-format on

} // namespace CameraParameters

#endif // AIRDEV_VISUAL_NAVIGATION_INCLUDE_CONSTANT_PARAMETERS_H