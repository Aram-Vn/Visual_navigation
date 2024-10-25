#include "../include/VideoProcessor.h"
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
namespace Air {

    VideoProcessor::VideoProcessor(const std::string& filePath)
        : cap(filePath),
          total_X(0.0),
          total_Y(0.0),
          yavRes(0.0),
          abs_total_X(0.0),
          abs_total_y(0.0)
    {
        if (!cap.isOpened())
        {
            throw std::runtime_error("Error: Could not open video file.");
        }

        cap >> prevFrame;
        cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(prevFrame, points[0], 120, 0.01, 10);
    }

    VideoProcessor::VideoProcessor(int cameraID)
        : cap(cameraID, cv::CAP_V4L2),
          total_X(0.0),
          total_Y(0.0),
          yavRes(0.0),
          abs_total_X(0.0),
          abs_total_y(0.0)
    {
        if (!cap.isOpened())
        {
            throw std::runtime_error("Error: Could not open camera.");
        }

        cap >> prevFrame;
        cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(prevFrame, points[0], 120, 0.01, 10);
    }

    VideoProcessor::~VideoProcessor()
    {
        std::cout << "\n YAV_RES " << yavRes << std::endl;
        std::cout << "\n Altitude " << droneMockData::Altitude << std::endl;
        std::cout << "\nabs_total_X: " << abs_total_X << std::endl;
        std::cout << "\nabs_total_y: " << abs_total_y << std::endl;
        cap.release();
        cv::destroyAllWindows();
    }

    void VideoProcessor::displayResults(const cv::Mat& frame)
    {
        cv::imshow("Tracked Points", frame);
        if (cv::waitKey(30) >= 0)
        {
            return;
        }
    }

    void VideoProcessor::processFrames()
    {
        while (cap.read(currFrame))
        {
            cv::Mat grayCurrFrame;
            cv::cvtColor(currFrame, grayCurrFrame, cv::COLOR_BGR2GRAY);

            // Optical flow calculation
            std::vector<uchar> status;
            std::vector<float> err;

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

                yavRes += yaw_angle;
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
                    cv::circle(grayCurrFrame, inliersNew[i], 3, cv::Scalar(0, 255, 0), -1);
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

            displayResults(grayCurrFrame);
            points[0] = points[1];
            prevFrame = std::move(grayCurrFrame);

            cv::goodFeaturesToTrack(prevFrame, points[0], 120, 0.01, 5);
        }
    }
} // namespace Air