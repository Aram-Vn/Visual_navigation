#include "../include/VideoProcessor.h"

namespace Air {

    VideoProcessor::VideoProcessor(const std::string& filePath, float altitude)
        : m_cap(filePath),
          m_total_X(0.0),
          m_total_Y(0.0),
          m_yav_res(0.0),
          m_abs_total_X(0.0),
          m_abs_total_Y(0.0),
          m_displacementThreshold(0.01),
          m_drone_mock_data(altitude)
    {
        if (!m_cap.isOpened())
        {
            throw std::runtime_error("Error: Could not open video file.");
        }

        m_cap >> m_prev_frame;
        cv::cvtColor(m_prev_frame, m_prev_frame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(m_prev_frame, m_points[0], 120, 0.01, 10);
    }

    VideoProcessor::VideoProcessor(int cameraID)
        : m_cap(cameraID, cv::CAP_V4L2),
          m_total_X(0.0),
          m_total_Y(0.0),
          m_yav_res(0.0),
          m_abs_total_X(0.0),
          m_abs_total_Y(0.0),
          m_displacementThreshold(0.01)
    {
        if (!m_cap.isOpened())
        {
            throw std::runtime_error("Error: Could not open camera.");
        }

        m_cap >> m_prev_frame;
        cv::cvtColor(m_prev_frame, m_prev_frame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(m_prev_frame, m_points[0], 120, 0.01, 10);
    }

    VideoProcessor::~VideoProcessor()
    {
        std::cout << "\n YAV_RES " << m_yav_res << std::endl;
        std::cout << "\n Altitude " << m_drone_mock_data.altitude << std::endl;
        std::cout << "\n m_abs_total_X: " << m_abs_total_X << std::endl;
        std::cout << "\n abs_total_Y__: " << m_abs_total_Y << std::endl;
        m_cap.release();
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

    void VideoProcessor::processFrames(void)
    {
        while (m_cap.read(m_curr_frame))
        {
            cv::Mat grayCurrFrame;
            cv::cvtColor(m_curr_frame, grayCurrFrame, cv::COLOR_BGR2GRAY);

            // Optical flow calculation
            std::vector<uchar> status;
            std::vector<float> err;

            // clang-format off
            cv::calcOpticalFlowPyrLK(m_prev_frame, grayCurrFrame, m_points[0], m_points[1], 
                    status,  err, cv::Size(15, 15), 3, 
                    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
            // clang-format on

            std::vector<cv::Point2f> inliersOld, inliersNew;
            for (size_t i = 0; i < m_points[1].size(); i++)
            {
                if (status[i])
                {
                    inliersOld.push_back(m_points[0][i]);
                    inliersNew.push_back(m_points[1][i]);
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

                m_yav_res += yaw_angle;
            }

            std::vector<float> totalDisplacementVec;
            std::vector<float> totalAngleVec;

            for (int i = 0; i < inliersMask.rows; i++)
            {
                if (inliersMask.at<uchar>(i))
                {
                    float dx = inliersNew[i].x - inliersOld[i].x;
                    float dy = inliersNew[i].y - inliersOld[i].y;

                    double displacement = std::hypot(dx, dy);
                    totalDisplacementVec.push_back(displacement);

                    if (displacement < m_displacementThreshold)
                        continue;

                    float angle = std::atan2(dy, dx);
                    totalAngleVec.push_back(angle);

                    // Draw green circle for tracked m_points
                    cv::circle(m_curr_frame, inliersNew[i], 3, cv::Scalar(0, 255, 0), -1);
                }
            }

            if (!totalDisplacementVec.empty())
            {
                float medianDisplacement = utilities::MathUtils::calculateMedian(totalDisplacementVec);
                float medianAngle        = utilities::MathUtils::calculateMedian(totalAngleVec);
                float medianAngleDegrees = medianAngle * (180.0 / CV_PI); // Convert angle to degrees

                std::cout << "Median Displacement: " << medianDisplacement << ", "
                          << "Median Angle: " << medianAngleDegrees << " degrees." << std::endl;

                float pixelSize_X = utilities::MathUtils::calculatePixelSize(
                    m_drone_mock_data.altitude, CameraParameters::Horizontal_FOV, CameraParameters::Resolution_width);

                float pixelSize_Y = utilities::MathUtils::calculatePixelSize(
                    m_drone_mock_data.altitude, CameraParameters::Vertical_FOV, CameraParameters::Resolution_height);

                float displacement_X = medianDisplacement * std::cos(medianAngle);
                float displacement_Y = medianDisplacement * std::sin(medianAngle);

                float displacementZ = 0.0f;

                // Create the displacement vector
                cv::Mat displacementVec = (cv::Mat_<float>(3, 1) << displacement_X, displacement_Y, displacementZ);

                cv::Mat rotationMatrix = utilities::RotationUtils::createRotationMatrix(
                    m_drone_mock_data.pitch, m_drone_mock_data.yaw, m_drone_mock_data.roll);

                cv::Mat transformedDisplacement = rotationMatrix * displacementVec;
                m_total_X += transformedDisplacement.at<float>(0) * pixelSize_X;
                m_total_Y += transformedDisplacement.at<float>(1) * pixelSize_Y;

                m_abs_total_X += std::abs(transformedDisplacement.at<float>(0) * pixelSize_X);
                m_abs_total_Y += std::abs(transformedDisplacement.at<float>(1) * pixelSize_Y);

                // m_total_X += displacement_X * pixelSize_X;
                // m_total_Y += displacement_Y * pixelSize_Y;

                // m_abs_total_X += std::abs(displacement_X * pixelSize_X);
                // m_abs_total_Y += std::abs(displacement_Y * pixelSize_Y);

                std::cout << "Real-world m_total_X in X: " << m_total_X << " m" << std::endl;
                std::cout << "Real-world m_total_Y in Y: " << m_total_Y << " m" << std::endl;
            }

            displayResults(m_curr_frame);
            m_points[0]  = m_points[1];
            m_prev_frame = std::move(grayCurrFrame);

            cv::goodFeaturesToTrack(m_prev_frame, m_points[0], 120, 0.01, 5);
        }
    }
} // namespace Air