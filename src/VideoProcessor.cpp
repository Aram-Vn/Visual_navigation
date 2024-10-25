#include "../include/VideoProcessor.h"

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
        std::cout << "\n abs_total_X: " << abs_total_X << std::endl;
        std::cout << "\n abs_total_y: " << abs_total_y << std::endl;
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
                float medianDisplacement = utilities::MathUtils::calculateMedian(totalDisplacementVec);
                float medianAngle        = utilities::MathUtils::calculateMedian(totalAngleVec);
                float medianAngleDegrees = medianAngle * (180.0 / CV_PI); // Convert angle to degrees

                std::cout << "Median Displacement: " << medianDisplacement << ", "
                          << "Median Angle: " << medianAngleDegrees << " degrees." << std::endl;

                float pixelSizeX = utilities::MathUtils::calculatePixelSize(
                    droneMockData::Altitude, CameraParameters::Horizontal_FOV, CameraParameters::Resolution_width);

                float pixelSizeY = utilities::MathUtils::calculatePixelSize(
                    droneMockData::Altitude, CameraParameters::Vertical_FOV, CameraParameters::Resolution_height);

                float displacementX = medianDisplacement * std::cos(medianAngle);
                float displacementY = medianDisplacement * std::sin(medianAngle);

                float displacementZ = 0.0f;

                // Create the displacement vector
                cv::Mat displacementVec = (cv::Mat_<float>(3, 1) << displacementX, displacementY, displacementZ);

                cv::Mat rotationMatrix = utilities::RotationUtils::createRotationMatrix(
                    droneMockData::pitch, droneMockData::yaw, droneMockData::roll);

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