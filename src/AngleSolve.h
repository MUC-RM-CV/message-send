#ifndef ANGLE_SOLVE_HPP
#define ANGLE_SOLVE_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

class AngleSolve {
private:
    //int aimOffsetX;
    //int aimOffsetY;
    //int cameraYawAngle;
    //int cameraPitchAngle;

    double GUN_CAM_DISTANCE_Y = 0;
    
    std::vector<cv::Point3d> BIG_ARMOR_POINTS_3D = {
            cv::Point3d(-230, -127, 0), // bl
            cv::Point3d(-230,  127, 0), // tl
            cv::Point3d( 230,  127, 0), // tr
            cv::Point3d( 230, -127, 0)  // br
    };
    cv::Mat cameraMatrix;       //IntrinsicMatrix
    cv::Mat distortionCoeffs;   //DistortionCoefficients

    double fx;
    double fy;
    double cx;
    double cy;

    static auto radToDeg(double rad) { return rad / CV_PI * 180; };

    auto pinHoleAngleSolve(cv::Point2f pos) {
        std::vector<cv::Point2f> in, out;
        in.push_back(pos);

        cv::undistortPoints(in, out, cameraMatrix, distortionCoeffs, cv::noArray(), cameraMatrix);
        auto pnt = out.front();

        auto rx = (pnt.x - cx) / fx;
        auto ry = (pnt.y - cy) / fy;

        auto yawAngle = radToDeg(atan(rx));
        auto pitchAngle = radToDeg(-atan(ry));

        return std::make_tuple(yawAngle, pitchAngle);
    }

public:
    AngleSolve() {
        // Only use these parameters for frame size @ '640 * 480'

        cameraMatrix = (cv::Mat_<double>(3,3) <<
                512.4496,   0,          320.94453,
                0,          511.079468, 243.58460494466,
                0,          0,          1);

        distortionCoeffs = (cv::Mat_<double>(1,5) <<
                -0.388116, 2.463448, 0.00759493, 0.0062624, -8.007717);

        fx = cameraMatrix.at<double>(0, 0);
        fy = cameraMatrix.at<double>(1, 1);
        cx = cameraMatrix.at<double>(0, 2);
        cy = cameraMatrix.at<double>(1, 2);
    }

    auto getAngle(std::vector<cv::Point2f> pts) {
        cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // init rVec
        cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // init tVec

        cv::solvePnP(BIG_ARMOR_POINTS_3D, pts, cameraMatrix, distortionCoeffs, rVec, tVec);

        auto x_pos = tVec.at<double>(0, 0);
        auto y_pos = tVec.at<double>(1, 0) - GUN_CAM_DISTANCE_Y;
        auto z_pos = tVec.at<double>(2, 0);
        auto distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);

        if (distance > 5000) {
            cv::Point2f pos;
            pos = (((pts[0] + pts[2]) / 2) + ((pts[1] + pts[3]) / 2)) / 2;
            return pinHoleAngleSolve(pos);
        } else {
            auto tan_pitch = - (y_pos / sqrt(x_pos*x_pos + z_pos * z_pos));
            auto tan_yaw = x_pos / z_pos;
            auto pitchAngle = radToDeg(tan_pitch);
            auto yawAngle = radToDeg(tan_yaw);
            return std::make_tuple(yawAngle, pitchAngle);
        }
    }
};

#endif // ANGLE_SOLVE_HPP
