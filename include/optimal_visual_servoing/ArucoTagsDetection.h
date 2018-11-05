/*
  Optimal Visual Servoing
  Copyright (C) 2018  Siddharth Jha, Aashay Bhise

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

struct PoseEstimationParams {
    PoseEstimationParams() {
        K.at<double> ( 0,0 ) = fx;
        K.at<double> ( 0,2 ) = cx;
        K.at<double> ( 1,1 ) = fy;
        K.at<double> ( 1,2 ) = cy;
    }
    double fx = 300;
    double fy = 300;
    double cx = 160;
    double cy = 120;
    cv::Mat K = cv::Mat ( 3, 3, CV_64F );
    double marker_length = 0.1;
};


class ArucoTagsDetection
{
private:
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    PoseEstimationParams pose_estimation_params_;

public:
    ArucoTagsDetection();
    ~ArucoTagsDetection();
    void generateArucoTag ( std::string output_file_marker, std::string output_file_charuco );
    void detectArucoTags ( cv::Mat &img, Eigen::Vector4d &marker_point, Eigen::Vector2d &marker_projection );
    void readDetectorParameters ( std::string filename );
};
