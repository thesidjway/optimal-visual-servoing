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

#include <optimal_visual_servoing/ArucoTagsDetection.h>

ArucoTagsDetection::ArucoTagsDetection() {
    detector_params_ = cv::aruco::DetectorParameters::create();
}

ArucoTagsDetection::~ArucoTagsDetection() {

}

void ArucoTagsDetection::generateArucoTag ( std::string output_file_marker, std::string output_file_charuco ) {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary ( 8 ); //DICT_6X6_50
    cv::Mat markerImage;
    cv::aruco::drawMarker ( dictionary, 23, 2000, markerImage, 1 );
    cv::imwrite ( output_file_marker, markerImage );
    cv::Ptr<cv::aruco::CharucoBoard> a = cv::aruco::CharucoBoard::create ( 7,
                                         7,
                                         0.1,
                                         0.07,
                                         dictionary );
    a->draw ( cv::Size ( 2000, 2000 ),
              markerImage,
              0,
              1 );
    cv::imwrite ( output_file_charuco, markerImage );
}

void ArucoTagsDetection::readDetectorParameters ( std::string params_file ) {
    YAML::Node config = YAML::LoadFile ( params_file );
    detector_params_->adaptiveThreshWinSizeMin = config["aruco"]["adaptiveThreshWinSizeMin"].as<double>();
    detector_params_->adaptiveThreshWinSizeMax = config["aruco"]["adaptiveThreshWinSizeMax"].as<double>();
    detector_params_->adaptiveThreshWinSizeStep = config["aruco"]["adaptiveThreshWinSizeStep"].as<double>();
    detector_params_->adaptiveThreshConstant = config["aruco"]["adaptiveThreshConstant"].as<double>();
    detector_params_->minMarkerPerimeterRate = config["aruco"]["minMarkerPerimeterRate"].as<double>();
    detector_params_->maxMarkerPerimeterRate = config["aruco"]["maxMarkerPerimeterRate"].as<double>();
    detector_params_->polygonalApproxAccuracyRate = config["aruco"]["polygonalApproxAccuracyRate"].as<double>();
    detector_params_->minCornerDistanceRate = config["aruco"]["minCornerDistanceRate"].as<double>();
    detector_params_->minDistanceToBorder = config["aruco"]["minDistanceToBorder"].as<double>();
    detector_params_->minMarkerDistanceRate = config["aruco"]["minMarkerDistanceRate"].as<double>();
    detector_params_->cornerRefinementWinSize = config["aruco"]["cornerRefinementWinSize"].as<double>();
    detector_params_->cornerRefinementMaxIterations = config["aruco"]["cornerRefinementMaxIterations"].as<double>();
    detector_params_->cornerRefinementMinAccuracy = config["aruco"]["cornerRefinementMinAccuracy"].as<double>();
    detector_params_->markerBorderBits = config["aruco"]["markerBorderBits"].as<double>();
    detector_params_->perspectiveRemovePixelPerCell = config["aruco"]["perspectiveRemovePixelPerCell"].as<double>();
    detector_params_->perspectiveRemoveIgnoredMarginPerCell = config["aruco"]["perspectiveRemoveIgnoredMarginPerCell"].as<double>();
    detector_params_->maxErroneousBitsInBorderRate = config["aruco"]["maxErroneousBitsInBorderRate"].as<double>();
    detector_params_->minOtsuStdDev = config["aruco"]["minOtsuStdDev"].as<double>();
    detector_params_->errorCorrectionRate = config["aruco"]["errorCorrectionRate"].as<double>();
    pose_estimation_params_.fx = config["camera"]["fx"].as<double>();
    pose_estimation_params_.fy = config["camera"]["fy"].as<double>();
    pose_estimation_params_.cx = config["camera"]["cx"].as<double>();
    pose_estimation_params_.cy = config["camera"]["cy"].as<double>();
    pose_estimation_params_.marker_length = config["aruco"]["markerLength"].as<double>();
}



void ArucoTagsDetection::detectArucoTags ( cv::Mat &img, Eigen::Vector4d &marker_point, Eigen::Vector2d &marker_projection ) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary ( 8 ); //DICT_6X6_50
    cv::aruco::detectMarkers ( img,
                               dictionary,
                               markerCorners,
                               markerIds,
                               detector_params_,
                               cv::noArray() );
    cv::Mat r_marker, t_marker;
    cv::aruco::estimatePoseSingleMarkers ( markerCorners, pose_estimation_params_.marker_length, pose_estimation_params_.K, cv::noArray(), r_marker, t_marker );
    if ( t_marker.rows > 0 ) {
        marker_point = Eigen::Vector4d ( t_marker.at<double> ( 0,0 ) / t_marker.at<double> ( 2,0 ), t_marker.at<double> ( 1,0 ) / t_marker.at<double> ( 2,0 ), 1.0, 1.0 );
        marker_projection ( 0 ) = ( markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x ) / 4 ;
        marker_projection ( 1 ) = ( markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y ) / 4 ;
    }
}

