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
    readDetectorParameters ( "/home/thesidjway/deprecated_stuff/markers-depth-estimator/data/detector_params.yml" );

}

ArucoTagsDetection::~ArucoTagsDetection() {

}

bool ArucoTagsDetection::readDetectorParameters ( std::string filename ) {
    cv::FileStorage fs ( filename, cv::FileStorage::READ );
    if ( !fs.isOpened() ) {
        return false;
    }
    fs["adaptiveThreshWinSizeMin"] >> detector_params_->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> detector_params_->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> detector_params_->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> detector_params_->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> detector_params_->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> detector_params_->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> detector_params_->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> detector_params_->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> detector_params_->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> detector_params_->minMarkerDistanceRate;
    fs["cornerRefinementWinSize"] >> detector_params_->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> detector_params_->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> detector_params_->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> detector_params_->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> detector_params_->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> detector_params_->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> detector_params_->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> detector_params_->minOtsuStdDev;
    fs["errorCorrectionRate"] >> detector_params_->errorCorrectionRate;
    return true;
}



void ArucoTagsDetection::detectArucoTags ( cv::Mat &img, Eigen::Vector3d &marker_point, Eigen::Vector2d &marker_projection ) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary ( 16 );
    cv::aruco::detectMarkers ( img,
                               dictionary,
                               markerCorners,
                               markerIds,
                               detector_params_,
                               cv::noArray() );
    cv::Mat r_marker, t_marker;
    cv::aruco::estimatePoseSingleMarkers ( markerCorners, pose_estimation_params_.marker_length, pose_estimation_params_.K, cv::noArray(), r_marker, t_marker );
    std::cout << r_marker << std::endl;
    std::cout << t_marker << std::endl;
    marker_point = Eigen::Vector3d ( t_marker.at<double> ( 0,0 ), t_marker.at<double> ( 1,0 ), t_marker.at<double> ( 2,0 ) );
    marker_projection ( 0 ) = ( markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x ) / 4 ;
    marker_projection ( 1 ) = ( markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y ) / 4 ;
}

