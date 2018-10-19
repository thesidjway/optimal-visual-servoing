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
    readDetectorParameters ( "/home/thesidjway/experimental_ws/src/markers-depth-estimator/data/detector_params.yml" );

}

ArucoTagsDetection::~ArucoTagsDetection() {

}

bool ArucoTagsDetection::readDetectorParameters ( std::string filename ) {
    cv::FileStorage fs ( filename, cv::FileStorage::READ );
    if ( !fs.isOpened() ) {
        return false;
    }
    fs["adaptiveThreshWinSizeMin"] >> params_->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params_->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params_->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params_->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params_->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params_->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params_->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params_->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params_->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params_->minMarkerDistanceRate;
    fs["cornerRefinementWinSize"] >> params_->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params_->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params_->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params_->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params_->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params_->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params_->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params_->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params_->errorCorrectionRate;
    return true;
}

void ArucoTagsDetection::detectArucoTags ( cv::Mat& img ) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Mat inputImage = cv::imread ( "/home/thesidjway/experimental_ws/src/markers-depth-estimator/data/image-test.png" );
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary ( 16 );
    cv::aruco::detectMarkers ( inputImage,
                               dictionary,
                               markerCorners,
                               markerIds,
                               params_,
                               cv::noArray() );
}

