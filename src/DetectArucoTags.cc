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

  
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    readDetectorParameters("/home/thesidjway/experimental_ws/src/markers-depth-estimator/data/detector_params.yml", detectorParams);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Mat inputImage = imread("/home/thesidjway/experimental_ws/src/markers-depth-estimator/data/image-test.png");
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(16);
    cv::aruco::detectMarkers(inputImage, 
			     dictionary, 
			     markerCorners, 
			     markerIds,
			     detectorParams,
			     noArray()
			    );
