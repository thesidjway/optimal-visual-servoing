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

#include <optimal_visual_servoing/Wrapper.h>

OVSWrapper::OVSWrapper() {
    initializeRosPipeline();
}

OVSWrapper::~OVSWrapper() {

}

void OVSWrapper::pointCloudCallback ( const sensor_msgs::PointCloud2ConstPtr& cloud_msg ) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( *cloud_msg, pcl_pc2 );
    cluster_extractor_.setInputCloud ( pcl_pc2 );
    cluster_extractor_.segmentPointcloud();
    std::vector<RangeDataTuple> segments;
    cluster_extractor_.extractSegmentFeatures ( segments );
    last_data_clusters_ = segments;

    return;
}

void OVSWrapper::imageCallback ( const sensor_msgs::ImageConstPtr& callback_image ) {
    try {
        cv::Mat read_image_distorted = cv_bridge::toCvShare ( callback_image, "bgr8" )->image;
        cv::cvtColor ( read_image_distorted, read_image_distorted, CV_BGR2GRAY );
        cv::Mat read_image;
        read_image = read_image_distorted;
        cv::imshow ( "Read Image", read_image );
        cv::waitKey ( 1 );
    } catch ( cv_bridge::Exception &e ) {
        ROS_ERROR ( "cv_bridge exception: %s", e.what() );
    }
}

void OVSWrapper::initializeRosPipeline() {
    pc_sub_ = n_.subscribe ( "/velodyne_points", 10, &OVSWrapper::pointCloudCallback, this ); // laser to point cloud data
    image_sub_ = n_.subscribe ( "/cam0/image_raw", 10, &OVSWrapper::imageCallback, this ); // image data
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "optimal_visual_servoing" );
    OVSWrapper wrapper;

//     DynamicWindowSampler dws;
//     ArucoTagsDetection detector;
//     Eigen::Vector3d pt;
//     Eigen::Vector2d proj;
//     cv::Mat b = cv::imread ( "/home/thesidjway/deprecated_stuff/markers-depth-estimator/data/image-test.png" );
//     detector.detectArucoTags ( b , pt, proj );
//     RobotState a = RobotState ( 0, 0, 1.0, 0.2, 0.6 );
//     std::vector<RobotState> feasible_states;
//     dws.getFeasibleSearchSpace ( a, feasible_states );

    while ( ros::ok() ) {
        if ( wrapper.last_data_clusters_.size() > 0 ) {
            OptimizationProblem opt_problem;
            for ( unsigned int i = 0 ; i < wrapper.last_data_clusters_.size() ; i++ ) {
                opt_problem.addRangeFactor ( wrapper.last_data_clusters_[i] );
            }
            opt_problem.optimizeGraph();
            wrapper.last_data_clusters_.clear();
        }

        ros::spinOnce();
    }

}
