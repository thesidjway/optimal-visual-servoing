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

void OVSWrapper::pointCloudCallback ( const sensor_msgs::LaserScanConstPtr& laser_scan ) {
    
    sensor_msgs::PointCloud2 callback_cloud;
    laser_geometry::LaserProjection lp;
    lp.projectLaser ( *laser_scan, callback_cloud );
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( callback_cloud, pcl_pc2 );
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pcl ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromPCLPointCloud2 ( pcl_pc2,*raw_pcl );
    cluster_extractor_.setInputCloud ( raw_pcl );
    cluster_extractor_.segmentPointcloud();
    std::vector<RangeDataTuple> segments;
    cluster_extractor_.extractSegmentFeatures ( segments );
    
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
    pc_sub_ = n_.subscribe ( "summit_xl_a/front_laser/scan", 10, &OVSWrapper::pointCloudCallback, this ); // laser to point cloud data
    image_sub_ = n_.subscribe ( "/cam0/image_raw", 10, &OVSWrapper::imageCallback, this ); // image data
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "optimal_visual_servoing" );
    OVSWrapper wrapper;
//     OptimizationProblem opt_problem;
//     DynamicWindowSampler dws;
//     ArucoTagsDetection detector;
//     Eigen::Vector3d pt;
//     Eigen::Vector2d proj;
//     cv::Mat b = cv::imread ( "/home/thesidjway/deprecated_stuff/markers-depth-estimator/data/image-test.png" );
//     detector.detectArucoTags ( b , pt, proj );
//     RobotState a = RobotState ( 0, 0, 1.0, 0.2, 0.6 );
//     std::vector<RobotState> feasible_states;
//     dws.getFeasibleSearchSpace ( a, feasible_states );
//     std::vector<RangeDataTuple> gen_data;
//     opt_problem.generateData ( gen_data );
//     for ( unsigned int i = 0 ; i < gen_data.size() ; i++ ) {
//         opt_problem.addRangeFactor ( gen_data[i] );
//     }
//     opt_problem.optimizeGraph();

    while ( ros::ok() ) {
        ros::spinOnce();
    }

}
