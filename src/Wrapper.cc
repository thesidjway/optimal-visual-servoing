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

#include <optimal_visual_servoing/OptimizationProblem.h>
#include <optimal_visual_servoing/DynamicWindowSampler.h>
#include <optimal_visual_servoing/ArucoTagsDetection.h>
#include <optimal_visual_servoing/LaserSegmentation.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>


void pointCloudCallback ( const sensor_msgs::PointCloud2ConstPtr& callback_cloud ) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( *callback_cloud,pcl_pc2 );
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pcl ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromPCLPointCloud2 ( pcl_pc2,*raw_pcl );
    return;
}

void imageCallback ( const sensor_msgs::ImageConstPtr& callback_image ) {
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


int main ( int argc, char **argv ) {
    OptimizationProblem opt_problem;
    DynamicWindowSampler dws;
    ArucoTagsDetection detector;
    Eigen::Vector3d pt;
    Eigen::Vector2d proj;
    cv::Mat b = cv::imread ( "/home/thesidjway/deprecated_stuff/markers-depth-estimator/data/image-test.png" );
    detector.detectArucoTags ( b , pt, proj );
    RobotState a = RobotState ( 0, 0, 1.0, 0.2, 0.6 );
    std::vector<RobotState> feasible_states;
    dws.getFeasibleSearchSpace ( a, feasible_states );
    std::vector<RangeDataTuple> gen_data;
    opt_problem.generateData ( gen_data );
    for ( unsigned int i = 0 ; i < gen_data.size() ; i++ ) {
        opt_problem.addRangeFactor ( gen_data[i] );
    }
    opt_problem.optimizeGraph();

    ros::init ( argc, argv, "optimal_visual_servoing" );
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe ( "/scan", 10, pointCloudCallback ); // laser to point cloud data
    ros::Subscriber image_sub = nh.subscribe ( "/image_raw", 10, imageCallback ); // laser to point cloud data
    while ( ros::ok() ) {
        ros::spinOnce();
    }

}
