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

OVSWrapper::OVSWrapper ( std:: string params_file ) {
    readWrapperParams ( params_file );
    initializeRosPipeline();
    cluster_extractor_.readClusteringParams ( params_file );
    opt_problem_.readOptimizationParams ( params_file );
    detector_.readDetectorParameters ( params_file );
}

OVSWrapper::~OVSWrapper() {

}

void OVSWrapper::readWrapperParams ( std:: string params_file ) {
    YAML::Node config = YAML::LoadFile ( params_file );
    wrapper_params_.K = cv::Mat::eye ( 3 , 3 , CV_32F );
    wrapper_params_.dist = cv::Mat::zeros ( 1 , 4 , CV_64F );

    wrapper_params_.pc2dTopic = config["ros"]["pc_2d_topic"].as<std::string>();
    wrapper_params_.pc3dTopic = config["ros"]["pc_3d_topic"].as<std::string>();
    wrapper_params_.imageTopic = config["ros"]["image_topic"].as<std::string>();
    wrapper_params_.ptzTopic = config["ros"]["ptz_topic"].as<std::string>();

    wrapper_params_.fx = config["camera"]["fx"].as<double>();
    wrapper_params_.fy = config["camera"]["fy"].as<double>();
    wrapper_params_.cx = config["camera"]["cx"].as<double>();
    wrapper_params_.cy = config["camera"]["cy"].as<double>();
    wrapper_params_.k1 = config["camera"]["k1"].as<double>();
    wrapper_params_.k2 = config["camera"]["k2"].as<double>();
    wrapper_params_.p1 = config["camera"]["p1"].as<double>();
    wrapper_params_.p2 = config["camera"]["p2"].as<double>();
    wrapper_params_.K.at<float> ( 0, 0 ) = wrapper_params_.fx;
    wrapper_params_.K.at<float> ( 0, 2 ) = wrapper_params_.cx;
    wrapper_params_.K.at<float> ( 1, 1 ) = wrapper_params_.fy;
    wrapper_params_.K.at<float> ( 1, 2 ) = wrapper_params_.cy;
    wrapper_params_.dist.at<double> ( 0, 0 ) = wrapper_params_.k1;
    wrapper_params_.dist.at<double> ( 0, 1 ) = wrapper_params_.k2;
    wrapper_params_.dist.at<double> ( 0, 2 ) = wrapper_params_.p1;
    wrapper_params_.dist.at<double> ( 0, 3 ) = wrapper_params_.p2;
}

void OVSWrapper::pointCloudCallback2D ( const sensor_msgs::LaserScanConstPtr& laser_scan ) {
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
    last_data_clusters_ = segments;
}

void OVSWrapper::pointCloudCallback3D ( const sensor_msgs::PointCloud2ConstPtr& cloud_msg ) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( *cloud_msg, pcl_pc2 );
    cluster_extractor_.setInputCloud ( pcl_pc2 );
    cluster_extractor_.segmentPointcloud();
    std::vector<RangeDataTuple> segments;
    cluster_extractor_.extractSegmentFeatures ( segments );
    last_data_clusters_ = segments;
}

void OVSWrapper::imageCallback ( const sensor_msgs::CompressedImageConstPtr& image_msg ) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy ( image_msg, sensor_msgs::image_encodings::BGR8 );
        cv::Mat read_image_distorted = cv_ptr->image;
        cv::cvtColor ( read_image_distorted, read_image_distorted, CV_BGR2GRAY );
        cv::Mat read_image;
        //cv::undistort ( read_image_distorted, read_image, wrapper_params_.K, wrapper_params_.dist );
        read_image = read_image_distorted;
        cv::imshow ( "Read Image", read_image );
        Eigen::Vector3d pt;
        Eigen::Vector2d proj;
        detector_.detectArucoTags ( read_image, pt, proj );
        cv::waitKey ( 1 );
    } catch ( cv_bridge::Exception &e ) {
        ROS_ERROR ( "cv_bridge exception: %s", e.what() );
    }
}

void OVSWrapper::initializeRosPipeline() {
    pc3d_sub_ = n_.subscribe ( wrapper_params_.pc3dTopic, 10, &OVSWrapper::pointCloudCallback3D, this );
    pc2d_sub_ = n_.subscribe ( wrapper_params_.pc2dTopic, 10, &OVSWrapper::pointCloudCallback2D, this ); // laser to point cloud data
    image_sub_ = n_.subscribe ( wrapper_params_.imageTopic, 10, &OVSWrapper::imageCallback, this ); // image data
    ptz_pub_ = n_.advertise<axis_camera::Axis> ( wrapper_params_.ptzTopic, 1000 );
}



int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "optimal_visual_servoing" );
    OVSWrapper wrapper ( "/home/thesidjway/research_ws/src/optimal-visual-servoing/params/optimal_visual_servoing.yaml" );
//     DynamicWindowSampler dws;

//     Eigen::Vector3d pt;
//     Eigen::Vector2d proj;
//     cv::Mat b = cv::imread ( "/home/thesidjway/deprecated_stuff/markers-depth-estimator/data/image-test.png" );
//     detector.detectArucoTags ( b , pt, proj );
//     RobotState a = RobotState ( 0, 0, 1.0, 0.2, 0.6 );
//     std::vector<RobotState> feasible_states;
//     dws.getFeasibleSearchSpace ( a, feasible_states );

    while ( ros::ok() ) {
        if ( wrapper.last_data_clusters_.size() > 0 ) {

            for ( unsigned int i = 0 ; i < wrapper.last_data_clusters_.size() ; i++ ) {
                wrapper.opt_problem_.addRangeFactor ( wrapper.last_data_clusters_[i] );
            }
            wrapper.opt_problem_.optimizeGraph();
            PTZCommand cmd = wrapper.opt_problem_.getPTZCommand();
            axis_camera::Axis ptz_msg;
            ptz_msg.pan = cmd.pan;
            ptz_msg.tilt = cmd.tilt;
            ptz_msg.zoom = cmd.zoom;
            wrapper.ptz_pub_.publish ( ptz_msg );
            wrapper.last_data_clusters_.clear();
        }
        ros::spinOnce();
    }
}
