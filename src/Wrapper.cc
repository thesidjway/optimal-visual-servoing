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

OVSWrapper::OVSWrapper ( std:: string params_file, std::string aruco_params_file ) {
    readWrapperParams ( params_file );
    initializeRosPipeline();
    cluster_extractor_.readClusteringParams ( params_file );
    opt_problem_.readOptimizationParams ( params_file );
    detector_.readDetectorParameters ( aruco_params_file );
    Tbody_in_world_ = Eigen::MatrixXd::Random ( 4, 4 );
    Ttag_in_world_ = Eigen::MatrixXd::Random ( 4, 4 );
//     pt_for_optimization_ = Eigen::Vector4d ( 0, 0, 5, 0 );


}

OVSWrapper::~OVSWrapper() {
}

void OVSWrapper::publishPanAndTilt ( PTZCommand cmd ) {
    if ( wrapper_params_.robotType == "husky" ) {
        axis_camera::Axis ptz_msg;
        ptz_msg.pan = cmd.pan;
        ptz_msg.tilt = cmd.tilt;
        ptz_pub_.publish ( ptz_msg );
    } else if ( wrapper_params_.robotType == "summit_xl" ) {
        std_msgs::Float64 pan_msg, tilt_msg;
        pan_msg.data = cmd.pan;
        tilt_msg.data = cmd.tilt;
        pan_pub_.publish ( pan_msg );
        tilt_pub_.publish ( tilt_msg );
    }
}

void OVSWrapper::publishCommandVel ( double vx, double w ) {
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = vx;
    cmd_vel_msg.angular.z = w;
    cmd_vel_pub_.publish ( cmd_vel_msg );
}

void OVSWrapper::publishPosition ( double x, double y, double theta ) {
    gazebo_msgs::ModelState state;
    state.model_name = wrapper_params_.vehicleName;
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = 0;
    double cy = cos ( theta * 0.5 );
    double sy = sin ( theta * 0.5 );
    double cr = 1;
    double sr = 0;
    double cp = 1;
    double sp = 0;
    double qw = cy * cr * cp + sy * sr * sp;
    double qx = cy * sr * cp - sy * cr * sp;
    double qy = cy * cr * sp + sy * sr * cp;
    double qz = sy * cr * cp - cy * sr * sp;
    state.pose.orientation.x = qx;
    state.pose.orientation.y = qy;
    state.pose.orientation.z = qz;
    state.pose.orientation.w = qw;
    state.reference_frame = wrapper_params_.referenceFrame;
    trajectory_pub_.publish ( state );
}


void OVSWrapper::readWrapperParams ( std:: string params_file ) {
    YAML::Node config = YAML::LoadFile ( params_file );
    wrapper_params_.K = cv::Mat::eye ( 3 , 3 , CV_32F );
    wrapper_params_.dist = cv::Mat::zeros ( 1 , 4 , CV_64F );

    wrapper_params_.pc2dTopic = config["ros"]["pc_2d_topic"].as<std::string>();
    wrapper_params_.pc3dTopic = config["ros"]["pc_3d_topic"].as<std::string>();
    wrapper_params_.imageTopic = config["ros"]["image_topic"].as<std::string>();
    wrapper_params_.ptzTopic = config["ros"]["ptz_topic"].as<std::string>();
    wrapper_params_.panTopic = config["ros"]["pan_topic"].as<std::string>();
    wrapper_params_.tiltTopic = config["ros"]["tilt_topic"].as<std::string>();
    wrapper_params_.cmdVelTopic = config["ros"]["cmd_vel_topic"].as<std::string>();
    wrapper_params_.robotType = config["ros"]["robot_type"].as<std::string>();
    wrapper_params_.gtTopic = config["ros"]["gt_topic"].as<std::string>();
    wrapper_params_.tagTopic = config["ros"]["tag_topic"].as<std::string>();
    wrapper_params_.publishXYTopic = config["ros"]["publish_xy_topic"].as<std::string>();
    wrapper_params_.vehicleName = config["ros"]["vehicle_name"].as<std::string>();
    wrapper_params_.referenceFrame = config["ros"]["reference_frame"].as<std::string>();

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
    pcl::fromPCLPointCloud2 ( pcl_pc2, *raw_pcl );
    cluster_extractor_.setInputCloud ( raw_pcl );
    cluster_extractor_.segmentPointcloud();
    std::vector<RangeDataTuple> segments;
    std::vector<LineSegmentDataTuple> line_segments;
    cluster_extractor_.extractSegmentFeatures ( segments, line_segments );
    last_data_clusters_ = segments;
    last_line_segments_ = line_segments;
}

void OVSWrapper::pointCloudCallback3D ( const sensor_msgs::PointCloud2ConstPtr& cloud_msg ) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( *cloud_msg, pcl_pc2 );
    cluster_extractor_.setInputCloud ( pcl_pc2 );
    cluster_extractor_.segmentPointcloud();
    std::vector<RangeDataTuple> segments;
    std::vector<LineSegmentDataTuple> line_segments;
    cluster_extractor_.extractSegmentFeatures ( segments, line_segments );
    last_data_clusters_ = segments;
}

void OVSWrapper::gtCallback ( const gazebo_msgs::LinkStatesConstPtr& gt_msg ) {

    double qx = gt_msg->pose[2].orientation.x;
    double qy = gt_msg->pose[2].orientation.y;
    double qz = gt_msg->pose[2].orientation.z;
    double qw = gt_msg->pose[2].orientation.w;
    double x = gt_msg->pose[2].position.x;
    double y = gt_msg->pose[2].position.y;
    double z = gt_msg->pose[2].position.z;
    double vx = gt_msg->twist[2].linear.x;
    double vy = gt_msg->twist[2].linear.y;
    double wz = gt_msg->twist[2].angular.z;


    tf::Quaternion q (
        qx,
        qy,
        qz,
        qw );
    tf::Matrix3x3 m ( q );
    double roll, pitch, yaw;
    m.getRPY ( roll, pitch, yaw );
    last_gt_ = Eigen::Vector3d ( x , y , yaw );
    last_vels_ = Eigen::Vector3d ( vx , vy , wz );
    double body_in_world[16];
    body_in_world[0] = m[0][0];
    body_in_world[1] = m[0][1];
    body_in_world[2] = m[0][2];
    body_in_world[3] = x;
    body_in_world[4] = m[1][0];
    body_in_world[5] = m[1][1];
    body_in_world[6] = m[1][2];
    body_in_world[7] = y;
    body_in_world[8] = m[2][0];
    body_in_world[9] = m[2][1];
    body_in_world[10] = m[2][2];
    body_in_world[11] = z;
    body_in_world[12] = 0;
    body_in_world[13] = 0;
    body_in_world[14] = 0;
    body_in_world[15] = 1;
    Eigen::Map<const Eigen::Matrix < double, 4, 4, Eigen::RowMajor> > Tbody_in_world ( body_in_world );
    Tbody_in_world_ = Tbody_in_world;
    last_state_gt_ = Eigen::Vector3d ( sqrt ( vx * vx + vy * vy ) , wz, yaw );


}

void OVSWrapper::arucoTagCallback ( const gazebo_msgs::ModelStatesConstPtr& aruco_msg ) {
    for ( unsigned int i = 0 ; i < aruco_msg->name.size(); i++ ) {
        if ( aruco_msg->name[i] == "aruco" ) {
            double qx = aruco_msg->pose[i].orientation.x;
            double qy = aruco_msg->pose[i].orientation.y;
            double qz = aruco_msg->pose[i].orientation.z;
            double qw = aruco_msg->pose[i].orientation.w;
            double x = aruco_msg->pose[i].position.x;
            double y = aruco_msg->pose[i].position.y;
            double z = aruco_msg->pose[i].position.z;

            tf::Quaternion q (
                qx,
                qy,
                qz,
                qw );

            tf::Matrix3x3 m ( q );
            double tag_in_world[16];
            tag_in_world[0] = m[0][0];
            tag_in_world[1] = m[0][1];
            tag_in_world[2] = m[0][2];
            tag_in_world[3] = x;
            tag_in_world[4] = m[1][0];
            tag_in_world[5] = m[1][1];
            tag_in_world[6] = m[1][2];
            tag_in_world[7] = y;
            tag_in_world[8] = m[2][0];
            tag_in_world[9] = m[2][1];
            tag_in_world[10] = m[2][2];
            tag_in_world[11] = z;
            tag_in_world[12] = 0;
            tag_in_world[13] = 0;
            tag_in_world[14] = 0;
            tag_in_world[15] = 1;
            Eigen::Map<const Eigen::Matrix < double, 4, 4, Eigen::RowMajor> > Ttag_in_world ( tag_in_world );
            Ttag_in_world_ = Ttag_in_world;
        }
    }
}

void OVSWrapper::imageCallback ( const sensor_msgs::CompressedImageConstPtr& image_msg ) {
    try {

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy ( image_msg, sensor_msgs::image_encodings::BGR8 );
        cv::Mat read_image_distorted = cv_ptr->image;
        cv::cvtColor ( read_image_distorted, read_image_distorted, CV_BGR2GRAY );
        cv::Mat read_image;
        read_image = read_image_distorted;
        Eigen::Vector4d pt;
        Eigen::Vector2d proj ( -10000, -10000 );
        detector_.detectArucoTags ( read_image, pt, proj );
        num_images_ ++;
        if ( proj.norm() < 10000 ) {
            pt_for_optimization_ = pt;
            ready_for_optimization_aruco_ = true;
        }

        cv::imshow ( "Read Image", read_image );

        cv::waitKey ( 1 );
    } catch ( cv_bridge::Exception &e ) {
        ROS_ERROR ( "cv_bridge exception: %s", e.what() );
    }
}

void OVSWrapper::initializeRosPipeline() {
    pc3d_sub_ = n_.subscribe ( wrapper_params_.pc3dTopic, 10, &OVSWrapper::pointCloudCallback3D, this );
    pc2d_sub_ = n_.subscribe ( wrapper_params_.pc2dTopic, 10, &OVSWrapper::pointCloudCallback2D, this ); // laser to point cloud data
    image_sub_ = n_.subscribe ( wrapper_params_.imageTopic, 10, &OVSWrapper::imageCallback, this ); // image data
    gt_sub_ = n_.subscribe ( wrapper_params_.gtTopic, 10, &OVSWrapper::gtCallback, this );
    tag_sub_ = n_.subscribe ( wrapper_params_.tagTopic, 10, &OVSWrapper::arucoTagCallback, this );
    ptz_pub_ = n_.advertise<axis_camera::Axis> ( wrapper_params_.ptzTopic, 1000 );
    pan_pub_ = n_.advertise<std_msgs::Float64> ( wrapper_params_.panTopic, 1000 );
    tilt_pub_ = n_.advertise<std_msgs::Float64> ( wrapper_params_.tiltTopic, 1000 );
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist> ( wrapper_params_.cmdVelTopic, 1000 );
    trajectory_pub_ = n_.advertise<gazebo_msgs::ModelState> ( wrapper_params_.publishXYTopic, 1000 );
}




int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "optimal_visual_servoing" );
    OVSWrapper wrapper ( "/home/thesidjway/research_ws/src/optimal-visual-servoing/params/optimal_visual_servoing.yaml",
                         "/home/thesidjway/research_ws/src/optimal-visual-servoing/params/aruco_params.yaml" );
    wrapper.last_optimization_time = ros::Time::now();
    while ( ros::ok() ) {

        if ( ( ros::Time::now().toNSec() - wrapper.last_optimization_time.toNSec() ) > 99999999 ) {
            double dt = ( ros::Time::now().toNSec() - wrapper.last_optimization_time.toNSec() ) / 1000000000.0;
            Eigen::Matrix4d tag_in_world = wrapper.Ttag_in_world_;
            if ( wrapper.ready_for_optimization_aruco_ ) {
                wrapper.opt_problem_.addDistanceFactor ( wrapper.pt_for_optimization_, wrapper.last_gt_, dt, wrapper.last_vels_, 10 );
                if ( wrapper.last_data_clusters_.size() > 0 || wrapper.last_line_segments_.size() > 0 ) {
                    wrapper.opt_problem_.addRangeFactors ( wrapper.last_data_clusters_, wrapper.last_line_segments_, 10 );
                    wrapper.last_data_clusters_.clear();
                    wrapper.last_line_segments_.clear();
                }
                wrapper.state_.x = 0.0;
                wrapper.state_.y = 0.0;
                wrapper.state_.vel = wrapper.last_state_gt_ ( 0 );
                if ( wrapper.last_state_gt_ ( 1 ) * wrapper.last_state_gt_ ( 1 ) > 1e-6 ) {
                    wrapper.state_.omega = wrapper.last_state_gt_ ( 1 );
                } else {
                    wrapper.state_.omega = 0.001;
                }
                wrapper.state_.yaw = 0;
                wrapper.dyn_win_.getFeasibleSearchSpaceBoundary ( wrapper.state_, wrapper.boundary_, wrapper.dynamic_window_ );
                wrapper.opt_problem_.addFeasibleBoundary ( wrapper.boundary_, 10000 );

                wrapper.opt_problem_.addTagFactors ( wrapper.pt_for_optimization_, 10 );
                PTZCommand cmd = wrapper.opt_problem_.getPTZCommand();
                wrapper.publishPanAndTilt ( cmd );
                wrapper.opt_problem_.optimizeGraph();
// 		MotionCommand motion_cmd = wrapper.opt_problem_.getMotionCommand();
//             wrapper.publishCommandVel ( motion_cmd.v, motion_cmd.omega );

//                 PositionCommand position_cmd = wrapper.opt_problem_.getLocalPositionCommand ( );
//                 MotionCommand motion_cmd;
//                 wrapper.dyn_win_.deriveVelocityCommand ( position_cmd, motion_cmd, wrapper.dynamic_window_ );
//                 wrapper.publishCommandVel ( motion_cmd.v, motion_cmd.omega );
// 		std::cout << "Results Vel: " <<  motion_cmd.v << " " << motion_cmd.omega << std::endl;
                PositionCommand position_cmd = wrapper.opt_problem_.getGlobalPositionCommand ( wrapper.last_gt_ );
                wrapper.publishPosition ( position_cmd.x, position_cmd.y, position_cmd.theta );
                wrapper.ready_for_optimization_aruco_ = false;

            } else {
//                 MotionCommand motion_cmd;
//                 wrapper.publishCommandVel ( 0.0, 0.0001 );
            }

            wrapper.last_optimization_time = ros::Time::now();

        }


        ros::spinOnce();
    }
}

