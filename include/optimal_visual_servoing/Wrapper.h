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

#include <optimal_visual_servoing/Optimization.h>
#include <optimal_visual_servoing/DynamicWindowSampler.h>
#include <optimal_visual_servoing/ArucoTagsDetection.h>
#include <optimal_visual_servoing/ClusterExtractor.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <laser_geometry/laser_geometry.h>
#include <image_transport/image_transport.h>
#include <axis_camera/Axis.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>

class OVSWrapper
{
private:
    ros::NodeHandle n_;
    ros::Subscriber pc2d_sub_;
    ros::Subscriber pc3d_sub_;
    ros::Subscriber image_sub_;
    void initializeRosPipeline();

public:
    OVSWrapper ( std::string params_file );
    ~OVSWrapper();
    ClusterExtractor cluster_extractor_;
    Optimization opt_problem_;
    ArucoTagsDetection detector_;
    
    ros::Publisher ptz_pub_;
    void pointCloudCallback3D ( const sensor_msgs::PointCloud2ConstPtr& callback_cloud );
    void pointCloudCallback2D ( const sensor_msgs::LaserScanConstPtr& laser_scan );
    void imageCallback ( const sensor_msgs::CompressedImageConstPtr& callback_image );
    std::vector<RangeDataTuple> last_data_clusters_;

};
