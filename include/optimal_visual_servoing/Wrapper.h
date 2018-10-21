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
#include <optimal_visual_servoing/ClusterExtractor.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <laser_geometry/laser_geometry.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>

class OVSWrapper
{
private:
    ros::NodeHandle n_;
    ros::Subscriber pc_sub_;
    ros::Subscriber image_sub_;
    void initializeRosPipeline();
    ClusterExtractor cluster_extractor_;

public:
    OVSWrapper();
    ~OVSWrapper();
    void pointCloudCallback ( const sensor_msgs::LaserScanConstPtr& callback_cloud );
    void imageCallback ( const sensor_msgs::ImageConstPtr& callback_image );
    std::vector<RangeDataTuple> last_data_clusters_;

};
