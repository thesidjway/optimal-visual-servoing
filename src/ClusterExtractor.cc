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
  Foundation, Inc., 51 Franklin Stree_t, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <optimal_visual_servoing/ClusterExtractor.h>

ClusterExtractor::ClusterExtractor() {
    tree_ = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
}

ClusterExtractor::~ClusterExtractor() {

}

void ClusterExtractor::setInputCloud ( pcl::PointCloud< pcl::PointXYZ >::Ptr& cloud ) {
    input_cloud_ = cloud;
}


void ClusterExtractor::segmentPointcloud ( ) {
    tree_->setInputCloud ( input_cloud_ );
    euc_cluster_.setClusterTolerance ( params_.cluster_tolerance );
    euc_cluster_.setMinClusterSize ( params_.min_cluster_size );
    euc_cluster_.setMaxClusterSize ( params_.max_cluster_size );
    euc_cluster_.setSearchMethod ( tree_ );
    euc_cluster_.setInputCloud ( input_cloud_ );
    euc_cluster_.extract ( cluster_indices_ );
}

void ClusterExtractor::extractSegmentFeatures ( std::vector<RangeDataTuple>& segments ) {

    for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_.begin (); it != cluster_indices_.end (); ++it ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster ( new pcl::PointCloud<pcl::PointXYZ> );
        float median_dist, bearing;
        float max_dist = -1.0, min_dist = 9999.0, max_bearing = -1.0, min_bearing = 361.0;
        float distal_angle, frontal_angle, width = 0.0;


        for ( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit ) {
            cloud_cluster->points.push_back ( input_cloud_->points[*pit] );

            pcl::PointXYZ samp = cloud_cluster->points[0];
            distal_angle  = atan2f ( samp._PointXYZ::data[1], samp._PointXYZ::data[0] );
            frontal_angle = distal_angle;
            for ( unsigned int k = 0; k < cloud_cluster->points.size(); k++ ) {
                pcl::PointXYZ v = cloud_cluster->points[k];
                float r_dist = v._PointXYZ::data[0] * v._PointXYZ::data[0] + v._PointXYZ::data[1] * v._PointXYZ::data[1];
                float theta  = atan2f ( v._PointXYZ::data[1], v._PointXYZ::data[0] ); // (y,x)
                max_dist = std::max ( r_dist, max_dist );
                min_dist = std::min ( r_dist, min_dist );
                max_bearing = std::max ( theta, max_bearing );
                min_bearing = std::min ( theta, min_bearing );
                if ( theta < frontal_angle || ( k == ( cloud_cluster->points.size() - 1 ) ) ) {
                    width = width + frontal_angle - distal_angle;
                    distal_angle = theta;
                    frontal_angle = distal_angle;
                } else {
                    frontal_angle = theta;
                }
                //std::cout << "sample parameters: " << r_dist << " : " << theta <<  std::endl;
            }

            bearing = max_bearing - width / 2.0;
            if ( width > M_PI && bearing > 0.0 ) {
                bearing = bearing - M_PI;
            } else if ( width > M_PI && bearing <= 0.0 ) {
                bearing = bearing + M_PI;
            }
        }

        RangeDataTuple cluster = RangeDataTuple ( min_dist, bearing, width );
        segments.push_back ( cluster );
        cluster.printDataTuple();
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
    cluster_indices_.clear();
}
