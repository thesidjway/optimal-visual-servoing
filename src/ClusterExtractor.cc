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
        double min_dist = 1000.0, width = -62.0;
        Eigen::Vector2d bearing_v1, bearing_v2, bearing, x ( 1,0 );


        for ( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit ) {
            cloud_cluster->points.push_back ( input_cloud_->points[*pit] );
        }

        pcl::PointXYZ v1 = cloud_cluster->points[0];
        Eigen::Vector2d pcl_v1 ( v1._PointXYZ::data[0], v1._PointXYZ::data[1] );
        pcl::PointXYZ v2 = cloud_cluster->points[cloud_cluster->points.size() - 1];
        Eigen::Vector2d pcl_v2 ( v2._PointXYZ::data[0], v2._PointXYZ::data[1] );
        double angle1 = atan2 ( pcl_v1 ( 1 ) , pcl_v1 ( 0 ) ) ;
        double angle2 = atan2 ( pcl_v2 ( 1 ) , pcl_v2 ( 0 ) ) ;
        double bearing_angle = ( angle1 + angle2 ) / 2.0 * 180.0 / M_PI;

        if ( angle2 < angle1 ) {
            angle2 += 2 * M_PI ;
        }
        width = ( angle2 - angle1 ) * 180.0 / M_PI ;


        for ( int j=0; j < cloud_cluster->points.size(); j++ ) {
            pcl::PointXYZ pt = cloud_cluster->points[j];
            Eigen::Vector2d pcl_pt ( pt._PointXYZ::data[0], pt._PointXYZ::data[1] );
            min_dist = std::min ( min_dist, pcl_pt.norm() );
        }



        RangeDataTuple cluster = RangeDataTuple ( min_dist, bearing_angle, width );
        segments.push_back ( cluster );
        cluster.printDataTuple();
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
    cluster_indices_.clear();
}
