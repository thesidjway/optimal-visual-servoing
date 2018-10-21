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

#pragma once

#include <optimal_visual_servoing/OptimizationProblem.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>


struct ClusterExtractionParams {

    ClusterExtractionParams() {}
    ClusterExtractionParams ( double cluster_tolerance,
                              int min_cluster_size,
                              int max_cluster_size )
        : cluster_tolerance ( cluster_tolerance ), max_cluster_size ( max_cluster_size ), min_cluster_size ( min_cluster_size ) {}

    double cluster_tolerance = 0.1f; // 1.0 equals 1 m
    int min_cluster_size  = 10;
    int max_cluster_size  = 5000;
};

class ClusterExtractor
{
private:
    ClusterExtractionParams params_;
    std::vector<pcl::PointIndices> cluster_indices_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc_cluster_;
    pcl::PointCloud< pcl::PointXYZ >::Ptr input_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_; 
    

public:
    ClusterExtractor();
    ~ClusterExtractor();
    void segmentPointcloud ();
    void setInputCloud ( pcl::PointCloud< pcl::PointXYZ >::Ptr& cloud );
    void extractSegmentFeatures ( std::vector<RangeDataTuple>& segments );

};
