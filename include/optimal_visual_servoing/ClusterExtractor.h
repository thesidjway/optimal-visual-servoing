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

#include <optimal_visual_servoing/Optimization.h>

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
#include <Eigen/Dense>
#include <cmath>
#include <yaml-cpp/yaml.h>


struct ClusterExtractionParams {

    ClusterExtractionParams() {}
    ClusterExtractionParams ( double cluster_tolerance,
                              int min_cluster_size,
                              double inlier_fraction,
                              double inlier_tolerance,
                              int min_line_segment_size,
                              int max_cluster_size,
			      int max_irregular_cluster_size
			    )
        : cluster_tolerance ( cluster_tolerance ), max_cluster_size ( max_cluster_size ), inlier_fraction ( inlier_fraction ), inlier_tolerance ( inlier_tolerance ), min_line_segment_size ( min_line_segment_size ), min_cluster_size ( min_cluster_size ), max_irregular_cluster_size (max_irregular_cluster_size) {}

    double cluster_tolerance = 0.1f; // 1.0 equals 1 m
    int min_cluster_size  = 10;
    int max_cluster_size  = 5000;
    double inlier_fraction = 0.85;
    double inlier_tolerance = 0.05;
    int min_line_segment_size = 10;
    int max_irregular_cluster_size = 10;
};

class ClusterExtractor
{
private:
    ClusterExtractionParams params_;
    std::vector<pcl::PointIndices> cluster_indices_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc_cluster_;
    pcl::PointCloud< pcl::PointXYZ >::Ptr input_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

    template <typename PointT>
    void fromPCLPointCloud2ToVelodyneCloud ( const pcl::PCLPointCloud2& msg,
            pcl::PointCloud<PointT>& cloud1D,
            std::vector< pcl::PointCloud<PointT> >& cloudVector,
            unsigned int rings );


public:
    ClusterExtractor();
    ~ClusterExtractor();
    void readClusteringParams ( std:: string params_file );
    void segmentPointcloud ();
    void setInputCloud ( pcl::PCLPointCloud2& cloud );
    void setInputCloud ( pcl::PointCloud< pcl::PointXYZ >::Ptr& cloud );
    void extractSegmentFeatures ( std::vector<RangeDataTuple>& segments, std::vector< LineSegmentDataTuple >& line_segments );
    float distFromLine ( float x, float y, float m, float c );
    void getLineParam ( float x1, float y1, float x2, float y2, float& m, float& c );
    void computeClusterParams(double& width, double& min_dist, double& bearing_angle, pcl::PointCloud<pcl::PointXYZ>::Ptr& ip, int pt_size,  std::vector<RangeDataTuple>& segments);
    void computeSegments ( pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,std::vector<LineSegmentDataTuple>&  extracted_segments,int frontal,int distal );
};
