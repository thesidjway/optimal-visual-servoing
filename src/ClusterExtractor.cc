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
    input_cloud_ = boost::make_shared < pcl::PointCloud< pcl::PointXYZ > > ();
}

ClusterExtractor::~ClusterExtractor() {

}

void ClusterExtractor::readClusteringParams ( std::string params_file ) {
    YAML::Node config = YAML::LoadFile ( params_file );
    params_.cluster_tolerance = config["clustering"]["cluster_tolerance"].as<double>();
    params_.min_cluster_size = config["clustering"]["min_cluster_size"].as<int>();
    params_.max_cluster_size = config["clustering"]["max_cluster_size"].as<int>();
    params_.inlier_fraction = config["clustering"]["inlier_fraction"].as<double>();
    params_.inlier_tolerance = config["clustering"]["inlier_tolerance"].as<double>();
    params_.min_line_segment_size = config["clustering"]["min_line_segment_size"].as<int>();
}


void ClusterExtractor::setInputCloud ( pcl::PCLPointCloud2& cloud ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1D ( new pcl::PointCloud<pcl::PointXYZ> );
    std::vector< pcl::PointCloud<pcl::PointXYZ> > cloud_vector;
    fromPCLPointCloud2ToVelodyneCloud ( cloud, *cloud1D, cloud_vector, 16 );
    input_cloud_ = boost::make_shared < pcl::PointCloud< pcl::PointXYZ > > ( cloud_vector[8] );
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

float ClusterExtractor::distFromLine ( float x, float y, float m, float c ) {
    return ( fabs ( m*x - y + c ) /sqrt ( m*m + 1.0 ) );
}

void ClusterExtractor::getLineParam ( float x1, float y1, float x2, float y2, float& m, float& c ) {
    m = ( y2 - y1 ) / ( x2 -x1 );
    c = ( y1 - x1* ( y2 - y1 ) / ( x2 -x1 ) );
}



void ClusterExtractor::computeSegments ( pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,std::vector<LineSegmentDataTuple>&  extracted_segments,int frontal,int distal ) {

    if ( ( distal - frontal ) < params_.min_line_segment_size )                    
        return;

    int inliers = 0, target_index;
    float  m , c, max_dist = -1;
    float thresh = params_.inlier_fraction,  in_dist = params_.inlier_tolerance;
    pcl::PointXYZ v1 = cloud->points[frontal];
    pcl::PointXYZ v2 = cloud->points[distal];
    getLineParam ( v1._PointXYZ::data[0],v1._PointXYZ::data[1],v2._PointXYZ::data[0],v2._PointXYZ::data[1],m,c );

    for ( int j=frontal; j<=distal; j++ ) {
        pcl::PointXYZ querypt = cloud->points[j];
        float x = querypt._PointXYZ::data[0];
        float y = querypt._PointXYZ::data[1];
        float dist = distFromLine ( querypt._PointXYZ::data[0],querypt._PointXYZ::data[1],m,c );
        if ( dist > max_dist ) {
            max_dist = std::max ( dist,max_dist );
            target_index = j;
        }

        if ( dist <= in_dist ) {
            inliers++;
        }
    }

    if ( inliers > thresh* ( distal - frontal ) ) {
        LineSegmentDataTuple line_segment = LineSegmentDataTuple ( v1._PointXYZ::data[0],v1._PointXYZ::data[1],v2._PointXYZ::data[0],v2._PointXYZ::data[1] );
        extracted_segments.push_back ( line_segment );
    } else {
        int frontal_back = frontal;
        frontal = target_index;
        computeSegments ( cloud, extracted_segments,frontal,distal );
        frontal =frontal_back;
        distal  = target_index;
        computeSegments ( cloud, extracted_segments,frontal,distal );
    }
}


void ClusterExtractor::extractSegmentFeatures ( std::vector<RangeDataTuple>& segments, std::vector< LineSegmentDataTuple >& line_segments ) {

    for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_.begin (); it != cluster_indices_.end (); ++it ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster ( new pcl::PointCloud<pcl::PointXYZ> );
        double min_dist = 1000.0, width = -62.0;
        Eigen::Vector2d bearing_v1, bearing_v2, bearing, x ( 1,0 );


        for ( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit ) {
            cloud_cluster->points.push_back ( input_cloud_->points[*pit] );
        }
        int frontal = 0;
        int distal = cloud_cluster->points.size() - 1;
        computeSegments ( cloud_cluster,line_segments,frontal,distal );

        RangeDataTuple cluster = RangeDataTuple ( min_dist, 0.0, width );
        segments.push_back ( cluster );
        //cluster.printDataTuple();
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
    cluster_indices_.clear();
}

template <typename PointT>
void ClusterExtractor::fromPCLPointCloud2ToVelodyneCloud ( const pcl::PCLPointCloud2& msg,
        pcl::PointCloud<PointT>& cloud1D,
        std::vector< pcl::PointCloud<PointT> >& cloudVector,
        unsigned int rings ) {
    cloud1D.header   = msg.header;
    cloud1D.width    = msg.width;
    cloud1D.height   = msg.height;
    cloud1D.is_dense = msg.is_dense == 1;
    uint32_t num_points = msg.width * msg.height;
    cloud1D.points.resize ( num_points );
    uint8_t* cloud_data1 = reinterpret_cast<uint8_t*> ( &cloud1D.points[0] );

    pcl::PointCloud<PointT>* cloudPerLaser = new pcl::PointCloud<PointT>[rings];
    uint8_t* cloud_data2[rings];

    unsigned int pointsCounter[rings] = {0};

    for ( unsigned int i=0; i<rings; ++i ) {
        cloudPerLaser[i] = pcl::PointCloud<PointT>();
        cloudPerLaser[i].header   = msg.header;
        cloudPerLaser[i].width    = msg.width;
        cloudPerLaser[i].height   = msg.height;
        cloudPerLaser[i].is_dense = msg.is_dense == 1;
        cloudPerLaser[i].points.resize ( num_points );
        cloud_data2[i] = reinterpret_cast<uint8_t*> ( &cloudPerLaser[i].points[0] );
    }

    for ( uint32_t row = 0; row < msg.height; ++row ) {
        const uint8_t* row_data = &msg.data[row * msg.row_step];

        for ( uint32_t col = 0; col < msg.width; ++col ) {
            const uint8_t* msg_data = row_data + col * msg.point_step;
            uint16_t* ring = ( uint16_t* ) ( msg_data+20 );
            memcpy ( cloud_data2[*ring], msg_data, 22 );
            memcpy ( cloud_data1, msg_data, 22 );
            pointsCounter[*ring]++;
            cloud_data1 += sizeof ( PointT );
            cloud_data2[*ring] += sizeof ( PointT );
        }
    }

    cloudVector = std::vector< pcl::PointCloud<PointT> > ( rings );

    for ( unsigned int i=0; i<rings; ++i ) {
        cloudPerLaser[i].width = pointsCounter[i];
        cloudPerLaser[i].height = 1;
        cloudPerLaser[i].points.resize ( pointsCounter[i] );
        cloudVector[i] = ( cloudPerLaser[i] );
    }

    delete[] cloudPerLaser;
}
