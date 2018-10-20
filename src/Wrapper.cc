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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main ( int argc, char **argv ) {
    OptimizationProblem opt_problem;
    DynamicWindowSampler dws;
    ArucoTagsDetection detector;
    Eigen::Vector3d pt;
    cv::Mat b = cv::imread ( "/home/thesidjway/deprecated_stuff/markers-depth-estimator/data/image-test.png" );
    detector.detectArucoTags ( b , pt );
    RobotState a = RobotState ( 0, 0, 1.0, 0.2, 0.6 );
    std::vector<RobotState> feasible_states;
    dws.getFeasibleSearchSpace ( a, feasible_states );
    std::vector<RangeDataTuple> gen_data;
    opt_problem.generateData ( gen_data );
    for ( unsigned int i = 0 ; i < gen_data.size() ; i++ ) {
        opt_problem.addRangeFactor ( gen_data[i] );
    }
    opt_problem.optimizeGraph();
}
