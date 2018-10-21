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

#include <cstdlib>
#include <vector>
#include <ctime>
#include <iostream>
#include <ceres/ceres.h>

struct RangeDataTuple {
    RangeDataTuple () {}
    RangeDataTuple ( double median_dist, double bearing, double width ) : median_dist ( median_dist ), bearing ( bearing ), width ( width ) {}
    double median_dist;
    double bearing;
    double width;
    void printDataTuple () {
        std::cout << " median_dist : " << median_dist << ", bearing : " << bearing << " , width : " << width  << std::endl;
    }
};

class OptimizationProblem
{
private:

    struct DistanceError {
        DistanceError ( Eigen::Vector3d target_pt, double weight )
            : target_pt ( target_pt ), weight ( weight ) {}

        template <typename T>
        bool operator() ( const T *const t,
                          T *residuals ) const {
            residuals[0] =  T ( sqrt ( ( T ( target_pt ( 0 ) ) - t[0] ) * ( T ( target_pt ( 0 ) ) - t[0] ) + ( T ( target_pt ( 1 ) ) - t[1] ) * ( T ( target_pt ( 1 ) )- t[1] ) + ( T ( target_pt ( 2 ) )- t[2] ) * ( T ( target_pt ( 2 ) ) - t[2] ) ) );
            return true;
        }
        static ceres::CostFunction *Create ( Eigen::Vector3d target_pt, double weight ) {
            return ( new ceres::AutoDiffCostFunction<DistanceError, 3, 1> ( new DistanceError ( target_pt, weight ) ) );
        }
        Eigen::Vector3d target_pt;
        double weight;
    };

    struct RangeError {
        RangeError ( RangeDataTuple data_tuple, double weight )
            : data_tuple ( data_tuple ), weight ( weight ) {}

        template <typename T>
        bool operator() ( const T *const x1,
                          const T *const y1,
                          T *residuals ) const {
            residuals[0] = T ( sqrt ( T ( x1[0] - data_tuple.median_dist * cos ( data_tuple.bearing ) ) * T ( x1[0] - data_tuple.median_dist * cos ( data_tuple.bearing ) ) +
                                      T ( y1[0] - data_tuple.median_dist * sin ( data_tuple.bearing ) ) * T ( y1[0] - data_tuple.median_dist * sin ( data_tuple.bearing ) ) ) );
            return true;
        }
        static ceres::CostFunction *Create ( RangeDataTuple data_tuple, double weight ) {
            return ( new ceres::AutoDiffCostFunction<RangeError, 1, 1, 1> ( new RangeError ( data_tuple, weight ) ) );
        }
        RangeDataTuple data_tuple;
        double weight;
    };

    struct ProjectionError { //incomplete, just a placeholder for now
        ProjectionError ( double weight ) : weight ( weight ) {}
        template <typename T>
        bool operator() ( const T *const x1,
                          const T *const y1,
                          T *residuals ) const {
            residuals[0] = T ( 0 );
            return true;
        }
        static ceres::CostFunction *Create ( double weight ) {
            return ( new ceres::AutoDiffCostFunction<ProjectionError, 1, 1, 1> ( new ProjectionError ( weight ) ) );
        }
        double weight;
    };

    ceres::Problem problem;
    double x1 = 0;
    double y1 = 0;
public:
    OptimizationProblem() {}
    ~OptimizationProblem() {}
    void addTagFactor ();
    void addRangeFactor ( RangeDataTuple &tuple );
    void generateData ( std::vector<RangeDataTuple> &gen_data );
    void optimizeGraph();
};
