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
        DistanceError ( RangeDataTuple data_tuple )
            : data_tuple ( data_tuple ) {}

        template <typename T>
        bool operator() ( const T *const r1,
                          const T *const theta1,
                          T *residuals ) const {
            residuals[0] =  T ( data_tuple.median_dist ) - r1[0];
            return true;
        }
        static ceres::CostFunction *Create ( RangeDataTuple data_tuple ) {
            return ( new ceres::AutoDiffCostFunction<DistanceError, 1, 1, 1> ( new DistanceError ( data_tuple ) ) );
        }

        RangeDataTuple data_tuple;
    };
    ceres::Problem problem;
    double r1 = 0.5;
    double theta1 = 10;
public:
    OptimizationProblem() {}
    ~OptimizationProblem() {}
    void addTagFactor ();
    void addRangeFactor ( RangeDataTuple &tuple );
    void generateData ( std::vector<RangeDataTuple> &gen_data );
    void optimizeGraph();
};
