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
#include <ceres/rotation.h>
#include <yaml-cpp/yaml.h>

#include <optimal_visual_servoing/ErrorFunction.h>



struct OptimizationParams { //Placeholder

    OptimizationParams() {}
    OptimizationParams ( double cluster_tolerance,
                         int min_cluster_size,
                         int max_cluster_size )
        : cluster_tolerance ( cluster_tolerance ), max_cluster_size ( max_cluster_size ), min_cluster_size ( min_cluster_size ) {}

    double cluster_tolerance = 0.1f; // 1.0 equals 1 m
    int min_cluster_size  = 10;
    int max_cluster_size  = 5000;
    double fx;
    double fy;
    double cx;
    double cy;
    Eigen::Matrix< double, 3, 4 > K;
};

class Optimization
{
private:

    ceres::Problem problem;
    double dx_dy_dtheta_vel_omega_[];
    double p_t_[];
    OptimizationParams params_;
public:
    Optimization() {}
    ~Optimization() {}
    void addTagFactors ( Eigen::Vector4d target_in_cam, double weight );
    void addRangeFactor ( RangeDataTuple &tuple, double weight );
    void optimizeGraph( );
    void readOptimizationParams ( std:: string params_file );
    inline PTZCommand getPTZCommand() {
        return PTZCommand ( p_t_[0] * 180.0 / M_PI, p_t_[1] * 180.0 / M_PI, 0 );
    }
};




