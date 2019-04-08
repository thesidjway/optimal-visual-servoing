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
#include <optimal_visual_servoing/DynamicWindowSampler.h>


struct OptimizationParams { //Placeholder

    OptimizationParams() {}

    double fx;
    double fy;
    double cx;
    double cy;
    Eigen::Matrix< double, 3, 4 > K;
};

class Optimization
{
private:
    double dx_dy_dtheta_[3];
    double vel_omega_[2];
    double p_t_[2];
    double last_p;
    double last_t;
    OptimizationParams params_;
    ceres::CostFunction *cost_function_distance;
    ceres::CostFunction *cost_function_projection;
    ceres::CostFunction *cost_function_boundary;
    std::vector<ceres::CostFunction*> cost_functions_range;
    bool ready_for_projection_ = false;
    bool ready_for_distance_ = false;
    bool ready_for_range_ = false;
    bool ready_boundary_ = false;

public:
    Optimization();
    ~Optimization();
    void addTagFactors ( Eigen::Vector4d target_in_cam, double weight );
    void addDistanceFactor ( Eigen::Vector4d target_in_cam, Eigen::Vector3d last_gt, double dt, Eigen::Vector3d last_vels, double weight );
    void addRangeFactors ( std::vector<RangeDataTuple> &cluster_tuple, std::vector<LineSegmentDataTuple> &line_tuple, double weight );
    void optimizeGraph();
    void addFeasibleBoundary ( Boundary boundary, double weight );
    void readOptimizationParams ( std:: string params_file );
    inline PTZCommand getPTZCommand() {
        return PTZCommand ( asin ( sin ( p_t_[0] ) ) , acos ( cos ( p_t_[1] ) ) , 0 );
    }
    inline MotionCommand getMotionCommand() {
        return MotionCommand ( vel_omega_[0], vel_omega_[1] );
    }
    
    inline PositionCommand getLocalPositionCommand () {
        return PositionCommand ( dx_dy_dtheta_[0], dx_dy_dtheta_[1] , dx_dy_dtheta_[2] );
    }
    inline PositionCommand getGlobalPositionCommand ( Eigen::Vector3d& last_gt ) {
        Eigen::Matrix3d Tcurr;
        Tcurr << cos ( last_gt ( 2 ) ), -sin ( last_gt ( 2 ) ), last_gt ( 0 ),
              sin ( last_gt ( 2 ) ), cos ( last_gt ( 2 ) ), last_gt ( 1 ),
              0 , 0 , 1 ;
        std::cout << Tcurr << std::endl;
        Eigen::Matrix3d Tdelta;
        Tdelta << cos ( dx_dy_dtheta_[2] ), -sin ( dx_dy_dtheta_[2] ), dx_dy_dtheta_[0],
               sin ( dx_dy_dtheta_[2] ), cos ( dx_dy_dtheta_[2] ), dx_dy_dtheta_[1],
               0 , 0 , 1 ;

        Eigen::Matrix3d Tfinal = Tcurr * Tdelta;
        std::cout << Tfinal << std::endl;
        dx_dy_dtheta_[0] = 0.0;
        dx_dy_dtheta_[1] = 0.0;
        dx_dy_dtheta_[2] = 0.0;

        return PositionCommand ( Tfinal ( 0,2 ), Tfinal ( 1,2 ) , atan2 ( Tfinal ( 1,0 ), Tfinal ( 0,0 ) ) );
    }
};





