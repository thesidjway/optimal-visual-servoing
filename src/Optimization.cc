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

#include <optimal_visual_servoing/Optimization.h>

Optimization::Optimization() {
    p_t_[0] = 1.0;
    p_t_[1] = 1.57;
}

Optimization::~Optimization() {

}


void Optimization::addRangeFactor ( RangeDataTuple &tuple, double weight ) {
    ceres::CostFunction *cost_function =
        RangeError::Create ( tuple, weight );

    problem_.AddResidualBlock ( cost_function,
                                NULL,
                                dx_dy_dtheta_vel_omega_ );

    problem_.SetParameterLowerBound ( dx_dy_dtheta_vel_omega_, 0, -0.3 );
    problem_.SetParameterUpperBound ( dx_dy_dtheta_vel_omega_, 0, 0.3 );

    problem_.SetParameterLowerBound ( dx_dy_dtheta_vel_omega_, 1, -0.2 );
    problem_.SetParameterUpperBound ( dx_dy_dtheta_vel_omega_, 1, 0.2 );

    problem_.SetParameterLowerBound ( dx_dy_dtheta_vel_omega_, 2, -0.3 );
    problem_.SetParameterUpperBound ( dx_dy_dtheta_vel_omega_, 2, 0.3 );

}

void Optimization::addTagFactors ( Eigen::Vector4d target_in_cam, double weight ) {
    double cam_in_body_old[16];
    double sp = sin ( p_t_[0] );
    double st = sin ( p_t_[1] );
    double cp = cos ( p_t_[0] );
    double ct = cos ( p_t_[1] );
    cam_in_body_old[0] = -sp;
    cam_in_body_old[1] = st * cp;
    cam_in_body_old[2] = cp * ct;
    cam_in_body_old[3] = 0.19;
    cam_in_body_old[4] = -cp;
    cam_in_body_old[5] = -st * sp;
    cam_in_body_old[6] = -ct * sp;
    cam_in_body_old[7] = 0.0;
    cam_in_body_old[8] = 0.0;
    cam_in_body_old[9] = -ct;
    cam_in_body_old[10] = st;
    cam_in_body_old[11] = 0.395;
    cam_in_body_old[12] = 0.0;
    cam_in_body_old[13] = 0.0;
    cam_in_body_old[14] = 0.0;
    cam_in_body_old[15] = 1.0;
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > eigen_cam_in_body_old ( cam_in_body_old );
    
    ceres::CostFunction *cost_function =
        ProjectionErrorPTOnly::Create ( target_in_cam, params_.K, eigen_cam_in_body_old, weight );
	
//     ceres::CostFunction *cost_function_change =
//         PanTiltChangeError::Create ( p_t_[0], p_t_[1], 0.5, 0.25 );
    problem_.AddResidualBlock ( cost_function,
                                NULL,
                                p_t_ );
//     problem_.AddResidualBlock ( cost_function_change,
//                                 NULL,
//                                 p_t_ );

    problem_.SetParameterLowerBound ( p_t_, 0, -3.14159 );
    problem_.SetParameterUpperBound ( p_t_, 0, 3.14159 );
    problem_.SetParameterLowerBound ( p_t_, 1, 0 );
    problem_.SetParameterUpperBound ( p_t_, 1, 3.14159 );
}


void Optimization::readOptimizationParams ( std::string params_file ) { //placeholder
    YAML::Node config = YAML::LoadFile ( params_file );
    params_.fx = config["camera"]["fx"].as<double>();
    params_.fy = config["camera"]["fy"].as<double>();
    params_.cx = config["camera"]["cx"].as<double>();
    params_.cy = config["camera"]["cy"].as<double>();
    params_.K << params_.fx, 0.0, params_.cx, 0.0,
              0.0, params_.fy, params_.cy, 0.0,
              0.0, 0.0, 1.0, 0.0;
}


void Optimization::optimizeGraph() {
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    std::cout << "Results: " << p_t_[0] << " " << p_t_[1] << std::endl;
    ceres::Solve ( options, &problem_, &summary );
}
