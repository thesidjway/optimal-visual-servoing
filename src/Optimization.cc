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

void Optimization::addRangeFactor ( RangeDataTuple &tuple, double weight ) {
    ceres::CostFunction *cost_function =
        RangeError::Create ( tuple, weight );
    problem.AddResidualBlock ( cost_function,
                               NULL,
                               dx_dy_dtheta_vel_omega_ );

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
    cam_in_body_old[3] = 0.0;
    cam_in_body_old[4] = -cp;
    cam_in_body_old[5] = -st * sp;
    cam_in_body_old[6] = -ct * sp;
    cam_in_body_old[7] = 0.0;
    cam_in_body_old[8] = 0.0;
    cam_in_body_old[9] = -ct;
    cam_in_body_old[10] = st;
    cam_in_body_old[11] = 0.0;
    cam_in_body_old[12] = 0.0;
    cam_in_body_old[13] = 0.0;
    cam_in_body_old[14] = 0.0;
    cam_in_body_old[15] = 1.0;
    Eigen::Map<const Eigen::Matrix<double, 4, 4> > eigen_cam_in_body_old ( cam_in_body_old );
    ceres::CostFunction *cost_function =
        ProjectionError::Create ( target_in_cam, params_.K, eigen_cam_in_body_old, weight );
}


void Optimization::readOptimizationParams ( std::string params_file ) { //placeholder
    YAML::Node config = YAML::LoadFile ( params_file );
    params_.cluster_tolerance = config["clustering"]["cluster_tolerance"].as<double>();
    params_.min_cluster_size = config["clustering"]["min_cluster_size"].as<int>();
    params_.max_cluster_size = config["clustering"]["max_cluster_size"].as<int>();
    params_.fx = config["camera"]["fx"].as<int>();
    params_.fy = config["camera"]["fy"].as<int>();
    params_.cx = config["camera"]["cx"].as<int>();
    params_.cy = config["camera"]["cy"].as<int>();
    params_.K << params_.fx, 0.0, params_.cx, 0.0,
              0.0, params_.fy, params_.cy, 0.0,
              0.0, 0.0, 0.0, 1.0;
}


void Optimization::optimizeGraph() {
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    std::cout << "Results: " << dx_dy_dtheta_vel_omega_[0] << " " << dx_dy_dtheta_vel_omega_[1] << std::endl;
    ceres::Solve ( options, &problem, &summary );
}
