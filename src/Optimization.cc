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

void Optimization::generateData ( std::vector<RangeDataTuple> &gen_data ) {
    srand ( time ( NULL ) );
    int angle_init = 0;
    int angle_final = 0;
    while ( angle_final < 360 ) {
        double dist = rand() % 10 + 1;
        int width = rand() % 30;
        angle_final = std::min ( angle_init + width, 360 );
        gen_data.push_back ( RangeDataTuple ( dist, ( angle_final + angle_init ) / 2.0, angle_final - angle_init ) );
        angle_init = angle_final;
    }
}

void Optimization::addRangeFactor ( RangeDataTuple &tuple ) {
    ceres::CostFunction *cost_function =
        RangeError::Create ( tuple, 1 );
    problem.AddResidualBlock ( cost_function,
                               NULL,
                               dx_dy_dtheta_vel_omega_ );

}

void Optimization::readOptimizationParams ( std::string params_file ) { //placeholder
    YAML::Node config = YAML::LoadFile ( params_file );
    params_.cluster_tolerance = config["clustering"]["cluster_tolerance"].as<double>();
    params_.min_cluster_size = config["clustering"]["min_cluster_size"].as<int>();
    params_.max_cluster_size = config["clustering"]["max_cluster_size"].as<int>();
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
