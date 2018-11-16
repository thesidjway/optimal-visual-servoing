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
    dx_dy_dtheta_[0] = 0.0;
    dx_dy_dtheta_[1] = 0.0;
    last_p = 0.0;
    last_t = 1.571;
    p_t_[0] = 0.0;
    p_t_[1] = 1.571;
    dx_dy_dtheta_[2] = 0.0;
    vel_omega_[0] = 0;
    vel_omega_[1] = 0.001;
}

Optimization::~Optimization() {

}


void Optimization::addRangeFactors ( std::vector<RangeDataTuple> &cluster_tuples, std::vector<LineSegmentDataTuple> &line_tuples, double weight ) {
    for ( uint i = 0 ; i < line_tuples.size() ; i++ ) {
        ceres::CostFunction* cost_function_line =
            LineSegmentError::Create ( line_tuples[i], weight );
        cost_functions_range.push_back ( cost_function_line );
    }
    for ( uint i = 0 ; i < cluster_tuples.size() ; i++ ) {
        ceres::CostFunction* cost_function_range =
            ClusterError::Create ( cluster_tuples[i], weight );
        cost_functions_range.push_back ( cost_function_range );
    }

    if ( cluster_tuples.size() > 0 || line_tuples.size() > 0 ) {
        ready_for_range_ = true;
    }

}
/*
void Optimization::addDynamicWindowFactors ( std::vector<RangeDataTuple> &cluster_tuples, std::vector<LineSegmentDataTuple> &line_tuples, double weight ) {
    for ( uint i = 0 ; i < line_tuples.size() ; i++ ) {
        ceres::CostFunction* cost_function_line =
            LineSegmentError::Create ( line_tuples[i], weight );
        cost_functions_range.push_back ( cost_function_line );
    }
    for ( uint i = 0 ; i < cluster_tuples.size() ; i++ ) {
        ceres::CostFunction* cost_function_range =
            ClusterError::Create ( cluster_tuples[i], weight );
        cost_functions_range.push_back ( cost_function_range );
    }

    if ( cluster_tuples.size() > 0 || line_tuples.size() > 0 ) {
        ready_for_range_ = true;
    }

}*/

void Optimization::addDistanceFactor ( Eigen::Vector4d tag_in_cam, Eigen::Vector3d last_gt, double dt, Eigen::Vector3d last_vels, double weight ) {
    double cam_in_body_old[16];
    double tag_in_world_copy[16];
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
    Eigen::Vector4d tag_in_body = eigen_cam_in_body_old * tag_in_cam;
    std::cout << "Tag in body: [ " << tag_in_body ( 0, 0 ) << " , " << tag_in_body ( 1, 0 ) << " ]" <<  std::endl;
    cost_function_distance = xyError::Create ( tag_in_body, weight, last_gt, dt );
//     cost_function_distance = DistanceError::Create ( tag_in_body, 1, last_gt, dt );
    ready_for_distance_ = true;
}

void Optimization::addTagFactors ( Eigen::Vector4d target_in_cam, double weight ) {
//     p_t_new_[0] = 0.0;
//     p_t_new_[1] = 1.59;
    double cam_in_body_old[16];
    double sp = sin ( last_p );
    double st = sin ( last_t );
    double cp = cos ( last_p );
    double ct = cos ( last_t );
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

    if ( eigen_cam_in_body_old.determinant() > 0.5 ) {
        cost_function_projection = ProjectionErrorPTOnly::Create ( target_in_cam, params_.K, eigen_cam_in_body_old, weight );
        ready_for_projection_ = true;
    }
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

void Optimization::addFeasibleBoundary ( Boundary boundary, double weight ) {
    cost_function_boundary = BoundaryError::Create ( boundary, weight );
    ready_boundary_ = true;
}


void Optimization::optimizeGraph() {
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    if ( ready_for_distance_ ) {
        problem.AddResidualBlock ( cost_function_distance,
                                   NULL,
                                   dx_dy_dtheta_ );
    }
    if ( ready_for_range_ ) {
        for ( uint i = 0; i < cost_functions_range.size(); i++ ) {
            problem.AddResidualBlock ( cost_functions_range[i],
                                       NULL,
                                       dx_dy_dtheta_ );
        }
        cost_functions_range.clear();
    }
    if ( ready_for_projection_ ) {
        problem.AddResidualBlock ( cost_function_projection,
                                   NULL,
                                   p_t_,
                                   dx_dy_dtheta_ );
        problem.SetParameterLowerBound ( p_t_, 0, -1.571 );
        problem.SetParameterUpperBound ( p_t_, 0, 1.571 );
        problem.SetParameterLowerBound ( p_t_, 1, 0.0 );
        problem.SetParameterUpperBound ( p_t_, 1, 3.1415 );
        last_p = p_t_[0];
        last_t = p_t_[1];
    }
    if ( ready_boundary_ ) {
        problem.AddResidualBlock ( cost_function_boundary,
                                   NULL,
                                   dx_dy_dtheta_ );
    }


//     problem.SetParameterLowerBound ( vel_omega_, 0, vel_omega_[0] - 0.3);
//     problem.SetParameterUpperBound ( vel_omega_, 0, vel_omega_[0] + 0.3);
//     problem.SetParameterLowerBound ( vel_omega_, 1, vel_omega_[1] - 0.15);
//     problem.SetParameterUpperBound ( vel_omega_, 1, vel_omega_[1] + 0.15);
    ceres::Solve ( options, &problem, &summary );

    ready_for_projection_ = false;
    ready_for_distance_ = false;
    ready_for_range_ = false;
    ready_boundary_ = false;
    std::cout << "Results Projection: " << asin(sin(p_t_[0])) << " " << acos(cos(p_t_[1])) << std::endl;
//     std::cout << "Results Vel: " << vel_omega_[0] << " " << vel_omega_[1] << std::endl;
    std::cout << "Results XYtheta: " << dx_dy_dtheta_[0] << " " << dx_dy_dtheta_[1] << " " <<  dx_dy_dtheta_[2] << std::endl;



}
