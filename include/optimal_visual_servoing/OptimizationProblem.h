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
            return ( new ceres::AutoDiffCostFunction<DistanceError, 1, 3> ( new DistanceError ( target_pt, weight ) ) );
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
            residuals[0] = T ( 1.0 ) / exp ( sqrt ( T ( x1[0] - data_tuple.median_dist * cos ( data_tuple.bearing ) ) * T ( x1[0] - data_tuple.median_dist * cos ( data_tuple.bearing ) ) +
                                                    T ( y1[0] - data_tuple.median_dist * sin ( data_tuple.bearing ) ) * T ( y1[0] - data_tuple.median_dist * sin ( data_tuple.bearing ) ) ) * T ( sqrt ( x1[0] * x1[0]  + y1[0]  * y1[0] ) - data_tuple.median_dist ) ) ;
            return true;
        }
        static ceres::CostFunction *Create ( RangeDataTuple data_tuple, double weight ) {
            return ( new ceres::AutoDiffCostFunction<RangeError, 1, 1, 1> ( new RangeError ( data_tuple, weight ) ) );
        }
        RangeDataTuple data_tuple;
        double weight;
    };

    struct ProjectionError {
        ProjectionError ( Eigen::Vector4d target_pt, Eigen::Vector2d projection, Eigen::Matrix4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight )
            : target_pt ( target_pt ), projection ( projection ), target_in_cam ( target_in_cam ), K ( K ), weight ( weight ), cam_in_body_old ( cam_in_body_old ) {}

        template <typename T>
        bool operator() ( const T *const p_t,
                          const T *const dx_dy_dtheta_vel_omega,
                          T *residuals ) const {
            T cam_in_body_new[16];
            T sp = sin ( p_t[0] );
            T st = sin ( p_t[1] );
            T cp = cos ( p_t[0] );
            T ct = cos ( p_t[1] );
            cam_in_body_new[0] = -sp;
            cam_in_body_new[1] = st * cp;
            cam_in_body_new[2] = cp * ct;
            cam_in_body_new[3] = T ( 0.0 );
            cam_in_body_new[4] = -cp;
            cam_in_body_new[5] = -st * sp;
            cam_in_body_new[6] = -ct * sp;
            cam_in_body_new[7] = T ( 0.0 );
            cam_in_body_new[8] = T ( 0.0 );
            cam_in_body_new[9] = -ct;
            cam_in_body_new[10] = st;
            cam_in_body_new[11] = T ( 0.0 );
            cam_in_body_new[12] = T ( 0.0 );
            cam_in_body_new[13] = T ( 0.0 );
            cam_in_body_new[14] = T ( 0.0 );
            cam_in_body_new[15] = T ( 1.0 );
            Eigen::Map<const Eigen::Matrix<T, 4, 4> > eigen_cam_in_body_new ( cam_in_body_new );

            T new_body_in_old_body[16];
            T sdtheta = sin ( dx_dy_dtheta_vel_omega[2] );
            T cdtheta = cos ( dx_dy_dtheta_vel_omega[2] );
            T dx = dx_dy_dtheta_vel_omega[0];
            T dy = dx_dy_dtheta_vel_omega[1];

            new_body_in_old_body[0] = cdtheta;
            new_body_in_old_body[1] = -sdtheta;
            new_body_in_old_body[2] =  T ( 0.0 );
            new_body_in_old_body[3] = dx;
            new_body_in_old_body[4] = sdtheta;
            new_body_in_old_body[5] = cdtheta;
            new_body_in_old_body[6] = T ( 0.0 );
            new_body_in_old_body[7] = dy;
            new_body_in_old_body[8] = T ( 0.0 );
            new_body_in_old_body[9] = T ( 0.0 );
            new_body_in_old_body[10] = T ( 1.0 );
            new_body_in_old_body[11] = T ( 0.0 );
            new_body_in_old_body[12] = T ( 0.0 );
            new_body_in_old_body[13] = T ( 0.0 );
            new_body_in_old_body[14] = T ( 0.0 );
            new_body_in_old_body[15] = T ( 1.0 );
            Eigen::Map<const Eigen::Matrix<T, 4, 4> > eigen_new_body_in_old_body ( new_body_in_old_body );


            Eigen::Matrix<T, 3, 4> TK = K.cast <T> ();
            Eigen::Matrix<T, 4, 4> Tcam_in_body_old = cam_in_body_old.cast<T> ();
            Eigen::Matrix<T, 4, 1> Ttarget_pt = target_pt.cast<T>();

            Eigen::Matrix<T, 3, 1> new_projection = TK * ( eigen_cam_in_body_new.inverse() * eigen_new_body_in_old_body.inverse()  * Tcam_in_body_old * Ttarget_pt );

            residuals[0] = T ( weight ) * ( new_projection ( 0 , 0 ) - TK ( 0 , 2 ) );
            residuals[1] = T ( weight ) * ( new_projection ( 1 , 0 ) - TK ( 1 , 2 ) );
            return true;
        }

        static ceres::CostFunction *Create ( Eigen::Vector4d target_pt, Eigen::Vector2d projection, Eigen::Matrix4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight ) {
            return ( new ceres::AutoDiffCostFunction<ProjectionError, 2, 5, 2> ( new ProjectionError ( target_pt, projection, target_in_cam, K, cam_in_body_old, weight ) ) );
        }

        Eigen::Vector4d target_pt;
        Eigen::Vector2d projection;
        Eigen::Matrix4d target_in_cam;
        Eigen::Matrix< double, 3, 4 > K;
        Eigen::Matrix4d cam_in_body_old;

        double weight;

    };

    ceres::Problem problem;
    double x1 = 0;
    double y1 = 0;
public:
    OptimizationProblem() {}
    ~OptimizationProblem() {}
    void addTagFactors ();
    void addRangeFactor ( RangeDataTuple &tuple );
    void generateData ( std::vector<RangeDataTuple> &gen_data );
    void optimizeGraph();
};
