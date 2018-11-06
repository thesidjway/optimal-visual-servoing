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
#include <iostream>

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

struct LineSegmentDataTuple {
    LineSegmentDataTuple () {}
    LineSegmentDataTuple ( double x_frontal, double y_frontal, double x_distal, double y_distal ) : x_frontal ( x_frontal ), y_frontal ( y_frontal ), x_distal ( x_distal ), y_distal ( y_distal ) {}
    double x_frontal;
    double y_frontal;
    double x_distal;
    double y_distal;
    void printLineSegmentDataTuple () {
        std::cout << " x_frontal : " << x_frontal << ", y_frontal : " << y_frontal << " , x_distal : " << x_distal << " , y_distal : " << y_distal << std::endl;
    }
};

struct PTZCommand {
    PTZCommand() {}
    PTZCommand ( double pan, double tilt, double zoom ) : pan ( pan ), tilt ( tilt ), zoom ( zoom ) {}
    double pan;
    double tilt;
    double zoom;
};


struct RangeError {
    RangeError ( RangeDataTuple data_tuple, double weight )
        : data_tuple ( data_tuple ), weight ( weight ) {}
    template <typename T>
    bool operator() ( const T *const dx_dy_dtheta_vel_omega,
                      T *residuals ) const {
        residuals[0] = exp ( sqrt ( T ( dx_dy_dtheta_vel_omega[0] - data_tuple.median_dist * cos ( data_tuple.bearing ) ) * T ( dx_dy_dtheta_vel_omega[0] - data_tuple.median_dist * cos ( data_tuple.bearing ) ) +
                                    T ( dx_dy_dtheta_vel_omega[1] - data_tuple.median_dist * sin ( data_tuple.bearing ) ) * T ( dx_dy_dtheta_vel_omega[1] - data_tuple.median_dist * sin ( data_tuple.bearing ) ) ) ) *
                       ( ( dx_dy_dtheta_vel_omega[0] - T ( 0.0 ) ) * ( dx_dy_dtheta_vel_omega[0] - T ( 0.0 ) )  + ( dx_dy_dtheta_vel_omega[1] - T ( 0.0 ) )  * ( dx_dy_dtheta_vel_omega[1] - T ( 0.0 ) ) - data_tuple.median_dist * data_tuple.median_dist ) ;
        return true;
    }
    static ceres::CostFunction *Create ( RangeDataTuple data_tuple, double weight ) {
        return ( new ceres::AutoDiffCostFunction<RangeError, 1, 5> ( new RangeError ( data_tuple, weight ) ) );
    }
    RangeDataTuple data_tuple;
    double weight;
};

struct VelocityConstraints { //TO BE MODIFIED
    VelocityConstraints ( double max_vel, double max_omega, double dt )
        : max_vel ( max_vel ), max_omega ( max_omega ), dt ( dt ) {}

    template <typename T>
    bool operator () ( const T *const dx_dy_dtheta_vel_omega,
                       T *residuals ) const {
        residuals[0] = T ( max_vel * max_vel - dx_dy_dtheta_vel_omega[0] / dt * dx_dy_dtheta_vel_omega[0] / dt - dx_dy_dtheta_vel_omega[1] / dt * dx_dy_dtheta_vel_omega[1] / dt );
        residuals[1] = T ( max_omega * max_omega - dx_dy_dtheta_vel_omega[2] / dt * dx_dy_dtheta_vel_omega[2] / dt );
        return true;
    }
    static ceres::CostFunction *Create ( double max_vel, double max_omega, double dt ) {
        return ( new ceres::AutoDiffCostFunction<VelocityConstraints, 2, 5> ( new VelocityConstraints ( max_vel, max_omega, dt ) ) );
    }
    double max_vel;
    double max_omega;
    double dt;
};

struct PanTiltChangeError {
    PanTiltChangeError ( double last_p, double last_t, double weight_p, double weight_t )
        : last_p ( last_p ), last_t ( last_t ), weight_p ( weight_p ), weight_t ( weight_t ) {}

    template <typename T>
    bool operator () ( const T *const p_t,
                       T *residuals ) const {
        residuals[0] = T ( weight_p ) * ( p_t[0] - T ( last_p ) );
        residuals[1] = T ( weight_t ) * ( p_t[1] - T ( last_t ) );
        return true;
    }

    static ceres::CostFunction *Create ( double last_p, double last_t, double weight_p, double weight_t ) {
        return ( new ceres::AutoDiffCostFunction<PanTiltChangeError, 2, 2> ( new PanTiltChangeError ( last_p, last_t, weight_p, weight_t ) ) );
    }
    double last_p;
    double last_t;
    double weight_p;
    double weight_t;
};

struct DistanceError {
    DistanceError ( Eigen::Vector4d target_in_cam, Eigen::Matrix4d cam_in_body_old, double weight, double desired_distance )
        : target_in_cam ( target_in_cam ), cam_in_body_old ( cam_in_body_old ), weight ( weight ), desired_distance ( desired_distance ) {}

    template <typename T>
    bool operator() ( const T *const dx_dy_dtheta_vel_omega,
                      T *residuals ) const {

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
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor> > eigen_new_body_in_old_body ( new_body_in_old_body );


        Eigen::Matrix<T, 4, 4> Tcam_in_body_old = cam_in_body_old.cast<T> ();
        Eigen::Matrix<T, 4, 1> Ttarget_in_cam = target_in_cam.cast<T>();
        Eigen::Matrix<T, 4, 1> target_in_body_new = eigen_new_body_in_old_body.inverse() * Tcam_in_body_old * Ttarget_in_cam;

        residuals[0] = T ( weight ) * T ( target_in_body_new.norm() ) - T ( desired_distance );
        return true;
    }

    static ceres::CostFunction *Create ( Eigen::Vector4d target_in_cam, Eigen::Matrix4d cam_in_body_old, double weight, double desired_distance ) {
        return ( new ceres::AutoDiffCostFunction<DistanceError, 1, 5> ( new DistanceError ( target_in_cam, cam_in_body_old, weight, desired_distance ) ) );
    }

    Eigen::Vector4d target_in_cam;
    Eigen::Matrix4d cam_in_body_old;
    double weight;
    double desired_distance;
};

struct ProjectionErrorPTOnly {
    ProjectionErrorPTOnly ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight )
        : target_in_cam ( target_in_cam ), K ( K ), weight ( weight ), cam_in_body_old ( cam_in_body_old ) {}

    template <typename T>
    bool operator() ( const T *const p_t,
                      T *residuals ) const {
        T cam_in_body_new[16];
        T sp = sin ( p_t[0] );
        T st = sin ( p_t[1] );
        T cp = cos ( p_t[0] );
        T ct = cos ( p_t[1] );
        cam_in_body_new[0] = -sp;
        cam_in_body_new[1] = st * cp;
        cam_in_body_new[2] = cp * ct;
        cam_in_body_new[3] = T ( 0.19 );
        cam_in_body_new[4] = -cp;
        cam_in_body_new[5] = -st * sp;
        cam_in_body_new[6] = -ct * sp;
        cam_in_body_new[7] = T ( 0.0 );
        cam_in_body_new[8] = T ( 0.0 );
        cam_in_body_new[9] = -ct;
        cam_in_body_new[10] = st;
        cam_in_body_new[11] = T ( 0.395 );
        cam_in_body_new[12] = T ( 0.0 );
        cam_in_body_new[13] = T ( 0.0 );
        cam_in_body_new[14] = T ( 0.0 );
        cam_in_body_new[15] = T ( 1.0 );
        Eigen::Map<const Eigen::Matrix < T, 4, 4, Eigen::RowMajor> > Tcam_in_body_new ( cam_in_body_new );

        T new_body_in_old_body[16];
        for ( uint i = 0 ; i < 16 ; i++ ) {
            if ( i%5 == 0 ) {
                new_body_in_old_body[i] = T ( 1.0 );
            } else {
                new_body_in_old_body[i] = T ( 0.0 );
            }
        }

        Eigen::Map<const Eigen::Matrix < T, 4, 4, Eigen::RowMajor > > eigen_new_body_in_old_body ( new_body_in_old_body );


        Eigen::Matrix<T, 3, 4> TK = K.cast <T> ();
        Eigen::Matrix<T, 4, 4> Tcam_in_body_old = cam_in_body_old.cast<T> ();
        Eigen::Matrix<T, 4, 1> Ttarget_in_cam = target_in_cam.cast<T>();

//         std::cout << Tcam_in_body_old ( 0,0 ) << " " << Tcam_in_body_old ( 0,1 ) << " " << Tcam_in_body_old ( 0,2 ) << " "<< Tcam_in_body_old ( 0,3 ) << std::endl
//                   << Tcam_in_body_old ( 1,0 ) << " " << Tcam_in_body_old ( 1,1 ) << " " << Tcam_in_body_old ( 1,2 ) << " "<< Tcam_in_body_old ( 1,3 ) << std::endl
//                   << Tcam_in_body_old ( 2,0 ) << " " << Tcam_in_body_old ( 2,1 ) << " " << Tcam_in_body_old ( 2,2 ) << " "<< Tcam_in_body_old ( 2,3 ) << std::endl
//                   << Tcam_in_body_old ( 3,0 ) << " " << Tcam_in_body_old ( 3,1 ) << " " << Tcam_in_body_old ( 3,2 ) << " "<< Tcam_in_body_old ( 3,3 ) << std::endl << std::endl;

        Eigen::Matrix<T, 3, 1> new_projection = TK * ( Tcam_in_body_new.inverse() * Tcam_in_body_old * Ttarget_in_cam ) / Ttarget_in_cam ( 2 , 0 );

        std::cout << "SOUL 1: " << new_projection ( 0 , 0 ) - TK ( 0 , 2 ) << std::endl;
        std::cout << "SOUL 2: " << new_projection ( 1 , 0 ) - TK ( 1 , 2 ) << std::endl;
        residuals[0] = T ( weight ) * ( new_projection ( 1 , 0 ) - TK ( 1 , 2 ) );
        residuals[1] = T ( weight ) * ( new_projection ( 0 , 0 ) - TK ( 0 , 2 ) );

        return true;
    }

    static ceres::CostFunction *Create ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight ) {
        return ( new ceres::AutoDiffCostFunction<ProjectionErrorPTOnly, 2, 2> ( new ProjectionErrorPTOnly ( target_in_cam, K, cam_in_body_old, weight ) ) );
    }

    Eigen::Vector4d target_in_cam;
    Eigen::Matrix< double, 3, 4 > K;
    Eigen::Matrix4d cam_in_body_old;
    double weight;
};


struct ProjectionError {
    ProjectionError ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight )
        : target_in_cam ( target_in_cam ), K ( K ), weight ( weight ), cam_in_body_old ( cam_in_body_old ) {}

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
        cam_in_body_new[3] = T ( 0.19 );
        cam_in_body_new[4] = -cp;
        cam_in_body_new[5] = -st * sp;
        cam_in_body_new[6] = -ct * sp;
        cam_in_body_new[7] = T ( 0.0 );
        cam_in_body_new[8] = T ( 0.0 );
        cam_in_body_new[9] = -ct;
        cam_in_body_new[10] = st;
        cam_in_body_new[11] = T ( 0.395 );
        cam_in_body_new[12] = T ( 0.0 );
        cam_in_body_new[13] = T ( 0.0 );
        cam_in_body_new[14] = T ( 0.0 );
        cam_in_body_new[15] = T ( 1.0 );
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor> > eigen_cam_in_body_new ( cam_in_body_new );

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

        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor> > eigen_new_body_in_old_body ( new_body_in_old_body );


        Eigen::Matrix<T, 3, 4> TK = K.cast <T> ();
        Eigen::Matrix<T, 4, 4> Tcam_in_body_old = cam_in_body_old.cast<T> ();
        Eigen::Matrix<T, 4, 1> Ttarget_in_cam = target_in_cam.cast<T>();

        Eigen::Matrix<T, 3, 1> new_projection = TK * ( eigen_cam_in_body_new.inverse() * eigen_new_body_in_old_body.inverse()  * Tcam_in_body_old * Ttarget_in_cam );

        residuals[0] = T ( weight ) * ( new_projection ( 0 , 0 ) - TK ( 0 , 2 ) );
        residuals[1] = T ( weight ) * ( new_projection ( 1 , 0 ) - TK ( 1 , 2 ) );
        return true;
    }

    static ceres::CostFunction *Create ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight ) {
        return ( new ceres::AutoDiffCostFunction<ProjectionError, 2, 5, 2> ( new ProjectionError ( target_in_cam, K, cam_in_body_old, weight ) ) );
    }

    Eigen::Vector4d target_in_cam;
    Eigen::Matrix< double, 3, 4 > K;
    Eigen::Matrix4d cam_in_body_old;
    double weight;
};
