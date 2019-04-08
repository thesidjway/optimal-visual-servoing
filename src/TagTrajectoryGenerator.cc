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


#include <optimal_visual_servoing/TagTrajectoryGenerator.h>

TagTrajectoryGenerator::TagTrajectoryGenerator ( std::__cxx11::string params_file ) {
    readTrajectoryGenerationParams ( params_file );
    initializeRosPipeline();
}

TagTrajectoryGenerator::~TagTrajectoryGenerator() {

}

void TagTrajectoryGenerator::publishTrajectory ( double x, double y, double z ) {
    gazebo_msgs::ModelState state;
    state.model_name = params_.modelName;
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.reference_frame = params_.referenceFrame;
    trajectory_pub_.publish ( state );
}

void TagTrajectoryGenerator::readTrajectoryGenerationParams ( std::string params_file ) {
    YAML::Node config = YAML::LoadFile ( params_file );
    params_.x_min = config["trajectory"]["x_min"].as<double>();
    params_.x_max = config["trajectory"]["x_max"].as<double>();
    params_.y_min = config["trajectory"]["y_min"].as<double>();
    params_.y_max = config["trajectory"]["y_max"].as<double>();
    params_.z_min = config["trajectory"]["z_min"].as<double>();
    params_.z_max = config["trajectory"]["z_max"].as<double>();
    params_.num_bezier_pts = config["trajectory"]["num_bezier_pts"].as<int>();
    params_.num_traj_pts = config["trajectory"]["num_traj_pts"].as<int>();
    params_.publish_rate = config["trajectory"]["publish_rate"].as<int>();
    params_.trajTopic = config["trajectory"]["ros_topic"].as<std::string>();
    params_.modelName = config["trajectory"]["model_name"].as<std::string>();
    params_.referenceFrame = config["trajectory"]["reference_frame"].as<std::string>();
}


double TagTrajectoryGenerator::bernstein ( int n, int j, double t ) {
    return factorial ( n ) / ( factorial ( j ) *factorial ( n-j ) ) *pow ( t,j ) *pow ( ( 1-t ), ( n-j ) );
}

void TagTrajectoryGenerator::linspace ( std::vector< double >& t ) {
    for ( double i = 0 ; i < 1; i+= 1.0 / params_.num_traj_pts ) {
        t.push_back ( i );
    }
}

std::vector<double> TagTrajectoryGenerator::hamilton ( std::vector<double> quat1, std::vector<double> quat2 ) {
    std::vector<double> quat3;
    quat3.push_back ( 0 );
    quat3.push_back ( 0 );
    quat3.push_back ( 0 );
    quat3.push_back ( 1 );
    quat3[0] = quat1[3]*quat2[0] + quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1];
    quat3[1] = quat1[3]*quat2[1] - quat1[0]*quat2[2] + quat1[1]*quat2[3] + quat1[2]*quat2[0];
    quat3[2] = quat1[3]*quat2[2] + quat1[0]*quat2[1] - quat1[1]*quat2[0] + quat1[2]*quat2[3];
    quat3[3] = quat1[3]*quat2[3]-quat1[0]*quat2[0]-quat1[1]*quat2[1]-quat1[2]*quat2[2];
    return quat3;
}

void TagTrajectoryGenerator::bezier ( std::vector< std::vector< double > >& points, std::vector< double >& t, std::vector<std::vector<double>>& trajectory ) {
    for ( unsigned int i = 0; i < t.size(); i++ ) {
        std::vector<double> a;
        srand ( time ( NULL ) );
        a.push_back ( 0 );
        a.push_back ( 0 );
        a.push_back ( 0 );
        trajectory.push_back ( a );
        for ( unsigned int j = 0 ; j < points.size() ; j++ ) {
            for ( unsigned int k = 0 ; k < 3; k++ ) {
                trajectory[i][k] = trajectory[i][k] + points[j][k] * bernstein ( points.size() - 1, j, t[i] );
            }
        }
    }
}

void TagTrajectoryGenerator::generateNRandomPoints ( std::vector<std::vector<double>>& points ) {
    std::srand ( time ( NULL ) );
    for ( unsigned int i = 0 ; i < params_.num_bezier_pts ; i++ ) {
        if ( i == 0 ) {
            std::vector<double> a;
            a.push_back ( 0 );
            a.push_back ( 0 );
            a.push_back ( (params_.z_max + params_.z_min) / 2  );
            points.push_back ( a );
        } else {
            double temp1 = fRand ( params_.x_min , params_.x_max );
            double temp2 = fRand ( params_.y_min , params_.y_max );
            double temp3 = fRand ( params_.z_min , params_.z_max );
            std::vector<double> a;
            a.push_back ( temp1 );
            a.push_back ( temp2 );
            a.push_back ( temp3 );
            points.push_back ( a );
        }
    }
}

void TagTrajectoryGenerator::initializeRosPipeline() {
    trajectory_pub_ = n_.advertise<gazebo_msgs::ModelState> ( params_.trajTopic, 1000 );
}

std::vector<double> TagTrajectoryGenerator::normalize ( std::vector<double> vector_in ) {
    std::vector<double> tmp = vector_in;
    tmp[0] = vector_in[0]/sqrt ( pow ( vector_in[0], 2 ) + pow ( vector_in[1], 2 ) + pow ( vector_in[2], 2 ) );
    tmp[1] = vector_in[1]/sqrt ( pow ( vector_in[0], 2 ) + pow ( vector_in[1], 2 ) + pow ( vector_in[2], 2 ) );
    tmp[2] = vector_in[2]/sqrt ( pow ( vector_in[0], 2 ) + pow ( vector_in[1], 2 ) + pow ( vector_in[2], 2 ) );
    return tmp;
}

void mySigintHandler ( int sig ) {
    ros::shutdown();
    exit ( EXIT_FAILURE );
}

int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "tag_trajectory_generator" );
    TagTrajectoryGenerator generator ( "/home/thesidjway/research_ws/src/optimal-visual-servoing/params/optimal_visual_servoing.yaml" );
    std::vector<std::vector<double>> generated_points, trajectory;
    std::vector<double> t_vals;
    generator.generateNRandomPoints ( generated_points );
    generator.linspace ( t_vals );
    generator.bezier ( generated_points, t_vals, trajectory );
    ros::Rate loop_rate ( generator.params_.publish_rate );
    for ( unsigned int i = 1; i < trajectory.size(); i++ ) {
//         std::cout << "Trajectory: " << trajectory[i][0] << " " << trajectory[i][1] << " " << trajectory[i][2] << std::endl;
        generator.publishTrajectory ( trajectory[i][0], trajectory[i][1], trajectory[i][2] );
        signal ( SIGINT, mySigintHandler );
        ros::spinOnce();
        loop_rate.sleep();
    };
};
