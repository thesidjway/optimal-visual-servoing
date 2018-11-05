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

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelState.h>
#include <signal.h>
#include <yaml-cpp/yaml.h>

struct trajectory_generation_params {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
    int num_bezier_pts;
    int num_traj_pts;
    int publish_rate;
    std::string trajTopic;
    std::string modelName;
    std::string referenceFrame;
};

class TagTrajectoryGenerator
{
private:
    ros::NodeHandle n_;
    ros::Publisher trajectory_pub_;
    double bernstein ( int n, int j, double t );
    std::vector<double> normalize ( std::vector<double> vector_in );
    std::vector<double> hamilton ( std::vector<double> quat1, std::vector<double> quat2 );
    void readTrajectoryGenerationParams ( std::string params_file );
    void initializeRosPipeline();

    inline unsigned int factorial ( unsigned int n ) {
        if ( n == 1 || n == 0 ) {
            return 1;
        } else {
            return n * factorial ( n - 1 );
        }
    }

    inline double fRand ( double fMin, double fMax ) {
        double f = ( double ) rand() / RAND_MAX;
        return fMin + f * ( fMax - fMin );
    }

public:
    TagTrajectoryGenerator ( std::string params_file );
    ~TagTrajectoryGenerator();
    void generateNRandomPoints ( std::vector<std::vector<double>>& points );
    void bezier ( std::vector<std::vector<double>>& points, std::vector<double>& t, std::vector<std::vector<double>>& trajectory );
    void linspace ( std::vector<double>& t );
    trajectory_generation_params params_;
    void publishTrajectory ( double x, double y, double z );
};
