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
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

struct Velocity {
    Velocity () {}
    Velocity ( double vel, double omega ) : vel ( vel ), omega ( omega ) {}
    double vel;
    double omega;
};

struct RobotState {
    RobotState ( double x, double y, double yaw, double vel, double omega ) : x ( x ), y ( y ), yaw ( yaw ), vel ( vel ), omega ( omega ) {}
    double x;
    double y;
    double yaw;
    double vel;
    double omega;
    void printState() {
        std::cout << std::setprecision ( 8 ) <<  "[" << x
                  << ", " << y
                  << ", " << yaw
                  << "] " << std::endl;
    }
};

struct DynamicWindow {
    DynamicWindow () {}
    DynamicWindow ( double vmin, double vmax, double yawrate_min, double yawrate_max ) : vmin ( vmin ), vmax ( vmax ), yawrate_min ( yawrate_min ), yawrate_max ( yawrate_max ) {}
    double vmin;
    double vmax;
    double yawrate_min;
    double yawrate_max;
    void printDW() {
        std::cout << std::setprecision ( 8 ) << "[" << vmin
                  << ", " << vmax
                  << ", " << yawrate_min
                  << ", " << yawrate_max
                  << "] " << std::endl;
    }
};

struct DynamicWindowParams {
    DynamicWindowParams() {}
    DynamicWindowParams ( double max_speed,
                          double min_speed,
                          double max_yawrate,
                          double max_accel,
                          double max_dyawrate,
                          double v_reso,
                          double yawrate_reso,
                          double dt,
                          double update_time ) : max_speed ( max_speed ), min_speed ( min_speed ), max_yawrate ( max_yawrate ), max_accel ( max_accel ),
        max_dyawrate ( max_dyawrate ), v_reso ( v_reso ), yawrate_reso ( yawrate_reso ), dt ( dt ), update_time ( update_time ) {}

    double max_speed = 1.0;
    double min_speed = -0.5;
    double max_yawrate = 40.0 * M_PI / 180.0;
    double max_accel = 0.2; //0.2
    double max_dyawrate = 40.0 * M_PI / 180.0;  //40
    double v_reso = 0.001;
    double yawrate_reso = 0.01 * M_PI / 180.0;
    double dt = 0.001;
    double update_time = 1.0;
    double robot_radius = 1.0;
};

class DynamicWindowSampler
{
private:
    void propagateMotion ( RobotState& state, Velocity& curr_velocity );
    void calcDynamicWindow ( const RobotState state, DynamicWindow& dynamic_window );
    void calcTrajectory ( RobotState& state, Velocity curr_velocity );
    DynamicWindowParams params_;

public:
    DynamicWindowSampler();
    ~DynamicWindowSampler();
    void getFeasibleSearchSpace ( RobotState& state, std::vector<RobotState>& feasible_states );
};
