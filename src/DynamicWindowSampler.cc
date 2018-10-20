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

#include <optimal_visual_servoing/DynamicWindowSampler.h>

DynamicWindowSampler::DynamicWindowSampler() {

}

DynamicWindowSampler::~DynamicWindowSampler() {

}


void DynamicWindowSampler::propagateMotion ( RobotState &state, Velocity &curr_velocity ) {
    state.yaw += curr_velocity.omega * params_.dt;
    state.x += curr_velocity.vel * cos ( state.yaw ) * params_.dt;
    state.y += curr_velocity.vel * sin ( state.yaw ) * params_.dt;
    state.vel = curr_velocity.vel;
    state.omega = curr_velocity.omega;
}

void DynamicWindowSampler::calcTrajectory ( RobotState &state, Velocity curr_velocity ) {
    propagateMotion ( state, curr_velocity );
    //state.printState();
}

void DynamicWindowSampler::calcDynamicWindow ( const RobotState state, DynamicWindow &dynamic_window ) {
    DynamicWindow Vs = DynamicWindow ( params_.min_speed, params_.max_speed,
                                       -params_.max_yawrate, params_.max_yawrate );

    DynamicWindow Vd = DynamicWindow ( state.vel - params_.max_accel * params_.dt,
                                       state.vel + params_.max_accel * params_.dt,
                                       state.omega - params_.max_dyawrate * params_.dt,
                                       state.omega + params_.max_dyawrate * params_.dt );

    dynamic_window = DynamicWindow ( std::max ( Vs.vmin, Vd.vmin ), std::min ( Vs.vmax, Vd.vmax ),
                                     std::max ( Vs.yawrate_min, Vd.yawrate_min ), std::min ( Vs.yawrate_max, Vd.yawrate_max ) );

}


void DynamicWindowSampler::getFeasibleSearchSpace ( RobotState &state, std::vector<RobotState> &feasible_states ) {
    DynamicWindow dynamic_window;
    calcDynamicWindow ( state, dynamic_window );
    for ( double i = dynamic_window.vmin ; i < dynamic_window.vmax ; i += params_.v_reso ) {
        for ( double j = dynamic_window.yawrate_min ; j < dynamic_window.yawrate_max ; j += params_.yawrate_reso ) {
            calcTrajectory ( state, Velocity ( i , j ) );
            feasible_states.push_back ( state );
        }
    }
}



