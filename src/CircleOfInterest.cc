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

#include <optimal_visual_servoing/CircleOfInterest.h>

CircleOfInterest::CircleOfInterest() {

}

CircleOfInterest::~CircleOfInterest() {

}


void CircleOfInterest::getCircleOfInterest ( Eigen::Vector3d& target_point, Circle& circle ) {
    if ( target_point ( 2 ) < sphere_radius_ ) {
        circle = Circle ( sqrt ( sphere_radius_ * sphere_radius_ - target_point ( 2 ) * target_point ( 2 ) ), target_point ( 0 ), target_point ( 1 ) );
    } else {
        circle = Circle ( 0, 1000, 1000 );
    }
}

