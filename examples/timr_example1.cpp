/** 
 *     This file is part of timr.
 *  
 *     timr is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     timr is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with timr. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file examples/dqexample1.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include "timr.hpp"
#include <array>

int main() {
using scalar_t = float;

    const std::string json_path("../examples/robot.json");
    const std::array<float, 6> init_joint_positions {0,0,0,0,0,0};

    timr::SerialManipulator<float, 6> robot(json_path, init_joint_positions);

    timr::Posef x_init = robot.end_pose();
    timr::Tranf t_init = x_init.translation();
    timr::Rotf r_init = x_init.rotation();

    float radius = 0.01;
    float rad_speed = 0.001;
    for (size_t i=0; ; ++i)
    {   
        timr::Tranf td = t_init + timr::Tranf(-0.02, radius * cos(rad_speed*i), radius * sin(rad_speed*i) + 0.05);
        timr::Posef xd(r_init, td);

        robot.update(xd);

        std::cout << (robot.end_pose().translation() - td).norm() << "\n";
    }

}             