//
//  Robot.cpp
//
//  Kinova SDK Wrapper
//  Copyright (C) 2018  Université de Lorraine - CNRS
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//  Created by Melanie Jouaiti and Lancelot Caron.
//

#include <dlfcn.h>
#include <iostream>
#include <string>

#include "Robot.h"

const int Robot::ACTUATORS_COUNT = 6;
const int Robot::FIRST_ACTUATOR_PID_ADDRESS = 16;
const double Robot::MAX_P = 2.0;
const double Robot::MAX_D = 0.1;
const double Robot::MIN_PID = 0.0;
const double Robot::DEFAULT_P = 2.0;
const double Robot::DEFAULT_I = 0.0;
const double Robot::DEFAUTL_D = 0.05;

/**
 * Robot constructor
 * @param libPath path of Kinova.API.USBCommandLayerUbuntu.so
 * @param optionFile path of the setup file
 */
Robot::Robot(const std::string& libPath)
{
	commandLayerHandle = dlopen(libPath.c_str(), RTLD_NOW|RTLD_GLOBAL);

	if (commandLayerHandle == NULL) {
        std::cout << "Error while loading library:" << libPath << std::endl;
	}
    
    this->initializeAPI();
	this->initializePositionTrajectory();
	this->initializeVelocityTrajectory();
}

/**
 * Robot destructor
 */
Robot::~Robot()
{
    closeAPI();
}
