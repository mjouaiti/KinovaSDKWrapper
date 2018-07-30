//
//  Robot.h
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

#include "Robot.h"

/**
 * This function sets the robotical arm in angular control mode.
 */
void Robot::setAngularControl() {
	(*MySetAngularControl)();
}

/**
 * This function sets the robotical arm in cartesian control mode (if possible).
 */
void Robot::setCartesianControl() {
	(*MySetCartesianControl)();
}

/**
 * This function sets new control mapping charts to the robotical arm.
 * @param command The new control mapping charts.
 */
void Robot::setControlMapping(ControlMappingCharts command)
{
    (*MySetControlMapping)(command);
}

void Robot::initFingers()
{
    (*MyInitFingers)();
}
