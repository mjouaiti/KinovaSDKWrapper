//
//  Robot-API.cpp
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
//  Created by Lancelot Caron.
//

#include "Robot.h"

/**
 * Start robot clock
 */
void Robot::startClock() {
    startTime = std::chrono::high_resolution_clock::now();
    currentTime = startTime;
}

/**
 * Update robot clock
 */
void Robot::updateClock() {
    currentTime = std::chrono::high_resolution_clock::now();
}

/**
 * Get robot time in microseconds
 * @return time
 */
double Robot::getMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(currentTime-startTime).count();
}

/**
 * Get robot time in seconds
 * @return time
 */
double Robot::getSeconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(currentTime-startTime).count() / 1000000.0;
}
