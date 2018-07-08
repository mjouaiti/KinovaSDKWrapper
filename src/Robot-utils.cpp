//
//  Robot-utils.cpp
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
 * Get the angular value of a given joint
 * @param actuatorNumber joint number
 * @param overallInfo Angular informations for all joints
 * @return joint value (double)
 */
double Robot::getActuatorAngularInfo(int actuatorNumber, AngularInfo overallInfo) {
    switch(actuatorNumber) {
        case 1:
            return (double)overallInfo.Actuator1;
            break;
        case 2:
            return (double)overallInfo.Actuator2;
            break;
        case 3:
            return (double)overallInfo.Actuator3;
            break;
        case 4:
            return (double)overallInfo.Actuator4;
            break;
        case 5:
            return (double)overallInfo.Actuator5;
            break;
        case 6:
            return (double)overallInfo.Actuator6;
            break;
        default:
            return 0.0;
    }
}

/**
 * Set the angular value of a given joint
 * @param actuatorNumber joint number
 * @param newValue new joint value
 * @param overallInfo Angular informations for all joints
 * @return joint information (AngularInfo)
 */
AngularInfo Robot::setActuatorAngularValue(int actuatorNumber, double newValue, AngularInfo overallInfo)
{
    AngularInfo newInfo = overallInfo;
    newValue = (float)newValue;
    switch(actuatorNumber) {
        case 1:
            newInfo.Actuator1 = newValue;
            break;
        case 2:
            newInfo.Actuator2 = newValue;
            break;
        case 3:
            newInfo.Actuator3 = newValue;
            break;
        case 4:
            newInfo.Actuator4 = newValue;
            break;
        case 5:
            newInfo.Actuator5 = newValue;
            break;
        case 6:
            newInfo.Actuator6 = newValue;
            break;
        default:
            newInfo = overallInfo;
    }
    return newInfo;
}

/**
 * Converts a std::vector to Kinova data type AngularInfo
 * @param vectorizedData vector if joint values
 * @return joint information (AngularInfo)
 */
AngularInfo Robot::convertVectorToAngularInfo(std::vector<float> vectorizedData) {
    AngularInfo res;
    res.Actuator1 = (float)vectorizedData[0];
    res.Actuator2 = (float)vectorizedData[1];
    res.Actuator3 = (float)vectorizedData[2];
    res.Actuator4 = (float)vectorizedData[3];
    res.Actuator5 = (float)vectorizedData[4];
    res.Actuator6 = (float)vectorizedData[5];
    return res;
}

/**
 * Converts Kinova data type AngularInfo to a std::vector
 * @param overallData AngularPosition variable
 * @return vector of joint vakues (std::vector<float>)
 */
std::vector<float> Robot::convertAngularPositionToVector(AngularPosition overallData) {
    std::vector<float> vectorizedData;
    vectorizedData.push_back((double)overallData.Actuators.Actuator1);
    vectorizedData.push_back((double)overallData.Actuators.Actuator2);
    vectorizedData.push_back((double)overallData.Actuators.Actuator3);
    vectorizedData.push_back((double)overallData.Actuators.Actuator4);
    vectorizedData.push_back((double)overallData.Actuators.Actuator5);
    vectorizedData.push_back((double)overallData.Actuators.Actuator6);
    
    return vectorizedData;
}

