//
//  Robot-phy-parameters.cpp
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

#include <iostream>

#include "Robot.h"

/**
 * This function sets the active spasm filter values of the robotical arm. This function is only used by rehab clients. Regular applications shouldn't use it.
 */
void Robot::setSpasmFilterValues(std::vector<float>command, int activationStatus)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetSpasmFilterValues)(c, activationStatus);
}

void Robot::setActuatorAdress(const int actualAdress, const int newAdress)
{
    (*MySetActuatorAdress)(actualAdress, newAdress);
}

/**
 * This function set the PID of all actuators.
 * @param newPID PID values
 */
void Robot::setPID(std::vector<std::vector<float> > newPID) {
	for (int actuatorNumber = 1; actuatorNumber < newPID.size() + 1; actuatorNumber++) {
		setPID(actuatorNumber, newPID[actuatorNumber - 1]);
	}
}

/**
 * This function set the PID of a specific actuator.
 * @param actuatorNumber joint number
 * @param newPID PID values
 */
void Robot::setPID(int actuatorNumber, std::vector<float> newPID) {
	float P, I, D;
	unsigned int actuatorPIDAddress;

	P = (float)newPID[0];
	I = (float)newPID[1];
	D = (float)newPID[2];
	if (P < MIN_PID || I < MIN_PID || D < MIN_PID || P > MAX_P || D > MAX_D) {
		std::cout << "Error : wrong PID values" << std::endl;
	} else {
		actuatorPIDAddress = FIRST_ACTUATOR_PID_ADDRESS + actuatorNumber - 1;
		(*MySetActuatorPID)(actuatorPIDAddress, P, I, D);
	}
}
