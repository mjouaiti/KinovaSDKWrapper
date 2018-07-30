//
//  Robot-measure.cpp
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
 * Returns the number of trajectories
 * @return number of trajectories (int)
 */
int Robot::getTrajectoryCount() {
    return trajectoryInfo.TrajectoryCount;
}
/**
 * Returns the values of the spasm filter
 * @return values of the spasm filter (std::vector<float>)
 */
std::vector<float> Robot::getSpasmFilterValues(int &activationStatus)
{
    float result[SPASM_FILTER_COUNT];
    (*MyGetSpasmFilterValues)(result, activationStatus);
    std::vector<float> values(std::begin(result), std::end(result));
    return values;
}

/*-------------------- Update --------------*/

/**
 * This function returns information about the trajectories FIFO stored inside the robotical arm.
 */
void Robot::updateTrajectoryInfo() {
    (*MyGetGlobalTrajectoryInfo)(trajectoryInfo);
}

/*----------- Get joints values ---------------*/
/**
 * This function returns the current that each actuator consumes on the main power supply. Unit is Amperes.
 * @return current that each actuator consumes on the main power supply (std::vector<float>)
 */
std::vector<float> Robot::getAngularCurrent()
{
    (*MyGetAngularCurrent)(angularCurrent);
	return convertAngularPositionToVector(angularCurrent);
}

/**
 * This function gets motor's current for each actuator Unit is Amperes.
 * @return motor's current for each actuator (std::vector<float>)
 */
std::vector<float> Robot::getAngularCurrentMotor()
{
    (*MyGetAngularCurrentMotor)(angularCurrentMotor);
	return convertAngularPositionToVector(angularCurrentMotor);
}

/**
 * This function returns the torque of each actuator. Unit is Newton meter [N * m].
 * @return torque of each actuator (std::vector<float>)
 */
std::vector<float> Robot::getAngularForce()
{
    (*MyGetAngularForce)(angularForce);
	return convertAngularPositionToVector(angularForce);
}

/**
 * This function returns the angular position of the robotical arm's end effector. Units are in degrees.
 * @return angular position of the robotical arm's end effector (std::vector<float>)
 */
std::vector<float> Robot::getAngularPosition()
{
    (*MyGetAngularPosition)(angularPosition);
	return convertAngularPositionToVector(angularPosition);
}

/**
 * This function gets the velocity of each actuator. Units are degrees / second.
 * @return velocity of each actuator (std::vector<float>)
 */
std::vector<float> Robot::getAngularVelocity()
{
    (*MyGetAngularVelocity)(angularVelocity);
	return convertAngularPositionToVector(angularVelocity);
}

/**
 * This function returns the cartesian position of the robotical arm's end effector. The orientation is defined by Euler angles (convention XYZ).
 * @return the cartesian position of the robotical arm's end effector (std::vector<float>)
 */
std::vector<float> Robot::getCartesianPosition()
{
    (*MyGetCartesianPosition)(cartesianPosition);
	std::vector<float> coordinates(3);
	coordinates[0] = (double)cartesianPosition.Coordinates.X;
	coordinates[1] = (double)cartesianPosition.Coordinates.Y;
	coordinates[2] = (double)cartesianPosition.Coordinates.Z;
	return coordinates;
}

/**
 * This function returns the cartesian force at the robotical arm's end effector. The translation unit is in Newtons and the orientation unit is Newton meters [N * m].
 * @return the cartesian force at the robotical arm's end effector (std::vector<float>)
 */
std::vector<float> Robot::getCartesianForce()
{
    (*MyGetCartesianForce)(cartesianForce);
	std::vector<float> force(3);
	force[0] = (double)cartesianForce.Coordinates.X;
	force[1] = (double)cartesianForce.Coordinates.Y;
	force[2] = (double)cartesianForce.Coordinates.Z;
	return force;
}

/*------------------ Calibrate ----------------*/
/**
 * calibrates angular current
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularCurrent(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularCurrent, numberOfIterations);
}

/**
 * calibrates angular current motor
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularCurrentMotor(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularCurrentMotor, numberOfIterations);
}

/**
 * calibrates angular force
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularForce(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularForce, numberOfIterations);
}

/**
 * calibrates angular position
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularPosition(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularPosition, numberOfIterations);
}

/**
 * calibrates angular position
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularVelocity(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularVelocity, numberOfIterations);
}

/**
 * Template calibration function
 * @param MyGetAngularData can be MyGetAngularVelocity, MyGetAngularPosition, MyGetAngularForce, MyGetAngularCurrent, MyGetAngularCurrentMotor
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularData(int (*MyGetAngularData)(AngularPosition &), int numberOfIterations) {
	AngularPosition currentData;
	std::vector<float> vectorizedData(ACTUATORS_COUNT);

	std::vector<float> mean(ACTUATORS_COUNT, 0);
	for (int i = 1; i <= numberOfIterations; i++) {
		(*MyGetAngularData)(currentData);
		vectorizedData = convertAngularPositionToVector(currentData);
		for (int j = 0; j < ACTUATORS_COUNT; j++) {
			mean[j] = vectorizedData[j] / (double)i + (double)(i-1) / (double)i * mean[j];
		}
	}

	return mean;
}
