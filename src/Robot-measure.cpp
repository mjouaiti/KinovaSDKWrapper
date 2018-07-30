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
 * Gets the acceleration of each actuator
 * @return acceleration of each actuator (std::vector<std::vector<float>>)
 */
std::vector<std::vector<float>> Robot::getActuatorAcceleration()
{
    AngularAcceleration data;
    std::vector<std::vector<float>> acc;
    data.InitStruct();
    (*MyGetActuatorAcceleration)(data);
    acc.push_back({data.Actuator1_X, data.Actuator1_Y, data.Actuator1_Z});
    acc.push_back({data.Actuator2_X, data.Actuator2_Y, data.Actuator2_Z});
    acc.push_back({data.Actuator3_X, data.Actuator3_Y, data.Actuator3_Z});
    acc.push_back({data.Actuator4_X, data.Actuator4_Y, data.Actuator4_Z});
    acc.push_back({data.Actuator5_X, data.Actuator5_Y, data.Actuator5_Z});
    acc.push_back({data.Actuator6_X, data.Actuator6_Y, data.Actuator6_Z});
    return acc;
}

/**
 * This function returns the actual torque command of each actuator.
 * @return The angular torque command(std::vector<float>)
 */
std::vector<float> Robot::getAngularTorqueCommand()
{
    float result[COMMAND_SIZE];
    (*MyGetAngularTorqueCommand)(result);
    std::vector<float> torque(std::begin(result), std::end(result));
    return torque;
}

/**
 * This function returns the actual estimation of the gravity torques.
 * @return The gravity torques (std::vector<float>)
 */
std::vector<float> Robot::getAngularTorqueGravityEstimation()
{
    float result[GRAVITY_PARAM_SIZE];
    (*MyGetAngularTorqueGravityEstimation)(result);
    std::vector<float> torque(std::begin(result), std::end(result));
    return torque;
}

/**
 * This function gets the end effector offset's parameters. The end effector's offset is a translation offset, in meters, applied to the end effector of the robotic arm.
 * @param status indicates if the offset is applied or not (0 = not applied, 1 = applied).
 * @return the offset (std::vector<float>)
 */
std::vector<float> Robot::getEndEffectorOffset(unsigned int &status)
{
    float x, y, z;
    (*MyGetEndEffectorOffset)(status, x, y, z);
    return {x, y, z};
}

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
 * This function returns the torque, without gravity, of each actuator. Unit is Newton meter [N * m].
 * @return (std::vector<float>)
 */
std::vector<float> Robot::getAngularForceGravityFree()
{
    (*MyGetAngularForceGravityFree)(angularForceGravityFree);
	return convertAngularPositionToVector(angularForceGravityFree);
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
 * calibrates angular force gravity free
 * @param numberOfIterations number of iterations
 */
std::vector<float> Robot::calibrateAngularForceGravityFree(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularForceGravityFree, numberOfIterations);
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
 * @param MyGetAngularData can be MyGetAngularVelocity, MyGetAngularPosition, MyGetAngularForceGravityFree, MyGetAngularForce, MyGetAngularCurrent, MyGetAngularCurrentMotor
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

std::string getGripperStatus()
{
    Gripper gripper;
    int result = (*MyGetGripperStatus)(gripper);
    std::string str = "";
    str << "Gripper's model : " << gripper.Model << endl;
    for(int i = 0; i < 3; i++)
    {
        str << "Finger #" << (i + 1) << endl;
        str << "Finger #" << (i + 1) << "'s actual acceleration : " << gripper.Fingers[i].ActualAcceleration << endl;
        str << "Finger #" << (i + 1) << "'s actual average current : " << gripper.Fingers[i].ActualAverageCurrent << endl;
        str << "Finger #" << (i + 1) << "'s actual command : " << gripper.Fingers[i].ActualCommand << endl;
        str << "Finger #" << (i + 1) << "'s actual current : " << gripper.Fingers[i].ActualCurrent << endl;
        str << "Finger #" << (i + 1) << "'s actual force : " << gripper.Fingers[i].ActualForce << endl;
        str << "Finger #" << (i + 1) << "'s actual position : " << gripper.Fingers[i].ActualPosition << endl;
        str << "Finger #" << (i + 1) << "'s actual speed : " << gripper.Fingers[i].ActualSpeed << endl;
        str << "Finger #" << (i + 1) << "'s actual temperature : " << gripper.Fingers[i].ActualTemperature << endl;
        str << "Finger #" << (i + 1) << "'s code version : " << gripper.Fingers[i].CodeVersion << endl;
        str << "Finger #" << (i + 1) << "'s communication errors : " << gripper.Fingers[i].CommunicationErrors << endl;
        str << "Finger #" << (i + 1) << "'s cycle count : " << gripper.Fingers[i].CycleCount << endl;
        str << "Finger #" << (i + 1) << "'s device ID : " << gripper.Fingers[i].DeviceID << endl;
        str << "Finger #" << (i + 1) << "'s finger address : " << gripper.Fingers[i].FingerAddress << endl;
        str << "Finger #" << (i + 1) << "'s ID : " << gripper.Fingers[i].ID << endl;
        str << "Finger #" << (i + 1) << "'s index : " << gripper.Fingers[i].Index << endl;
        str << "Finger #" << (i + 1) << "'s is finger connected : " << gripper.Fingers[i].IsFingerConnected << endl;
        str << "Finger #" << (i + 1) << "'s is finger init : " << gripper.Fingers[i].IsFingerInit << endl;
        str << "Finger #" << (i + 1) << "'s max acceleration : " << gripper.Fingers[i].MaxAcceleration << endl;
        str << "Finger #" << (i + 1) << "'s max angle : " << gripper.Fingers[i].MaxAngle << endl;
        str << "Finger #" << (i + 1) << "'s max current : " << gripper.Fingers[i].MaxCurrent << endl;
        str << "Finger #" << (i + 1) << "'s max force : " << gripper.Fingers[i].MaxForce << endl;
        str << "Finger #" << (i + 1) << "'s max speed : " << gripper.Fingers[i].MaxSpeed << endl;
        str << "Finger #" << (i + 1) << "'s min angle : " << gripper.Fingers[i].MinAngle << endl;
        str << "Finger #" << (i + 1) << "'s oscillator tuning value : " << gripper.Fingers[i].OscillatorTuningValue << endl;
        str << "Finger #" << (i + 1) << "'s peak current : " << gripper.Fingers[i].PeakCurrent << endl;
        str << "Finger #" << (i + 1) << "'s peak max temp : " << gripper.Fingers[i].PeakMaxTemp << endl;
        str << "Finger #" << (i + 1) << "'s peak min temp : " << gripper.Fingers[i].PeakMinTemp << endl;
        str << "Finger #" << (i + 1) << "'s run time : " << gripper.Fingers[i].RunTime << endl << endl;
    }

}
