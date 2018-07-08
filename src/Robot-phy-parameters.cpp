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
 * This function sets the angular inertia and damping value.
 * @param inertia A struct that contains all angular inertia values. (Unit: [(Kg * m^2) / degree])
 * @param damping A struct that contains all angular damping values. (Unit: [(N * s) / degree])
 */
void Robot::setInertiaDamping(std::vector<float> inertia, std::vector<float> damping)
{
    AngularInfo commandInertia;
    AngularInfo commandDamping;
    commandInertia.InitStruct();
    commandDamping.InitStruct();
    commandDamping.Actuator1 = damping[0];
    commandDamping.Actuator2 = damping[1];
    commandDamping.Actuator3 = damping[2];
    commandDamping.Actuator4 = damping[3];
    commandDamping.Actuator5 = damping[4];
    commandDamping.Actuator6 = damping[5];
    commandInertia.Actuator1 = inertia[0];
    commandInertia.Actuator2 = inertia[1];
    commandInertia.Actuator3 = inertia[2];
    commandInertia.Actuator4 = inertia[3];
    commandInertia.Actuator5 = inertia[4];
    commandInertia.Actuator6 = inertia[5];
    (*MySetAngularInertiaDamping)(commandInertia, commandDamping);
}

/**
 * This function sets the cartesian inertia and damping value.
 *@param inertia A struct that contains all cartesian inertia values. (Translation unit: [Kg] - Orientation unit: [Kg * m^2])
 *@param damping A struct that contains all cartesian damping values. (Translation unit: [(N * s) / m] - Orientation unit: [(N * s) / Rad])
 */
void Robot::setCartesianInertiaDamping(std::vector<float> inertia, std::vector<float> damping)
{
    CartesianInfo commandInertia;
    CartesianInfo commandDamping;
    commandInertia.InitStruct();
    commandDamping.InitStruct();
    commandDamping.ThetaX = damping[0];
    commandDamping.ThetaY = damping[1];
    commandDamping.ThetaZ = damping[2];
    commandInertia.ThetaX = inertia[0];
    commandInertia.ThetaY = inertia[1];
    commandInertia.ThetaZ = inertia[2];
    (*MySetCartesianInertiaDamping)(commandInertia, commandDamping);
}

/**
 * This function sets the delta position over an actuator position limit where the virtual wall begins. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The gain of each actuator.
 */
void Robot::setPositionLimitDistance(std::vector<float>command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetPositionLimitDistance)(c);
}

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

/*---------------- Gravity --------------*/

/**
 * This function is used to run a sequence to estimate the optimal gravity parameters when the robot is standing (Z). The arm must be in Trajectory-Position mode before this function is called. Before using this procedure, you should make sure that the torque sensors are all well calibrated. This procedure is explained in the user guide and in the Advanced Specification Guide. When the program is launched, the robot will execute a trajectory. The user must remain alert and turn the robot off if something wrong occurs (for example if the robot collides with an object). When the program ends, it will output the parameters in the console and in a text file named “ParametersOptimal_Z.txt” in the program folder. These parameters can then be sent as input to the function SetOptimalZParam().
 * @param type Robot type (MICO_6DOF_SERVICE)
 */
std::vector<float> Robot::runGravityZEstimationSequence(ROBOT_TYPE type)
{
    (*MySwitchTrajectoryTorque)(POSITION);
    double OptimalzParam[OPTIMAL_Z_PARAM_SIZE];
    (*MyRunGravityZEstimationSequence)(type, OptimalzParam);
    return std::vector<float>(std::begin(OptimalzParam), std::end(OptimalzParam));
}

/**
 * This function sets the gravity parameters using the manual mode. For Mico and Jaco, the input is a vector of 42 floats. The parameters are: [m1,m2,m3,m4,m5,m6,x1,x2,x3,x4,x5,x6,y1,y2,y3,y4,y5,y6,z1,z2,z3,z4,z5,z6] where m is the links mass, and x,y,z is the links center of mass position. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command parameters vector of size 42
 */
void Robot::setGravityManualInputParam(std::vector<float> command)
{
    if(command.size() != 42)
    {
        std::cout << "WARNING:: setGravityManualInputParam requires a vector of length 42." << std::endl;
        std::cout << "\tExiting the function now..." << std::endl;
        return;
    }
    (*MySwitchTrajectoryTorque)(POSITION);
    (*MySetGravityType)(MANUAL_INPUT);
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityManualInputParam)(c);
}

/**
 * This function is used to set the gravity parameters using the optimal mode. For Mico and Jaco, the input is a vector of 16 floats. For now, this mode only works when the robot is standing (G = [0;0;-9.81]). The parameters will reset to default values every time the robotic arm is rebooted. In order to find these values, the user must run the function RunGravityZEstimationSequence(). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command the gravity parameters vector
 */
void Robot::setGravityOptimalZParam(std::vector<float> command)
{
    (*MySwitchTrajectoryTorque)(POSITION);
    (*MySetGravityType)(MANUAL_INPUT);
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityOptimalZParam)(c);
}

/**
 * This function sets the payload gravity parameters. The input is a vector of 4 floats : GravityPayload = [pm, px, py, pz]. The parameter pm is the payload mass while px, py and pz are the payload center of mass positions measured from the end-effector frame. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command the gravity payload vector of size 4
 */
void Robot::setGravityPayload(std::vector<float> command)
{
    (*MySwitchTrajectoryTorque)(POSITION);
    (*MySetGravityType)(MANUAL_INPUT);
    if(command.size() != 4)
    {
        std::cout << "WARNING:: setGravityPayload requires a vector of length 4." << std::endl;
        std::cout << "\tExiting the function now..." << std::endl;
        return;
    }
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityPayload)(c);
}

/**
 * This function sets the gravity vector direction. By default, the gravity is estimated for the classic case where the robot is standing: G = [0;0;-9.81]. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The gravity vector of size 3.
 */
void Robot::setGravityVector(std::vector<float> command)
{
    (*MySwitchTrajectoryTorque)(POSITION);
    (*MySetGravityType)(MANUAL_INPUT);
    if(command.size() != 3)
    {
        std::cout << "WARNING:: setGravityVector requires a vector of length 3." << std::endl;
        std::cout << "\tExiting the function now..." << std::endl;
        return;
    }
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityVector)(c);
}

/**
 * This function is sets the gravity type to MANUAL_INPUT or OPTIMAL. The Manual mode is used by default and consists of specifying each robot link mass and center of mass. Default parameters are provided and can be changed with the function SetManualInputParam(). The second option is the optimal mode where the gravity parameters are accurately estimated by placing the robot in various positions. The precision of the optimal mode is normally 3 times better than the manual mode. These parameters can be set with the function SetOptimalZParam(). The function RunGravityZEstimationSequence() will run a sequence of points in order to identify the gravity parameters and will output the parameters to send with the function SetOptimalZParam(). For now, this mode only works when the robot is standing (G = [0;0;-9.81]). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param type The gravity type. (MANUAL_INPUT or OPTIMAL)
 */
void Robot::setGravityType(GRAVITY_TYPE type)
{
    (*MySwitchTrajectoryTorque)(POSITION);
    (*MySetGravityType)(type);
}

