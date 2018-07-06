#include <iostream>

#include "Robot.h"

void Robot::setInertiaDamping(std::vector<double> inertia, std::vector<double> damping)
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

void Robot::setCartesianInertiaDamping(std::vector<double> inertia, std::vector<double> damping)
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


void Robot::setPositionLimitDistance(std::vector<double>command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetPositionLimitDistance)(c);
}

void Robot::setSpasmFilterValues(std::vector<double>command, int activationStatus)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetSpasmFilterValues)(c, activationStatus);
}

void Robot::setPID(std::vector<std::vector<double> > newPID) {
	for (int actuatorNumber = 1; actuatorNumber < newPID.size() + 1; actuatorNumber++) {
		setPID(actuatorNumber, newPID[actuatorNumber - 1]);
	}
}

void Robot::setPID(int actuatorNumber, std::vector<double> newPID) {
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

std::vector<double> Robot::runGravityZEstimationSequence(ROBOT_TYPE type)
{
    double OptimalzParam[OPTIMAL_Z_PARAM_SIZE];
    (*MyRunGravityZEstimationSequence)(type, OptimalzParam);
    return std::vector<double>(std::begin(OptimalzParam), std::end(OptimalzParam));
}

void Robot::setGravityManualInputParam(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityManualInputParam)(c);
}

void Robot::setGravityOptimalZParam(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityOptimalZParam)(c);
}

void Robot::setGravityPayload(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityPayload)(c);
}

void Robot::setGravityVector(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetGravityVector)(c);
}

void Robot::setGravityType(GRAVITY_TYPE type)
{
    (*MySetGravityType)(type);
}

