#include <iostream>

#include "Robot.h"

/*-------------- Velocity ------------------------*/

void Robot::initializeVelocityTrajectory() {
    velocityTrajectory.InitStruct();
    velocityTrajectory.Position.Type = ANGULAR_VELOCITY;
}

void Robot::setVelocity(int actuatorNumber, double newVelocity) {
    velocityTrajectory.Position.Actuators = setActuatorAngularInfo(actuatorNumber, newVelocity, velocityTrajectory.Position.Actuators);
    
    (*MySendBasicTrajectory)(velocityTrajectory);
}

void Robot::setFingerVelocity(int fingerNumber, double newVelocity) {
    switch(fingerNumber)
    {
	case 1:
	    velocityTrajectory.Position.Fingers.Finger1 = newVelocity;
	    break;
	case 2:
	    velocityTrajectory.Position.Fingers.Finger2 = newVelocity;
	    break;
	case 3:
	    velocityTrajectory.Position.Fingers.Finger3 = newVelocity;
	    break;
    }
    
    (*MySendBasicTrajectory)(velocityTrajectory);
}

void Robot::setVelocity(std::vector<double> newVelocity) {
    velocityTrajectory.Position.Actuators = convertVectorToAngularInfo(newVelocity);
    
    (*MySendBasicTrajectory)(velocityTrajectory);
}

/*---------------- Position ------------------*/

void Robot::initializePositionTrajectory() {
    positionTrajectory.InitStruct();
    positionTrajectory.Position.Type = ANGULAR_POSITION;
    
    positionTrajectory.Position.Actuators = angularPosition.Actuators;
    positionTrajectory.Position.Fingers = angularPosition.Fingers;
}

void Robot::setPosition(int actuatorNumber, double pointToSend) {
    positionTrajectory.Position.Actuators = setActuatorAngularInfo(actuatorNumber, pointToSend, positionTrajectory.Position.Actuators);
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

void Robot::setPosition(std::vector<double> pointToSend) {
    positionTrajectory.Position.Actuators = convertVectorToAngularInfo(pointToSend);
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

void Robot::moveFromCurrentPosition(int actuatorNumber, double deltaTheta) {
    positionTrajectory.Position.Actuators = setActuatorAngularInfo(actuatorNumber, getAngularPosition(actuatorNumber) + deltaTheta, positionTrajectory.Position.Actuators);
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

void Robot::moveFromCurrentPosition(std::vector<double> deltaVector) {
    std::vector<double> vectorToSend(ACTUATORS_COUNT);
    
    std::vector<double> currentPosition = getAngularPosition();
    for (int i = 0; i < ACTUATORS_COUNT; i++) {
        vectorToSend[i] = currentPosition[i] + deltaVector[i];
    }
    positionTrajectory.Position.Actuators = convertVectorToAngularInfo(vectorToSend);
    
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

void Robot::setPositionTrajectoryLimitations(double maxSpeed1, double maxSpeed2) {
    positionTrajectory.LimitationsActive = 1;
    positionTrajectory.Limitations.speedParameter1 = (float)maxSpeed1;
    positionTrajectory.Limitations.speedParameter2 = (float)maxSpeed2;
}

/*--------------- Torque/Force ------------*/

void Robot::setTorque(int actuator, double newTorque)
{
    TorqueCommand[actuator] = newTorque;
    (*MySendAngularTorqueCommand)(TorqueCommand);
}

void Robot::setTorque(std::vector<double> newTorque)
{
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        TorqueCommand[i] = newTorque[i];
    }
    (*MySendAngularTorqueCommand)(TorqueCommand);
}

void Robot::setCartesianForce(int actuator, double newForce)
{
    CartForceCommand[actuator] = newForce;
    
    (*MySendCartesianForceCommand)(CartForceCommand);
}

void Robot::setCartesianForce(std::vector<double> newForce)
{
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        CartForceCommand[i] = newForce[i];
    }
    
    (*MySendCartesianForceCommand)(CartForceCommand);
}

void Robot::setTorqueDamping(std::vector<double> damping)
{
    float dampingCommand[ACTUATORS_COUNT];
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        dampingCommand[i] = damping[i];
    }
    (*MySetTorqueActuatorDamping)(dampingCommand);
}


/*----------------------*/
void Robot::setEndEffectorOffset(bool applied, std::vector<double> offset)
{
    (*MySetEndEffectorOffset)(applied, offset[0], offset[1], offset[2]);
}

void Robot::setJointZero(int actuatorNumber)
{
    (*MySetJointZero)(FIRST_ACTUATOR_PID_ADDRESS + actuatorNumber - 1);
}

void Robot::eraseTrajectories() {
    (*MyEraseAllTrajectories)();
}
