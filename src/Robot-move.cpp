//
//  Robot-move.cpp
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

/*-------------- Velocity ------------------------*/

/**
 * Initializes the velocity trajectory structure and activates angular velocity control
 */
void Robot::initializeVelocityTrajectory() {
    velocityTrajectory.InitStruct();
    velocityTrajectory.Position.Type = ANGULAR_VELOCITY;
    velocityTrajectory.Position.HandMode = VELOCITY_MODE;
}

/**
 * Sets the velocity of a given joint
 * @param actuatorNumber joint number
 * @param newVelocity velocity command
 */
void Robot::setVelocity(int actuatorNumber, double newVelocity)
{
    velocityTrajectory.Position.Actuators = setActuatorAngularValue(actuatorNumber, newVelocity, velocityTrajectory.Position.Actuators);
    
    (*MySendBasicTrajectory)(velocityTrajectory);
}

/**
 * sets the velocity of the fingers
 * @param fingerNumber finger number
 * @param newVelocity velocity command
 */
void Robot::setFingerVelocity(int fingerNumber, double newVelocity)
{
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
    (*MySendAdvanceTrajectory)(velocityTrajectory);
}

/**
 * sets the position of the fingers
 * @param fingerNumber finger number
 * @param newPosition position command
 */
void Robot::setFingerPosition(int fingerNumber, double newPosition)
{
    switch(fingerNumber)
    {
        case 1:
            positionTrajectory.Position.Fingers.Finger1 = newPosition;
            break;
        case 2:
            positionTrajectory.Position.Fingers.Finger2 = newPosition;
            break;
        case 3:
            positionTrajectory.Position.Fingers.Finger3 = newPosition;
            break;
    }
    
    (*MySendBasicTrajectory)(positionTrajectory);
}

/**
 * sends velocity command to the joints
 * @param newVelocity velocity command
 */
void Robot::setVelocity(std::vector<float> newVelocity) {
    velocityTrajectory.Position.Actuators = convertVectorToAngularInfo(newVelocity);
    
    (*MySendBasicTrajectory)(velocityTrajectory);
}

/*---------------- Position ------------------*/

/**
 * Initializes the position trajectory structure and activates angular position control
 */
void Robot::initializePositionTrajectory() {
    positionTrajectory.InitStruct();
    positionTrajectory.Position.Type = ANGULAR_POSITION;
    
    positionTrajectory.Position.Actuators = angularPosition.Actuators;
    positionTrajectory.Position.Fingers = angularPosition.Fingers;
    velocityTrajectory.Position.HandMode = POSITION_MODE;
}

/**
 * send position command for one joint
 * @param actuatorNumber joint number
 * @param pointToSend position command
 */
void Robot::setPosition(int actuatorNumber, double pointToSend) {
    positionTrajectory.Position.Actuators = setActuatorAngularValue(actuatorNumber, pointToSend, positionTrajectory.Position.Actuators);
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

/**
 * send position command
 * @param pointToSend position commands
 */
void Robot::setPosition(std::vector<float> pointToSend) {
    positionTrajectory.Position.Actuators = convertVectorToAngularInfo(pointToSend);
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

/**
 * Apply offset to a given joint
 * @param actuatorNumber joint number
 * @param deltaTheta offset value
 */
void Robot::moveFromCurrentPosition(int actuatorNumber, double deltaTheta) {
    positionTrajectory.Position.Actuators = setActuatorAngularValue(actuatorNumber, getAngularPosition()[actuatorNumber] + deltaTheta, positionTrajectory.Position.Actuators);
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

/**
 * Apply offset to joints
 * @param deltaVector offset vector
 */
void Robot::moveFromCurrentPosition(std::vector<float> deltaVector) {
    std::vector<float> vectorToSend(ACTUATORS_COUNT);
    
    std::vector<float> currentPosition = getAngularPosition();
    for (int i = 0; i < ACTUATORS_COUNT; i++) {
        vectorToSend[i] = currentPosition[i] + deltaVector[i];
    }
    positionTrajectory.Position.Actuators = convertVectorToAngularInfo(vectorToSend);
    
    (*MySendAdvanceTrajectory)(positionTrajectory);
}

/**
 * Sets trajectory speed limitations
 * @param maxSpeed1 In a cartesian context, this represents the translation velocity but in an angular context, this represents the velocity of the actuators 1, 2 and 3.
 * @param maxSpeed2 In a cartesian context, this represents the translation velocity but in an angular context, this represents the velocity of the actuators 4, 5 and 6.
 */
void Robot::setPositionTrajectoryLimitations(double maxSpeed1, double maxSpeed2) {
    positionTrajectory.LimitationsActive = 1;
    positionTrajectory.Limitations.speedParameter1 = (float)maxSpeed1;
    positionTrajectory.Limitations.speedParameter2 = (float)maxSpeed2;
}

/*--------------- Torque/Force ------------*/

/**
 * This function sends torque commands to the actuators. The torque command must be sent at least every 250 ms (this value can be configured). Otherwise the command will be set to zero (as a safety feature).
 * @param actuator joint number
 * @param newTorque torqueCommand
 */
void Robot::setTorque(int actuator, double newTorque)
{
    TorqueCommand[actuator] = newTorque;
    (*MySendAngularTorqueCommand)(TorqueCommand);
}

/**
 * This function sends torque commands to the actuators. The torque command must be sent at least every 250 ms (this value can be configured). Otherwise the command will be set to zero (as a safety feature).
 * @param newTorque torqueCommand
 */
void Robot::setTorque(std::vector<float> newTorque)
{
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        TorqueCommand[i] = newTorque[i];
    }
    (*MySendAngularTorqueCommand)(TorqueCommand);
}

/**
 * This function sends cartesian force commands to the robot for one actuator. The force command must be sent at least every 250 ms (this value can be configured). Otherwise the command will be set to zero (as a safety feauture). In this first implementation, the force command is in an open loop: the torque is inserted into the following equation and the resulting torque is sent to the actuators.
 $ @param actuator joint number
 * @param newForce The force command.
 */
void Robot::setCartesianForce(int actuator, double newForce)
{
    CartForceCommand[actuator] = newForce;
    
    (*MySendCartesianForceCommand)(CartForceCommand);
}

/**
 * This function sends cartesian force commands to the robot. The force command must be sent at least every 250 ms (this value can be configured). Otherwise the command will be set to zero (as a safety feauture). In this first implementation, the force command is in an open loop: the torque is inserted into the following equation and the resulting torque is sent to the actuators.
 * @param newForce The force command.
 */
void Robot::setCartesianForce(std::vector<float> newForce)
{
    (*MySendCartesianForceCommand)(&newForce[0]);
}

/**
 * This function sets the actuators damping gain. This can be used to stabilize the system in some situations. The values must be over or equal to zero and are typically within 0-2. The maximum allowed damping can be set with the function SetTorqueDampingMax(). Default parameters can be found in the torque control documentation page under the section “Default parameters”. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param damping The damping for each actuator.
 */
void Robot::setTorqueDamping(std::vector<float> damping)
{
    float dampingCommand[ACTUATORS_COUNT];
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        dampingCommand[i] = damping[i];
    }
    (*MySetTorqueActuatorDamping)(dampingCommand);
}


/*----------------------*/
/**
 * This function sets the end effector offset's parameters. The end effector's offset is a translation offset, in meters, applied to the end effector of the robotic arm.
 * @param applied indicates if the offset is applied or not (0 = not applied, 1 = applied)
 * @param offset end effector's offset
 */
void Robot::setEndEffectorOffset(bool applied, std::vector<float> offset)
{
    (*MySetEndEffectorOffset)(applied, offset[0], offset[1], offset[2]);
}

/**
 * This function sets the current position of a specific actuator at a zero. Mainly used for calibration.
 * @param actuatorNumber joint number
 */
void Robot::setJointZero(int actuatorNumber)
{
    (*MySetJointZero)(FIRST_ACTUATOR_PID_ADDRESS + actuatorNumber - 1);
}

/**
 * This function erases all the trajectories inside the robotical arm's FIFO.
 */
void Robot::eraseTrajectories()
{
    (*MyEraseAllTrajectories)();
    std::cout << "All trajectories have been deleted." << std::endl;
}
