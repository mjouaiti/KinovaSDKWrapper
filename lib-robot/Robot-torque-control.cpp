//
//  Robot-torque-control.cpp
//  lib-robot
//
//  Created by Melanie Jouaiti on 06/07/2018.
//  Copyright © 2018 Melanie Jouaiti. All rights reserved.
//

#include "Robot.h"

void Robot::startForceControl() {
    (*MyStartForceControl)();
}

void Robot::stopForceControl() {
    (*MyStopForceControl)();
}

void Robot::startTorqueControl()
{
    (*MySetTorqueControlType)(DIRECTTORQUE);
    (*MySwitchTrajectoryTorque)(TORQUE);
}

void Robot::startImpedanceControl()
{
    (*MySetTorqueControlType)(IMPEDANCECARTESIAN);
}

/**
 *
 * @return the mode. 0 = Trajectory mode; 1 = Torque mode.
 */
int Robot::getTrajectoryTorqueMode()
{
    int mode;
    (*MyGetTrajectoryTorqueMode)(mode);
    return mode;
}

void Robot::setTorqueMinMax(std::vector<double> minTorque, std::vector<double> maxTorque)
{
    AngularInfo commandMin;
    AngularInfo commandMax;
    commandMin.InitStruct();
    commandMax.InitStruct();
    commandMin.Actuator1 = minTorque[0];
    commandMin.Actuator2 = minTorque[1];
    commandMin.Actuator3 = minTorque[2];
    commandMin.Actuator4 = minTorque[3];
    commandMin.Actuator5 = minTorque[4];
    commandMin.Actuator6 = minTorque[5];
    commandMax.Actuator1 = maxTorque[0];
    commandMax.Actuator2 = maxTorque[1];
    commandMax.Actuator3 = maxTorque[2];
    commandMax.Actuator4 = maxTorque[3];
    commandMax.Actuator5 = maxTorque[4];
    commandMax.Actuator6 = maxTorque[5];
    (*MySetAngularTorqueMinMax)(commandMin, commandMax);
}

void Robot::setTorqueVibrationController(float value)
{
    (*MySetTorqueVibrationController)(value);
}

void Robot::setTorqueSafetyFactor(float factor)
{
    (*MySetTorqueSafetyFactor)(factor);
}

void Robot::setTorqueActuatorGain(float* gains)
{
    (*MySetTorqueActuatorGain)(gains);
}

void Robot::setTorqueBrake(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueBrake)(c);
}

void Robot::setTorqueCommandMax(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueCommandMax)(c);
}

void Robot::setTorqueDampingMax(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueDampingMax)(c);
}

void Robot::setTorqueErrorDeadband(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueErrorDeadband)(c);
}

void Robot::setTorqueErrorResend(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueErrorResend)(c);
}

void Robot::setTorqueFeedCurrent(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFeedCurrent)(c);
}

void Robot::setTorqueFeedCurrentVoltage(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFeedCurrentVoltage)(c);
}

void Robot::setTorqueFeedFilter(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFeedFilter)(c);
}

void Robot::setTorqueFeedVelocity(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFeedVelocity)(c);
}

void Robot::setTorqueFeedVelocityUnderGain(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFeedVelocityUnderGain)(c);
}

void Robot::setTorqueFilterControlEffort(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFilterControlEffort)(c);
}

void Robot::setTorqueFilterError(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFilterError)(c);
}

void Robot::setTorqueFilterMeasuredTorque(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFilterMeasuredTorque)(c);
}

void Robot::setTorqueFilterVelocity(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueFilterVelocity)(c);
}

void Robot::setTorqueGainMax(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueGainMax)(c);
}

void Robot::setTorqueInactivityTimeActuator(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueInactivityTimeActuator)(c);
}

void Robot::setTorqueInactivityTimeMainController(int time)
{
    (*MySetTorqueInactivityTimeMainController)(time);
}

/**
 * This function manages the torque inactivity time option. If no torque command is sent after a given time (250ms by default), the controller will take a action
 * @param type  option (0): The robot will return in position mode; option (1): The torque commands will be set to zero. By default, option (1) is set for the Kinova classic robots (Jaco2 and Mico) while option (0) is set for generic mode.
 */
void Robot::setTorqueInactivityType(int type)
{
    (*MySetTorqueInactivityType)(type);
}

void Robot::setTorquePositionLimitDampingGain(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorquePositionLimitDampingGain)(c);
}

void Robot::setTorquePositionLimitDampingMax(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorquePositionLimitDampingMax)(c);
}

void Robot::setTorquePositionLimitRepulsGain(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorquePositionLimitRepulsGain)(c);
}

void Robot::setTorquePositionLimitRepulsMax(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorquePositionLimitRepulsMax)(c);
}

void Robot::setTorqueRateLimiter(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueRateLimiter)(c);
}

/**
 * This function is used to activate/deactivate the robot protection zone. There are several situations where the robot can come in contact with itself. To prevent collisions that could damage the robot, a basic robot protection zone covering the majority of collisions is implemented. In the first zone around the robot’s base, the damping is increased to reduce the robot's velocity. If the end-effector reaches a second smaller zone around the robot’s base, the robot will stop and return in position control.
 * @param protectionLevel The protection level. The possible values are 0, 1 and 2. 0: The protection is deactivated. 1: There is some damping when the effector enters the protected zone. 2: There is some damping when the effector enters the protected zone and if the effector continues in that direction, the robot will switch back in position mode.
 */
void Robot::setTorqueRobotProtection(int protectionLevel)
{
    (*MySetTorqueRobotProtection)(protectionLevel);
}

void Robot::setTorqueStaticFriction(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueStaticFriction)(c);
}

void Robot::setTorqueStaticFrictionMax(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueStaticFrictionMax)(c);
}

void Robot::setTorqueVelocityLimitFilter(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetTorqueVelocityLimitFilter)(c);
}

void Robot::setSwitchThreshold(std::vector<double> command)
{
    float* c = new float[command.size()];
    for(unsigned int i = 0; i < command.size(); i++)
        c[i] = command[i];
    (*MySetSwitchThreshold)(c);
}
