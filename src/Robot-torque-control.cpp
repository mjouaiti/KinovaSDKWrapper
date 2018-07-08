//
//  Robot-torque-control.cpp
//  lib-robot
//
//  Created by Melanie Jouaiti on 06/07/2018.
//  Copyright © 2018 Melanie Jouaiti. All rights reserved.
//

#include "Robot.h"

/**
 * This function activates the reactive force control.
 */
void Robot::startForceControl() {
    (*MyStartForceControl)();
}

/**
 * This function terminates the force control.
 */
void Robot::stopForceControl() {
    (*MyStopForceControl)();
}

/**
 * This function is used to specify the Torque Control Mode (direct torque control).
 */
void Robot::startTorqueControl()
{
    (*MySetTorqueControlType)(DIRECTTORQUE);
    (*MySwitchTrajectoryTorque)(TORQUE);
}

/**
 * This function is used to specify the Torque Control Mode (impedance control). The options IMPEDANCE ANGULAR and IMPEDANCECARTESIAN.
 */
void Robot::startImpedanceControl()
{
    (*MySetTorqueControlType)(IMPEDANCECARTESIAN);
}

/**
 * This function returns the mode. 0 = Trajectory mode; 1 = Torque mode.
 * @return the mode. 0 = Trajectory mode; 1 = Torque mode.
 */
int Robot::getTrajectoryTorqueMode()
{
    int mode;
    (*MyGetTrajectoryTorqueMode)(mode);
    return mode;
}

/**
 * This function sets the angular torque's maximum and minimum values.
 * @param minTorque A struct that contains all angular minimum values. (Unit: [N * m])
 * @param maxTorque A struct that contains all angular maximum values. (Unit: [N * m])
 */
void Robot::setTorqueMinMax(std::vector<float> minTorque, std::vector<float> maxTorque)
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

/**
 * This function is used to set the vibration observer/controller level. In different situations, for example, when interacting with a stiff environment, vibration may occur. The vibration observer/controller is implemented to eliminate these vibrations. The input value varies between 0 and 1. A value of 0 means that the feature is deactivated and a value of 1 means that the vibration observer/controller is set to its maximal level. The observer analyzes all actuators' velocity signal. If the velocity signal frequency and amplitude are too high, the controller will reduce the control gains, thus reducing the vibration.
 * @param value Vibration control level
 */
void Robot::setTorqueVibrationController(float value)
{
    (*MySetTorqueVibrationController)(value);
}

/**
 * This function is a safety feature used to specify the velocity threshold over which the robot will stop and return to position control. This is a safety feature to prevent the arm from performing undesired and/or dangerous motions when implementing, or using, a program. The value varies between 0 and 1. A value of 0.1 means that the safety will trigger if an actuator reaches 10% of its maximal speed. If a value of 1 is set, the safety feature is disabled. The user must be very cautious if he chooses to increase the safety factor.
 * @param factor type of control
 */
void Robot::setTorqueSafetyFactor(float factor)
{
    (*MySetTorqueSafetyFactor)(factor);
}

/**
 * This function sets the actuators feedback gain. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param gains The gain for each actuator
 */
void Robot::setTorqueActuatorGain(std::vector<float> gains)
{
    (*MySetTorqueActuatorGain)(&gains[0]);
}

/**
 * This function sets the brake value when the velocity is near zero. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The torque brake for each actuator
 */
void Robot::setTorqueBrake(std::vector<float> command)
{
    (*MySetTorqueBrake)(&command[0]);
}

/**
 * This function sets the actuator's maximum allowed torque command. Default parameters can be found in the torque control documentation page under the section “Default parameters”. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The torque command's maximum for each actuator
 */
void Robot::setTorqueCommandMax(std::vector<float> command)
{
    (*MySetTorqueCommandMax)(&command[0]);
}

/**
 * This functions sets the maximum damping. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The maximum damping
 */
void Robot::setTorqueDampingMax(std::vector<float> command)
{
    (*MySetTorqueDampingMax)(&command[0]);
}

/**
 * This function sets the deadband on the error. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The deadband for each actuator
 */
void Robot::setTorqueErrorDeadband(std::vector<float> command)
{
    (*MySetTorqueErrorDeadband)(&command[0]);
}

/**
 * This function sets the time in ms before the actuator resends an error message if it was not cleared. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The time for resending errors for each actuator
 */
void Robot::setTorqueErrorResend(std::vector<float> command)
{
    (*MySetTorqueErrorResend)(&command[0]);
}

/**
 * This function sets the amount of current feedforward sent to the actuators. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The current feed for each actuator
 */
void Robot::setTorqueFeedCurrent(std::vector<float> command)
{
    (*MySetTorqueFeedCurrent)(&command[0]);
}

/**
 * This function sets the conversion from current to voltage gain. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The feedforward
 */
void Robot::setTorqueFeedCurrentVoltage(std::vector<float> command)
{
    (*MySetTorqueFeedCurrentVoltage)(&command[0]);
}

/**
 * This function sets the filter on the feedforward. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The feed filter for each actuator
 */
void Robot::setTorqueFeedFilter(std::vector<float> command)
{
    (*MySetTorqueFeedFilter)(&command[0]);
}

/**
 * This function sets the amount of velocity feedforward (back EMF) sent to the actuators. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The velocity feed for each actuator
 */
void Robot::setTorqueFeedVelocity(std::vector<float> command)
{
    (*MySetTorqueFeedVelocity)(&command[0]);
}

/**
 * This function sets the velocity feedforward under compensation gain. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The feedforward
 */
void Robot::setTorqueFeedVelocityUnderGain(std::vector<float> command)
{
    (*MySetTorqueFeedVelocityUnderGain)(&command[0]);
}

/**
 * This function sets the filter on the torque control effort (command-measured). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The control effort for each actuator
 */
void Robot::setTorqueFilterControlEffort(std::vector<float> command)
{
    (*MySetTorqueFilterControlEffort)(&command[0]);
}

/**
 * This function sets the filter on the torque error (command-measured). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The error filter for each actuator
 */
void Robot::setTorqueFilterError(std::vector<float> command)
{
    (*MySetTorqueFilterError)(&command[0]);
}

/**
 * This function sets the filter on the torque error (command-measured). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The error filter for each actuator
 */
void Robot::setTorqueFilterMeasuredTorque(std::vector<float> command)
{
    (*MySetTorqueFilterMeasuredTorque)(&command[0]);
}

/**
 * This function sets the filter on the estimated velocity. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The velocity filter for each actuator
 */
void Robot::setTorqueFilterVelocity(std::vector<float> command)
{
    (*MySetTorqueFilterVelocity)(&command[0]);
}

/**
 * This function sets the maximum allowed control gain. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The maximum gain
 */
void Robot::setTorqueGainMax(std::vector<float> command)
{
    (*MySetTorqueGainMax)(&command[0]);
}

/**
 * This function sets the amount of time over which the system sending messages to the actuators is considered non-responsive. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The amout of time considered non-responsive
 */
void Robot::setTorqueInactivityTimeActuator(std::vector<float> command)
{
    (*MySetTorqueInactivityTimeActuator)(&command[0]);
}

/**
 * This function sets the amount of time over which the system sending messages to the main controller is considered non-rresponsive. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param time The amout of time considered non-responsive
 */
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

/**
 * This function sets the position limit damping gain. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The position damping gain for each actuator
 */
void Robot::setTorquePositionLimitDampingGain(std::vector<float> command)
{
    (*MySetTorquePositionLimitDampingGain)(&command[0]);
}

/**
 * This function sets the maximum position limit damping gain. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The maximum position for each actuator
 */
void Robot::setTorquePositionLimitDampingMax(std::vector<float> command)
{
    (*MySetTorquePositionLimitDampingMax)(&command[0]);
}

/**
 * This function sets the position limit repulse gain (stifness). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The position limit repulse gain for each actuator
 */
void Robot::setTorquePositionLimitRepulsGain(std::vector<float> command)
{
    (*MySetTorquePositionLimitRepulsGain)(&command[0]);
}

/**
 * This function sets the maximum position limit repulse gain (max stifness). The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The maximum position limit repulse gain for each actuator
 */
void Robot::setTorquePositionLimitRepulsMax(std::vector<float> command)
{
    (*MySetTorquePositionLimitRepulsMax)(&command[0]);
}

/**
 * This function sets the torque signal rate limiter. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command the torque rate
 */
void Robot::setTorqueRateLimiter(std::vector<float> command)
{
    (*MySetTorqueRateLimiter)(&command[0]);
}

/**
 * This function is used to activate/deactivate the robot protection zone. There are several situations where the robot can come in contact with itself. To prevent collisions that could damage the robot, a basic robot protection zone covering the majority of collisions is implemented. In the first zone around the robot’s base, the damping is increased to reduce the robot's velocity. If the end-effector reaches a second smaller zone around the robot’s base, the robot will stop and return in position control.
 * @param protectionLevel The protection level. The possible values are 0, 1 and 2. 0: The protection is deactivated. 1: There is some damping when the effector enters the protected zone. 2: There is some damping when the effector enters the protected zone and if the effector continues in that direction, the robot will switch back in position mode.
 */
void Robot::setTorqueRobotProtection(int protectionLevel)
{
    (*MySetTorqueRobotProtection)(protectionLevel);
}

/**
 * This function sets the static friction value for each actuator. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The static friction for each actuator
 */
void Robot::setTorqueStaticFriction(std::vector<float> command)
{
    (*MySetTorqueStaticFriction)(&command[0]);
}

/**
 * This function sets the maximum static friction value. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The feedforward
 */
void Robot::setTorqueStaticFrictionMax(std::vector<float> command)
{
    (*MySetTorqueStaticFrictionMax)(&command[0]);
}

/**
 * This function sets the filter for the estimation of the velocity used for the maximum velocity safety feature. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The velocity filter for each actuator
 */
void Robot::setTorqueVelocityLimitFilter(std::vector<float> command)
{
    (*MySetTorqueVelocityLimitFilter)(&command[0]);
}

/**
 * When switching from position to torque control, the controller verifies if the torque command is close to the actual measured torque (as a safety feature to prevent high velocity motions if it is not the case). This function can be used to change the threshold over which the switching to torque mode is rejected. Default parameters can be found in the torque control documentation page under the section “Default parameters”. The parameters will reset to default values every time the robotic arm is rebooted.
 * @param command The torque switch threshold of each actuator
 */
void Robot::setSwitchThreshold(std::vector<float> command)
{
    (*MySetSwitchThreshold)(&command[0]);
}
