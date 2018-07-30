//
//  Robot.h
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
 * This function sets the robotical arm in angular control mode.
 */
void Robot::setAngularControl() {
    (*MySwitchTrajectoryTorque)(POSITION);
	(*MySetAngularControl)();
}

/**
 * This function sets the robotical arm in cartesian control mode (if possible).
 */
void Robot::setCartesianControl() {
    (*MySwitchTrajectoryTorque)(POSITION);
	(*MySetCartesianControl)();
}

void Robot::initFingers()
{
    (*MyInitFingers)();
}

/**
 * This function activates/deactivates the automatic null space (elbow) motion during Cartesian control. This function is only available for 7 dof robots.
 * @param state When state=1, protection is activated. When state=0, protection is deactivated.
 */
void Robot::activateAutoNullSpaceMotionCartesian(int state)
{
    (*MyActivateAutoNullSpaceMotionCartesian)(state);
}

/**
 * This function activates/deactivates the automatic collision avoidance algorithms.
 * @param state When state=1, protection is activated. When state=0, protection is deactivated.
 */
void Robot::activateCollisionAutomaticAvoidance(int state)
{
    (*MyActivateCollisionAutomaticAvoidance)(state);
}

/**
 * This function activates/deactivates an extra safety feature based on the torque value at the wrist actuator. This feature is available for robots with a spherical wrist and prevents the application of a torque over a given limit at the wrist actuator. This limit was chosen in the eventuality that someone would get an arm or other body part stuck between the two wrist joints. That way, the user won't get harmed and would be able to release his body part quickly and easily. The drawback to this safety limit is that it reduces the robot's payload in some directions of movement.
 * @param state When state=1, protection is activated. When state=0, protection is deactivated.
 */
void Robot::activateExtraProtectionPinchingWrist(int state)
{
    (*MyActivateExtraProtectionPinchingWrist)(state);
}

/**
 * This function activates/deactivates the automatic singularity avoidance algorithms.
 * @param state When state=1, protection is activated. When state=0, protection is deactivated.
 */
void Robot::activateSingularityAutomaticAvoidance(int state)
{
    (*MyActivateSingularityAutomaticAvoidance)(state);

}

/**
 * This function deactivates the motion in the null space (angular control) and resets your robotic device to cartesian control (if the position of the robotic device permits it).
 * @warning You can only use this function if your robotic device has at least 7 joints.
 */
void Robot::stopRedundantJointNullSpaceMotion()
{
    (*MyStopRedundantJointNullSpaceMotion)();
}

/**
 * This function activates the motion in the null space (angular control).
 * @warning You can only use this function if your robotic device has at least 7 joints.
 */
void Robot::startRedundantJointNullSpaceMotion()
{
    (*MyStartRedundantJointNullSpaceMotion)();
}

/**
 * This function sets new control mapping charts to the robotical arm.
 * @param command The new control mapping charts.
 */
void Robot::setControlMapping(ControlMappingCharts command)
{
    (*MySetControlMapping)(command);
}
