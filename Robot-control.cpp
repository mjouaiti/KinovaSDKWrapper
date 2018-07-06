#include "Robot.h"

void Robot::setAngularControl() {
    (*MySwitchTrajectoryTorque)(POSITION);
	(*MySetAngularControl)();
}

void Robot::setCartesianControl() {
    (*MySwitchTrajectoryTorque)(POSITION);
	(*MySetCartesianControl)();
}


void Robot::activateAutoNullSpaceMotionCartesian(int state)
{
    (*MyActivateAutoNullSpaceMotionCartesian)(state);
}

void Robot::activateCollisionAutomaticAvoidance(int state)
{
    (*MyActivateCollisionAutomaticAvoidance)(state);
}

void Robot::activateSingularityAutomaticAvoidance(int state)
{
    (*MyActivateSingularityAutomaticAvoidance)(state);

}

void Robot::stopRedundantJointNullSpaceMotion()
{
    (*MyStopRedundantJointNullSpaceMotion)();
}

void Robot::startRedundantJointNullSpaceMotion()
{
    (*MyStartRedundantJointNullSpaceMotion)();
}

void Robot::setControlMapping(ControlMappingCharts command)
{
    (*MySetControlMapping)(command);
}
