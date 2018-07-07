#include "Robot.h"


/**
 *
 * @return
 */
int Robot::getTrajectoryCount() {
    return trajectoryInfo.TrajectoryCount;
}

std::vector<double> Robot::getSpasmFilterValues(int &activationStatus)
{
    float result[SPASM_FILTER_COUNT];
    (*MyGetSpasmFilterValues)(result, activationStatus);
    std::vector<double> values;
    for(unsigned int i = 0; i < SPASM_FILTER_COUNT; i++)
        values.push_back(result[i]);
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
 * @return acceleration of each actuator (std::vector<std::vector<double>>)
 */
std::vector<std::vector<double>> Robot::getActuatorAcceleration()
{
    AngularAcceleration data;
    std::vector<std::vector<double>> acc;
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
 * @return The angular torque command(std::vector<double>)
 */
std::vector<double> Robot::getAngularTorqueCommand()
{
    float result[COMMAND_SIZE];
    (*MyGetAngularTorqueCommand)(result);
    std::vector<double> torque(std::begin(result), std::end(result));
}

/**
 * This function returns the actual estimation of the gravity torques.
 * @return The gravity torques (std::vector<double>)
 */
std::vector<double> Robot::getAngularTorqueGravityEstimation()
{
    float result[GRAVITY_PARAM_SIZE];
    (*MyGetAngularTorqueGravityEstimation)(result);
    std::vector<double> torque(std::begin(result), std::end(result));
}

/**
 * This function gets the end effector offset's parameters. The end effector's offset is a translation offset, in meters, applied to the end effector of the robotic arm.
 * @param status indicates if the offset is applied or not (0 = not applied, 1 = applied).
 * @return the offset (std::vector<double>)
 */
std::vector<double> Robot::getEndEffectorOffset(unsigned int &status)
{
    float x, y, z;
    (*MyGetEndEffectorOffset)(status, x, y, z);
    return {x, y, z};
}

/**
 * This function returns the current that each actuator consumes on the main power supply. Unit is Amperes.
 * @return current that each actuator consumes on the main power supply (std::vector<double>)
 */
std::vector<double> Robot::getAngularCurrent()
{
    (*MyGetAngularCurrent)(angularCurrent);
	return convertAngularPositionToVector(angularCurrent);
}

/**
 * This function gets motor's current for each actuator Unit is Amperes.
 * @return motor's current for each actuator (std::vector<double>)
 */
std::vector<double> Robot::getAngularCurrentMotor()
{
    (*MyGetAngularCurrentMotor)(angularCurrentMotor);
	return convertAngularPositionToVector(angularCurrentMotor);
}

/**
 * This function returns the torque of each actuator. Unit is Newton meter [N * m].
 * @return torque of each actuator (std::vector<double>)
 */
std::vector<double> Robot::getAngularForce()
{
    (*MyGetAngularForce)(angularForce);
	return convertAngularPositionToVector(angularForce);
}

/**
 * This function returns the torque, without gravity, of each actuator. Unit is Newton meter [N * m].
 * @return (std::vector<double>)
 */
std::vector<double> Robot::getAngularForceGravityFree()
{
    (*MyGetAngularForceGravityFree)(angularForceGravityFree);
	return convertAngularPositionToVector(angularForceGravityFree);
}

/**
 * This function returns the angular position of the robotical arm's end effector. Units are in degrees.
 * @return angular position of the robotical arm's end effector (std::vector<double>)
 */
std::vector<double> Robot::getAngularPosition()
{
    (*MyGetAngularPosition)(angularPosition);
	return convertAngularPositionToVector(angularPosition);
}

/**
 * This function gets the velocity of each actuator. Units are degrees / second.
 * @return velocity of each actuator (std::vector<double>)
 */
std::vector<double> Robot::getAngularVelocity()
{
    (*MyGetAngularVelocity)(angularVelocity);
	return convertAngularPositionToVector(angularVelocity);
}

/**
 * This function returns the cartesian position of the robotical arm's end effector. The orientation is defined by Euler angles (convention XYZ).
 * @return the cartesian position of the robotical arm's end effector (std::vector<double>)
 */
std::vector<double> Robot::getCartesianPosition()
{
    (*MyGetCartesianPosition)(cartesianPosition);
	std::vector<double> coordinates(3);
	coordinates[0] = (double)cartesianPosition.Coordinates.X;
	coordinates[1] = (double)cartesianPosition.Coordinates.Y;
	coordinates[2] = (double)cartesianPosition.Coordinates.Z;
	return coordinates;
}

/**
 * This function returns the cartesian force at the robotical arm's end effector. The translation unit is in Newtons and the orientation unit is Newton meters [N * m].
 * @return the cartesian force at the robotical arm's end effector (std::vector<double>)
 */
std::vector<double> Robot::getCartesianForce()
{
    (*MyGetCartesianForce)(cartesianForce);
	std::vector<double> force(3);
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
std::vector<double> Robot::calibrateAngularCurrent(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularCurrent, numberOfIterations);
}

/**
 * calibrates angular current motor
 * @param numberOfIterations number of iterations
 */
std::vector<double> Robot::calibrateAngularCurrentMotor(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularCurrentMotor, numberOfIterations);
}

/**
 * calibrates angular force
 * @param numberOfIterations number of iterations
 */
std::vector<double> Robot::calibrateAngularForce(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularForce, numberOfIterations);
}

/**
 * calibrates angular force gravity free
 * @param numberOfIterations number of iterations
 */
std::vector<double> Robot::calibrateAngularForceGravityFree(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularForceGravityFree, numberOfIterations);
}

/**
 * calibrates angular position
 * @param numberOfIterations number of iterations
 */
std::vector<double> Robot::calibrateAngularPosition(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularPosition, numberOfIterations);
}

/**
 * calibrates angular position
 * @param numberOfIterations number of iterations
 */
std::vector<double> Robot::calibrateAngularVelocity(int numberOfIterations)
{
	return calibrateAngularData(*MyGetAngularVelocity, numberOfIterations);
}

/**
 * Template calibration function
 * @param (*MyGetAngularData)(AngularPosition &) MyGetAngularData can be MyGetAngularVelocity, MyGetAngularPosition, MyGetAngularForceGravityFree, MyGetAngularForce, MyGetAngularCurrent, MyGetAngularCurrentMotor
 * @param numberOfIterations number of iterations
 */
std::vector<double> Robot::calibrateAngularData(int (*MyGetAngularData)(AngularPosition &), int numberOfIterations) {
	AngularPosition currentData;
	std::vector<double> vectorizedData(ACTUATORS_COUNT);

	std::vector<double> mean(ACTUATORS_COUNT, 0);
	for (int i = 1; i <= numberOfIterations; i++) {
		(*MyGetAngularData)(currentData);
		vectorizedData = convertAngularPositionToVector(currentData);
		for (int j = 0; j < ACTUATORS_COUNT; j++) {
			mean[j] = vectorizedData[j] / (double)i + (double)(i-1) / (double)i * mean[j];
		}
	}

	return mean;
}
