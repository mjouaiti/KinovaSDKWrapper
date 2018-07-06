#include "Robot.h"


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

void Robot::updateAngularCurrent() {
	(*MyGetAngularCurrent)(angularCurrent);
}

void Robot::updateAngularCurrentMotor() {
	(*MyGetAngularCurrentMotor)(angularCurrentMotor);
}

void Robot::updateAngularForce() {
	(*MyGetAngularForce)(angularForce);
}

void Robot::updateAngularForceGravityFree() {
	(*MyGetAngularForceGravityFree)(angularForceGravityFree);
}

void Robot::updateAngularPosition() {
	(*MyGetAngularPosition)(angularPosition);
}

void Robot::updateAngularVelocity() {
	(*MyGetAngularVelocity)(angularVelocity);
}

void Robot::updateTrajectoryInfo() {
    (*MyGetGlobalTrajectoryInfo)(trajectoryInfo);
}

void Robot::updateCartesianPosition() {
    (*MyGetCartesianPosition)(cartesianPosition);
}

void Robot::updateCartesianForce() {
    (*MyGetCartesianForce)(cartesianForce);
}

void Robot::updateAll() {
    this->updateAngularCurrent();
    this->updateAngularCurrentMotor();
    this->updateAngularForce();
    this->updateAngularForceGravityFree();
    this->updateAngularPosition();
    this->updateAngularVelocity();
    this->updateTrajectoryInfo();
    this->updateCartesianPosition();
    this->updateCartesianForce();
    
    this->updateClock();
}

/*----------- Get joints values ---------------*/

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

std::vector<double> Robot::getAngularTorqueCommand()
{
    float result[COMMAND_SIZE];
    (*MyGetAngularTorqueCommand)(result);
    std::vector<double> torque;
    for(unsigned int i = 0; i < COMMAND_SIZE; i++)
        torque.push_back(result[i]);
}

std::vector<double> Robot::getAngularTorqueGravityEstimation()
{
    float result[GRAVITY_PARAM_SIZE];
    (*MyGetAngularTorqueGravityEstimation)(result);
    std::vector<double> torque;
    for(unsigned int i = 0; i < COMMAND_SIZE; i++)
        torque.push_back(result[i]);
}

std::vector<double> Robot::getEndEffectorOffset(unsigned int &status)
{
    float x, y, z;
    (*MyGetEndEffectorOffset)(status, x, y, z);
    return {x, y, z};
}

double Robot::getAngularCurrent(int actuatorNumber) {
	return getActuatorAngularInfo(actuatorNumber, angularCurrent.Actuators);
}

double Robot::getAngularCurrentMotor(int actuatorNumber) {
	return getActuatorAngularInfo(actuatorNumber, angularCurrentMotor.Actuators);
}

double Robot::getAngularForce(int actuatorNumber) {
	return getActuatorAngularInfo(actuatorNumber, angularForce.Actuators);
}

double Robot::getAngularForceGravityFree(int actuatorNumber) {
	return getActuatorAngularInfo(actuatorNumber, angularForceGravityFree.Actuators);
}

double Robot::getAngularPosition(int actuatorNumber) {
	return getActuatorAngularInfo(actuatorNumber, angularPosition.Actuators);
}

double Robot::getAngularVelocity(int actuatorNumber) {
	return getActuatorAngularInfo(actuatorNumber, angularVelocity.Actuators);
}

std::vector<double> Robot::getAngularCurrent() {
	return convertAngularPositionToVector(angularCurrent);
}

std::vector<double> Robot::getAngularCurrentMotor() {
	return convertAngularPositionToVector(angularCurrentMotor);
}

std::vector<double> Robot::getAngularForce() {
	return convertAngularPositionToVector(angularForce);
}

std::vector<double> Robot::getAngularForceGravityFree() {
	return convertAngularPositionToVector(angularForceGravityFree);
}

std::vector<double> Robot::getAngularPosition() {
	return convertAngularPositionToVector(angularPosition);
}

std::vector<double> Robot::getAngularVelocity() {
	return convertAngularPositionToVector(angularVelocity);
}

std::vector<double> Robot::getCartesianPosition() {
	std::vector<double> coordinates(3);
	coordinates[0] = (double)cartesianPosition.Coordinates.X;
	coordinates[1] = (double)cartesianPosition.Coordinates.Y;
	coordinates[2] = (double)cartesianPosition.Coordinates.Z;
	return coordinates;
}

std::vector<double> Robot::getCartesianForce() {
	std::vector<double> force(3);
	force[0] = (double)cartesianForce.Coordinates.X;
	force[1] = (double)cartesianForce.Coordinates.Y;
	force[2] = (double)cartesianForce.Coordinates.Z;
	return force;
}

/*------------------ Calibrate ----------------*/
std::vector<double> Robot::calibrateAngularCurrent(int numberOfIterations) {
	return calibrateAngularData(*MyGetAngularCurrent, numberOfIterations);
}

std::vector<double> Robot::calibrateAngularCurrentMotor(int numberOfIterations) {
	return calibrateAngularData(*MyGetAngularCurrent, numberOfIterations);
}

std::vector<double> Robot::calibrateAngularForce(int numberOfIterations) {
	return calibrateAngularData(*MyGetAngularForce, numberOfIterations);
}

std::vector<double> Robot::calibrateAngularForceGravityFree(int numberOfIterations) {
	return calibrateAngularData(*MyGetAngularForceGravityFree, numberOfIterations);
}

std::vector<double> Robot::calibrateAngularPosition(int numberOfIterations) {
	return calibrateAngularData(*MyGetAngularPosition, numberOfIterations);
}

std::vector<double> Robot::calibrateAngularVelocity(int numberOfIterations) {
	return calibrateAngularData(*MyGetAngularVelocity, numberOfIterations);
}

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
