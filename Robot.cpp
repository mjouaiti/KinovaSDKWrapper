#include <dlfcn.h>
#include <iostream>
#include <string>

#include "Robot.h"

const int Robot::ACTUATORS_COUNT = 6;
const int Robot::FIRST_ACTUATOR_PID_ADDRESS = 16;
const double Robot::MAX_P = 2.0;
const double Robot::MAX_D = 0.1;
const double Robot::MIN_PID = 0.0;
const double Robot::DEFAULT_P = 2.0;
const double Robot::DEFAULT_I = 0.0;
const double Robot::DEFAUTL_D = 0.05;

Robot::Robot(const std::string& libPath, const std::string& optionFile)
{
    //"/home/melanie/Documents/mico_handshaking/lib-robot/Kinova.API.USBCommandLayerUbuntu.so"
	commandLayerHandle = dlopen(libPath.c_str(), RTLD_NOW|RTLD_GLOBAL);

	if (commandLayerHandle == NULL) {
		std::cout << "Error while loading library." << std::endl;
	}

	this->initializeAPI(optionFile);
	this->initializePositionTrajectory();
	this->initializeVelocityTrajectory();
}
