#include "Robot.h"

void Robot::startClock() {
    startTime = std::chrono::high_resolution_clock::now();
    currentTime = startTime;
}

void Robot::updateClock() {
    currentTime = std::chrono::high_resolution_clock::now();
}

double Robot::getMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(currentTime-startTime).count();
}

double Robot::getSeconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(currentTime-startTime).count() / 1000000.0;
}
