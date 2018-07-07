#include "Robot.h"

/**
 * Start robot clock
 */
void Robot::startClock() {
    startTime = std::chrono::high_resolution_clock::now();
    currentTime = startTime;
}

/**
 * Update robot clock
 */
void Robot::updateClock() {
    currentTime = std::chrono::high_resolution_clock::now();
}

/**
 * Get robot time in microseconds
 * @return time
 */
double Robot::getMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(currentTime-startTime).count();
}

/**
 * Get robot time in seconds
 * @return time
 */
double Robot::getSeconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(currentTime-startTime).count() / 1000000.0;
}
