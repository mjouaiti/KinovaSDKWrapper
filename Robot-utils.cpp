#include "Robot.h"

double Robot::getActuatorAngularInfo(int actuatorNumber, AngularInfo overallInfo) {
    switch(actuatorNumber) {
        case 1:
            return (double)overallInfo.Actuator1;
            break;
        case 2:
            return (double)overallInfo.Actuator2;
            break;
        case 3:
            return (double)overallInfo.Actuator3;
            break;
        case 4:
            return (double)overallInfo.Actuator4;
            break;
        case 5:
            return (double)overallInfo.Actuator5;
            break;
        case 6:
            return (double)overallInfo.Actuator6;
            break;
        default:
            return 0.0;
    }
}

AngularInfo Robot::setActuatorAngularInfo(int actuatorNumber, double newValue, AngularInfo overallInfo) {
    AngularInfo newInfo = overallInfo;
    newValue = (float)newValue;
    switch(actuatorNumber) {
        case 1:
            newInfo.Actuator1 = newValue;
            break;
        case 2:
            newInfo.Actuator2 = newValue;
            break;
        case 3:
            newInfo.Actuator3 = newValue;
            break;
        case 4:
            newInfo.Actuator4 = newValue;
            break;
        case 5:
            newInfo.Actuator5 = newValue;
            break;
        case 6:
            newInfo.Actuator6 = newValue;
            break;
        default:
            newInfo = overallInfo;
    }
    return newInfo;
}

AngularInfo Robot::setFingerAngularInfo(int fingerNumber, double newValue, AngularInfo overallInfo) {
    AngularInfo newInfo = overallInfo;
    newValue = (float)newValue;
    /*switch(actuatorNumber) {
        case 1:
            newInfo.Finger1 = newValue;
            break;
        case 2:
            newInfo.Finger2 = newValue;
            break;
        case 3:
            newInfo.Finger3 = newValue;
            break;
        default:
            newInfo = overallInfo;
    }*/
    return newInfo;
}

AngularInfo Robot::convertVectorToAngularInfo(std::vector<double> vectorizedData) {
    AngularInfo res;
    res.Actuator1 = (float)vectorizedData[0];
    res.Actuator2 = (float)vectorizedData[1];
    res.Actuator3 = (float)vectorizedData[2];
    res.Actuator4 = (float)vectorizedData[3];
    res.Actuator5 = (float)vectorizedData[4];
    res.Actuator6 = (float)vectorizedData[5];
    return res;
}

std::vector<double> Robot::convertAngularPositionToVector(AngularPosition overallData) {
    std::vector<double> vectorizedData;
    vectorizedData.push_back((double)overallData.Actuators.Actuator1);
    vectorizedData.push_back((double)overallData.Actuators.Actuator2);
    vectorizedData.push_back((double)overallData.Actuators.Actuator3);
    vectorizedData.push_back((double)overallData.Actuators.Actuator4);
    vectorizedData.push_back((double)overallData.Actuators.Actuator5);
    vectorizedData.push_back((double)overallData.Actuators.Actuator6);
    
    return vectorizedData;
}

