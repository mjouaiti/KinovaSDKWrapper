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

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <chrono>
#include <fstream>

#include "../KinovaTypes.h"
#include "../Kinova.API.UsbCommandLayerUbuntu.h"

class Robot {
public:
    Robot(const std::string& libPath);
    ~Robot();
    void tryAPI();
    
    /* Robot-API.cpp */
    void closeAPI();
    void initializeAPI();
    void programFlash(const std::string& filename);
    std::vector<KinovaDevice> getDevices(int &result);
    void refresDevicesList();
    void setActiveDevice(KinovaDevice device);
    void setClientConfigurations(ClientConfigurations clientConfigurations);
    void setModel(const std::string& command, const std::string& temp);
    void setSerialNumber(const std::string& command, const std::string& temp);
    
    /* Robot-phy-parameters.cpp */
    void setPID(std::vector<std::vector<float> > newPID);
    void setPID(int actuatorNumber, std::vector<float> newPID);
    void setActuatorAdress(const int actualAdress, const int newAdress);
    void setSpasmFilterValues(std::vector<float>command, int activationStatus);
    
    /* Robot-measures.cpp */
    void updateTrajectoryInfo();
    int getTrajectoryCount();
    std::vector<float> getAngularCurrent();
    std::vector<float> getAngularCurrentMotor();
    std::vector<float> getAngularForce();
    std::vector<float> getAngularPosition();
    std::vector<float> getAngularVelocity();
    std::vector<std::vector<float>> getActuatorAcceleration();
    std::vector<float> getCartesianPosition();
    std::vector<float> getCartesianForce();
    std::vector<float> getEndEffectorOffset(unsigned int &status);
    std::vector<float> calibrateAngularCurrent(int numberOfIterations);
    std::vector<float> calibrateAngularCurrentMotor(int numberOfIterations);
    std::vector<float> calibrateAngularForce(int numberOfIterations);
    std::vector<float> calibrateAngularPosition(int numberOfIterations);
    std::vector<float> calibrateAngularVelocity(int numberOfIterations);
    std::vector<float> getSpasmFilterValues(int &activationStatus);
    
    /* Robot-move.cpp */
    void setVelocity(int actuator, double newVelocity);
    void setVelocity(std::vector<float> newVelocity);
    void setFingerVelocity(int fingerNumber, double newVelocity);
    void setFingerPosition(int fingerNumber, double newPosition);
    void setEndEffectorOffset(bool applied, std::vector<float> offset);
    
    void setJointZero(int actuatorNumber);
    void setPosition(int actuatorNumber, double pointToSend);
    void setPosition(std::vector<float> pointToSend);
    void moveFromCurrentPosition(int actuatorNumber, double deltaTheta);
    void moveFromCurrentPosition(std::vector<float> deltaVector);
    void setPositionTrajectoryLimitations(double maxSpeed1, double maxSpeed2);

    void setCartesianForce(int actuator, double newForce);
    void setCartesianForce(std::vector<float> newForce);
    
    void eraseTrajectories();
    
    /* Robot-control.cpp */
    void setAngularControl();
    void setCartesianControl();
    
    void setControlMapping(ControlMappingCharts command);
    
    /* Robot-time.cpp */
    void startClock();
    void updateClock();
    double getMicroseconds();
    double getSeconds();
    
    static const int ACTUATORS_COUNT;
    static const int FIRST_ACTUATOR_PID_ADDRESS;
    static const double MAX_P;
    static const double MAX_D;
    static const double MIN_PID;
    static const double DEFAULT_P;
    static const double DEFAULT_I;
    static const double DEFAUTL_D;
    
private:
    /* ---------- FIELDS ---------- */
    /* API fields */
    void* commandLayerHandle;
    /* Measures fields */
    AngularPosition angularCurrent;
    AngularPosition angularCurrentMotor;
    AngularPosition angularForce;
    AngularPosition angularPosition;
    AngularPosition angularVelocity;
    CartesianPosition cartesianForce;
    CartesianPosition cartesianPosition;
    TrajectoryFIFO trajectoryInfo;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime, currentTime;
    /* Move fields */
    TrajectoryPoint positionTrajectory;
    TrajectoryPoint velocityTrajectory;
    /* File fields */
    std::fstream measuresFile;
    
    float CartForceCommand[6];
    
    /* ---------- METHODS ---------- */
    /* Robot-utils.cpp */
    static double getActuatorAngularInfo(int actuatorNumber, AngularInfo overallInfo);
    static AngularInfo setActuatorAngularValue(int actuatorNumber, double newValue, AngularInfo overallInfo);
    static std::vector<float> convertAngularPositionToVector(AngularPosition overallData);
    static AngularInfo convertVectorToAngularInfo(std::vector<float> vectorizedData);
    /* Robot-measure.cpp */
    std::vector<float> calibrateAngularData(int (*MyGetAngularData)(AngularPosition &), int numberOfIterations);
    /* Robot-move.cpp */
    void initializeVelocityTrajectory();
    void initializePositionTrajectory();
    
    /* ---------- API STUFF ---------- */
    int (*MyClearErrorLog)();
    int (*MyCloseAPI)();
    int (*MyEraseAllProtectionZones)();
    int (*MyEraseAllTrajectories)();
    int (*MyGetActualTrajectoryInfo)(TrajectoryPoint &);
    int (*MyGetAngularCurrent)(AngularPosition &);
    int (*MyGetAngularCurrentMotor)(AngularPosition &);
    int (*MyGetAngularForce)(AngularPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetAngularVelocity)(AngularPosition &);
    int (*MyGetAPIVersion)(int Response[API_VERSION_COUNT]);
    int (*MyGetCartesianCommand)(CartesianPosition &);
    int (*MyGetCartesianForce)(CartesianPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
    int (*MyGetClientConfigurations)(ClientConfigurations &command);
    int (*MyGetCodeVersion)(int Response[CODE_VERSION_COUNT]);
    int (*MyGetControlMapping)(ControlMappingCharts &);
    int (*MyGetControlType)(int &);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*MyGetForcesInfo)(ForcesInfo &Response);
    int (*MyGetGeneralInformations)(GeneralInformations &);
    int (*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*MyGetGripperStatus)(Gripper &);
    int (*MyGetProtectionZone)(ZoneList &Response);
    int (*MyGetQuickStatus)(QuickStatus &Response);
    int (*MyGetSensorsInfo)(SensorsInfo &);
    int (*MyGetSingularityVector)(SingularityVector &);
    int (*MyGetSpasmFilterValues)(float Response[SPASM_FILTER_COUNT], int &activationStatus);
    int (*MyGetSystemError)(unsigned int, SystemError &);
    int (*MyGetSystemErrorCount)(unsigned int &);
    int (*MyInitAPI)();
    int (*MyInitFingers)();
    int (*MyMoveHome)(int &);
    int (*MyProgramFlash)(const char *filename);
    int (*MyRefresDevicesList)(void);
    int (*MyRestoreFactoryDefault)();
    int (*MySendAdvanceTrajectory)(TrajectoryPoint);
    int (*MySendBasicTrajectory)(TrajectoryPoint);
    int (*MySendJoystickCommand)(JoystickCommand command);
    int (*MySetActiveDevice)(KinovaDevice device);
    int (*MySetActuatorAdress)(int ActuatorAdress, int newAdress);
    int (*MySetActuatorPID)(unsigned int address, float P, float I, float D);
    int (*MySetActuatorPIDFilter)(int ActuatorAdress, float filterP, float filterI, float filterD);
    int (*MySetAngularControl)();
    int (*MySetCartesianControl)();
    int (*MySetClientConfigurations)(ClientConfigurations config);
    int (*MySetControlMapping)(ControlMappingCharts Command);
    int (*MySetJointZero)(int ActuatorAdress);
    int (*MySetProtectionZone)(ZoneList);
    int (*MySetSerialNumber)(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);
    int (*MySetSpasmFilterValues)(float Command[SPASM_FILTER_COUNT], int activationStatus);
    int (*MyStartControlAPI)();
    int (*MyStartCurrentLimitation)();
    int (*MyStopControlAPI)();
    int (*MyStopCurrentLimitation)();
};

#endif

