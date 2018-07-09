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
    EthernetConfiguration* getEthernetConfiguration();
    void programFlash(const std::string& filename);
    std::vector<KinovaDevice> getDevices(int &result);
    void refresDevicesList();
    void setActiveDevice(KinovaDevice device);
    void setClientConfigurations(ClientConfigurations clientConfigurations);
    void setEthernetConfiguration(EthernetConfiguration *config);
    void setLocalMACAddress(unsigned char* mac, std::string& temp);
    void setModel(const std::string& command, const std::string& temp);
    void setSerialNumber(const std::string& command, const std::string& temp);
    
    /* Robot-phy-parameters.cpp */
    void setInertiaDamping(std::vector<float> inertia, std::vector<float> damping);
    void setCartesianInertiaDamping(std::vector<float> inertia, std::vector<float> damping);
    void setPID(std::vector<std::vector<float> > newPID);
    void setPID(int actuatorNumber, std::vector<float> newPID);
    void setPositionLimitDistance(std::vector<float>command);
    void setSpasmFilterValues(std::vector<float>command, int activationStatus);
    void setTorqueMinMax(std::vector<float> minTorque, std::vector<float> maxTorque);
    
    std::vector<float> runGravityZEstimationSequence(ROBOT_TYPE type);
    void setGravityManualInputParam(std::vector<float> command);
    void setGravityOptimalZParam(std::vector<float> command);
    void setGravityPayload(std::vector<float> command);
    void setGravityVector(std::vector<float> command);
    void setGravityType(GRAVITY_TYPE type);
    
    /* Robot-torque-control.cpp */
    void setTorqueVibrationController(float value);
    void setTorqueSafetyFactor(float factor);
    void setTorqueActuatorGain(std::vector<float> gains);
    void setTorqueBrake(std::vector<float> command);
    void setTorqueCommandMax(std::vector<float> command);
    void setTorqueDampingMax(std::vector<float> command);
    void setTorqueErrorDeadband(std::vector<float> command);
    void setTorqueErrorResend(std::vector<float> command);
    void setTorqueFeedCurrent(std::vector<float> command);
    void setTorqueFeedCurrentVoltage(std::vector<float> command);
    void setTorqueFeedFilter(std::vector<float> command);
    void setTorqueFeedVelocity(std::vector<float> command);
    void setTorqueFeedVelocityUnderGain(std::vector<float> command);
    void setTorqueFilterControlEffort(std::vector<float> command);
    void setTorqueFilterError(std::vector<float> command);
    void setTorqueFilterMeasuredTorque(std::vector<float> command);
    void setTorqueFilterVelocity(std::vector<float> command);
    void setTorqueGainMax(std::vector<float> command);
    void setTorqueInactivityTimeActuator(std::vector<float> command);
    void setTorqueInactivityTimeMainController(int time);
    void setTorqueInactivityType(int type);
    void setTorquePositionLimitDampingGain(std::vector<float> command);
    void setTorquePositionLimitDampingMax(std::vector<float> command);
    void setTorquePositionLimitRepulsGain(std::vector<float> command);
    void setTorquePositionLimitRepulsMax(std::vector<float> command);
    void setTorqueRateLimiter(std::vector<float> command);
    void setTorqueRobotProtection(int protectionLevel);
    void setTorqueStaticFriction(std::vector<float> command);
    void setTorqueStaticFrictionMax(std::vector<float> command);
    void setTorqueVelocityLimitFilter(std::vector<float> command);
    void setSwitchThreshold(std::vector<float> command);
    
    /* Robot-measures.cpp */
    void updateTrajectoryInfo();
    int getTrajectoryCount();
    std::vector<float> getAngularCurrent();
    std::vector<float> getAngularCurrentMotor();
    std::vector<float> getAngularForce();
    std::vector<float> getAngularForceGravityFree();
    std::vector<float> getAngularPosition();
    std::vector<float> getAngularVelocity();
    std::vector<std::vector<float>> getActuatorAcceleration();
    std::vector<float> getCartesianPosition();
    std::vector<float> getCartesianForce();
    std::vector<float> getAngularTorqueCommand();
    std::vector<float> getAngularTorqueGravityEstimation();
    std::vector<float> getEndEffectorOffset(unsigned int &status);
    std::vector<float> calibrateAngularCurrent(int numberOfIterations);
    std::vector<float> calibrateAngularCurrentMotor(int numberOfIterations);
    std::vector<float> calibrateAngularForce(int numberOfIterations);
    std::vector<float> calibrateAngularForceGravityFree(int numberOfIterations);
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
    
    void setTorque(int actuator, double newTorque);
    void setTorque(std::vector<float> newTorque);
    void setCartesianForce(int actuator, double newForce);
    void setCartesianForce(std::vector<float> newForce);
    
    void eraseTrajectories();
    
    /* Robot-control.cpp */
    void activateAutoNullSpaceMotionCartesian(int state);
    void activateCollisionAutomaticAvoidance(int state);
    void activateSingularityAutomaticAvoidance(int state);
    void activateExtraProtectionPinchingWrist(int state);
    
    void setAngularControl();
    void setCartesianControl();
    void startForceControl();
    void stopForceControl();
    int getTrajectoryTorqueMode();
    
    void startTorqueControl();
    void startImpedanceControl();
    void setTorqueDamping(std::vector<float> damping);
    void stopRedundantJointNullSpaceMotion();
    void startRedundantJointNullSpaceMotion();
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
    AngularPosition angularForceGravityFree;
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
    
    float TorqueCommand[6];
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
    int (*MyActivateAutoNullSpaceMotionCartesian)(int state);
    int (*MyActivateCollisionAutomaticAvoidance)(int state);
    int (*MyActivateExtraProtectionPinchingWrist)(int state);
    int (*MyActivateSingularityAutomaticAvoidance)(int state);
    int (*MyClearErrorLog)();
    int (*MyCloseAPI)();
    int (*MyEraseAllProtectionZones)();
    int (*MyEraseAllTrajectories)();
    int (*MyGetActualTrajectoryInfo)(TrajectoryPoint &);
    int (*MyGetActuatorAcceleration)(AngularAcceleration &Response);
    int (*MyGetAngularCurrent)(AngularPosition &);
    int (*MyGetAngularCurrentMotor)(AngularPosition &);
    int (*MyGetAngularForce)(AngularPosition &);
    int (*MyGetAngularForceGravityFree)(AngularPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetAngularTorqueCommand)(float Command[COMMAND_SIZE]);
    int (*MyGetAngularTorqueGravityEstimation)(float Command[COMMAND_SIZE]);
    int (*MyGetAngularVelocity)(AngularPosition &);
    int (*MyGetAPIVersion)(std::vector<int> &);
    int (*MyGetCartesianCommand)(CartesianPosition &);
    int (*MyGetCartesianForce)(CartesianPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
    int (*MyGetClientConfigurations)(ClientConfigurations &command);
    int (*MyGetCodeVersion)(std::vector<int> &);
    int (*MyGetControlMapping)(ControlMappingCharts &);
    int (*MyGetControlType)(int &);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*MyGetEndEffectorOffset)(unsigned int &status, float &x, float &y, float &z);
    int (*MyGetEthernetConfiguration)(EthernetConfiguration *config);
    int (*MyGetForcesInfo)(ForcesInfo &Response);
    int (*MyGetGeneralInformations)(GeneralInformations &);
    int (*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*MyGetGripperStatus)(Gripper &);
    int (*MyGetPositionCurrentActuators)(std::vector<float> &data);
    int (*MyGetProtectionZone)(ZoneList &Response);
    int (*MyGetQuickStatus)(QuickStatus &Response);
    int (*MyGetSensorsInfo)(SensorsInfo &);
    int (*MyGetSingularityVector)(SingularityVector &);
    int (*MyGetSpasmFilterValues)(float Response[SPASM_FILTER_COUNT], int &activationStatus);
    int (*MyGetSystemError)(unsigned int, SystemError &);
    int (*MyGetSystemErrorCount)(unsigned int &);
    int (*MyGetTrajectoryTorqueMode)(int &);
    int (*MyInitAPI)();
    int (*MyInitFingers)();
    int (*MyMoveHome)(int &);
    int (*MyProgramFlash)(const char *filename);
    int (*MyRefresDevicesList)(void);
    int (*MyRestoreFactoryDefault)();
    int (*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
    int (*MySendAdvanceTrajectory)(TrajectoryPoint);
    int (*MySendAngularTorqueCommand)(float Command[6]);
    int (*MySendBasicTrajectory)(TrajectoryPoint);
    int (*MySendCartesianForceCommand)(float Command[6]);
    int (*MySendJoystickCommand)(JoystickCommand command);
    int (*MySetActiveDevice)(KinovaDevice device);
    int (*MySetActuatorPID)(unsigned int address, float P, float I, float D);
    int (*MySetActuatorPIDFilter)(int ActuatorAdress, float filterP, float filterI, float filterD);
    int (*MySetAngularControl)();
    int (*MySetAngularInertiaDamping)(AngularInfo, AngularInfo);
    int (*MySetAngularTorqueMinMax)(AngularInfo, AngularInfo);
    int (*MySetCartesianControl)();
    int (*MySetCartesianForceMinMax)(CartesianInfo, CartesianInfo);
    int (*MySetCartesianInertiaDamping)(CartesianInfo inertia, CartesianInfo damping);
    int (*MySetClientConfigurations)(ClientConfigurations config);
    int (*MySetControlMapping)(ControlMappingCharts Command);
    int (*MySetEndEffectorOffset)(unsigned int status, float x, float y, float z);
    int (*MySetEthernetConfiguration)(EthernetConfiguration *config);
    int (*MySetFrameType)(int);
    int (*MySetGravityManualInputParam)(float Command[GRAVITY_PARAM_SIZE]);
    int (*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
    int (*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
    int (*MySetGravityType)(GRAVITY_TYPE type);
    int (*MySetGravityVector)(float gravityVector[GRAVITY_VECTOR_SIZE]);
    int (*MySetJointZero)(int ActuatorAdress);
    int (*MySetLocalMACAddress)(unsigned char mac[MAC_ADDRESS_LENGTH], char temp[STRING_LENGTH]);
    int (*MySetModel)(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);
    int (*MySetPositionLimitDistance)(float Command[COMMAND_SIZE]);
    int (*MySetProtectionZone)(ZoneList);
    int (*MySetSerialNumber)(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);
    int (*MySetSpasmFilterValues)(float Command[SPASM_FILTER_COUNT], int activationStatus);
    int (*MySetSwitchThreshold)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueActuatorDamping)(float* Command);
    int (*MySetTorqueActuatorGain)(float Command[6]);
    int (*MySetTorqueBrake)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueCommandMax)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
    int (*MySetTorqueDampingMax)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueErrorDeadband)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueErrorResend)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFeedCurrent)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFeedCurrentVoltage)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFeedFilter)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFeedVelocity)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFeedVelocityUnderGain)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFilterControlEffort)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFilterError)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFilterMeasuredTorque)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueFilterVelocity)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueGainMax)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueInactivityTimeActuator)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueInactivityTimeMainController)(int time);
    int (*MySetTorqueInactivityType)(int);
    int (*MySetTorquePositionLimitDampingGain)(float Command[COMMAND_SIZE]);
    int (*MySetTorquePositionLimitDampingMax)(float Command[COMMAND_SIZE]);
    int (*MySetTorquePositionLimitRepulsGain)(float Command[COMMAND_SIZE]);
    int (*MySetTorquePositionLimitRepulsMax)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueRateLimiter)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueRobotProtection)(int protectionLevel);
    int (*MySetTorqueSafetyFactor)(float factor);
    int (*MySetTorqueStaticFriction)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueStaticFrictionMax)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueVelocityLimitFilter)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueVibrationController)(float value);
    int (*MySetTorqueZero)(int);
    int (*MyStartControlAPI)();
    int (*MyStartCurrentLimitation)();
    int (*MyStartForceControl)();
    int (*MyStartRedundantJointNullSpaceMotion)();
    int (*MyStopControlAPI)();
    int (*MyStopCurrentLimitation)();
    int (*MyStopForceControl)();
    int (*MyStopRedundantJointNullSpaceMotion)();
    int (*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    
};

#endif

