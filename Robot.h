#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <chrono>
#include <fstream>

#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
//#include "Kinova.API.EthCommandLayerUbuntu.h"

class Robot {
public:
    Robot(const std::string& libPath, const std::string& optionFile);
    
    /* Robot-API.cpp */
    void closeAPI();
    void initializeAPI(const std::string& optionFile);
    //EthernetConfiguration* getEthernetConfiguration();
    void programFlash(const std::string& filename);
    std::vector<KinovaDevice> getDevices(int &result);
    void refresDevicesList();
    void setActiveDevice(KinovaDevice device);
    void setClientConfigurations(ClientConfigurations clientConfigurations);
    //void setEthernetConfiguration(EthernetConfiguration *config);
    void setLocalMACAddress(unsigned char* mac, std::string& temp);
    void setModel(const std::string& command, const std::string& temp);
    void setSerialNumber(const std::string& command, const std::string& temp);
    
    /* Robot-phy-parameters.cpp */
    void setInertiaDamping(std::vector<double> inertia, std::vector<double> damping);
    void setCartesianInertiaDamping(std::vector<double> inertia, std::vector<double> damping);
    void setPID(std::vector<std::vector<double> > newPID);
    void setPID(int actuatorNumber, std::vector<double> newPID);
    void setPositionLimitDistance(std::vector<double>command);
    void setSpasmFilterValues(std::vector<double>command, int activationStatus);
    void setTorqueMinMax(std::vector<double> minTorque, std::vector<double> maxTorque);
    
    void setTorqueVibrationController(float value);
    void setTorqueSafetyFactor(float factor);
    void setTorqueActuatorGain(float* gains);
    void setTorqueBrake(std::vector<double> command);
    void setTorqueCommandMax(std::vector<double> command);
    void setTorqueDampingMax(std::vector<double> command);
    void setTorqueErrorDeadband(std::vector<double> command);
    void setTorqueErrorResend(std::vector<double> command);
    void setTorqueFeedCurrent(std::vector<double> command);
    void setTorqueFeedCurrentVoltage(std::vector<double> command);
    void setTorqueFeedFilter(std::vector<double> command);
    void setTorqueFeedVelocity(std::vector<double> command);
    void setTorqueFeedVelocityUnderGain(std::vector<double> command);
    void setTorqueFilterControlEffort(std::vector<double> command);
    void setTorqueFilterError(std::vector<double> command);
    void setTorqueFilterMeasuredTorque(std::vector<double> command);
    void setTorqueFilterVelocity(std::vector<double> command);
    void setTorqueGainMax(std::vector<double> command);
    void setTorqueInactivityTimeActuator(std::vector<double> command);
    void setTorqueInactivityTimeMainController(int time);
    void setTorqueInactivityType(int type);
    void setTorquePositionLimitDampingGain(std::vector<double> command);
    void setTorquePositionLimitDampingMax(std::vector<double> command);
    void setTorquePositionLimitRepulsGain(std::vector<double> command);
    void setTorquePositionLimitRepulsMax(std::vector<double> command);
    void setTorqueRateLimiter(std::vector<double> command);
    void setTorqueRobotProtection(int protectionLevel);
    void setTorqueStaticFriction(std::vector<double> command);
    void setTorqueStaticFrictionMax(std::vector<double> command);
    void setTorqueVelocityLimitFilter(std::vector<double> command);
    void setSwitchThreshold(std::vector<double> command);
    
    std::vector<double> runGravityZEstimationSequence(ROBOT_TYPE type);
    void setGravityManualInputParam(std::vector<double> command);
    void setGravityOptimalZParam(std::vector<double> command);
    void setGravityPayload(std::vector<double> command);
    void setGravityVector(std::vector<double> command);
    void setGravityType(GRAVITY_TYPE type);
    
    /* Robot-measures.cpp */
    void updateAll();
    void updateAngularCurrent();
    void updateAngularCurrentMotor();
    void updateAngularForce();
    void updateAngularForceGravityFree();
    void updateAngularPosition();
    void updateAngularVelocity();
    void updateTrajectoryInfo();
    void updateCartesianPosition();
    void updateCartesianForce();
    double getAngularCurrent(int actuatorNumber);
    double getAngularCurrentMotor(int actuatorNumber);
    double getAngularForce(int actuatorNumber);
    double getAngularForceGravityFree(int actuatorNumber);
    double getAngularPosition(int actuatorNumber);
    double getAngularVelocity(int actuatorNumber);
    int getTrajectoryCount();
    std::vector<double> getAngularCurrent();
    std::vector<double> getAngularCurrentMotor();
    std::vector<double> getAngularForce();
    std::vector<double> getAngularForceGravityFree();
    std::vector<double> getAngularPosition();
    std::vector<double> getAngularVelocity();
    std::vector<std::vector<double>> getActuatorAcceleration();
    std::vector<double> getCartesianPosition();
    std::vector<double> getCartesianForce();
    std::vector<double> getAngularTorqueCommand();
    std::vector<double> getAngularTorqueGravityEstimation();
    std::vector<double> getEndEffectorOffset(unsigned int &status);
    std::vector<double> calibrateAngularCurrent(int numberOfIterations);
    std::vector<double> calibrateAngularCurrentMotor(int numberOfIterations);
    std::vector<double> calibrateAngularForce(int numberOfIterations);
    std::vector<double> calibrateAngularForceGravityFree(int numberOfIterations);
    std::vector<double> calibrateAngularPosition(int numberOfIterations);
    std::vector<double> calibrateAngularVelocity(int numberOfIterations);
    std::vector<double> getSpasmFilterValues(int &activationStatus);
    
    /* Robot-move.cpp */
    
    void setVelocity(int actuator, double newVelocity);
    void setVelocity(std::vector<double> newVelocity);
    void setFingerVelocity(int fingerNumber, double newVelocity);
    void setEndEffectorOffset(bool applied, std::vector<double> offset);
    
    void setJointZero(int actuatorNumber);
    void setPosition(int actuatorNumber, double pointToSend);
    void setPosition(std::vector<double> pointToSend);
    void moveFromCurrentPosition(int actuatorNumber, double deltaTheta);
    void moveFromCurrentPosition(std::vector<double> deltaVector);
    void setPositionTrajectoryLimitations(double maxSpeed1, double maxSpeed2);
    
    void setTorque(int actuator, double newTorque);
    void setTorque(std::vector<double> newTorque);
    void setCartesianForce(int actuator, double newForce);
    void setCartesianForce(std::vector<double> newForce);
    
    void eraseTrajectories();
    
    /* Robot-control.cpp */
    void activateAutoNullSpaceMotionCartesian(int state);
    void activateCollisionAutomaticAvoidance(int state);
    void activateSingularityAutomaticAvoidance(int state);
    
    void setAngularControl();
    void setCartesianControl();
    void startForceControl();
    void stopForceControl();
    int getTrajectoryTorqueMode();
    
    void startTorqueControl();
    void startImpedanceControl();
    void setTorqueDamping(std::vector<double> damping);
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
    static AngularInfo setActuatorAngularInfo(int actuatorNumber, double newValue, AngularInfo overallInfo);
    AngularInfo setFingerAngularInfo(int fingerNumber, double newValue, AngularInfo overallInfo);
    static std::vector<double> convertAngularPositionToVector(AngularPosition overallData);
    static AngularInfo convertVectorToAngularInfo(std::vector<double> vectorizedData);
    /* Robot-measure.cpp */
    std::vector<double> calibrateAngularData(int (*MyGetAngularData)(AngularPosition &), int numberOfIterations);
    /* Robot-move.cpp */
    void initializeVelocityTrajectory();
    void initializePositionTrajectory();
    
    /* ---------- API STUFF ---------- */
    int (*MyActivateAutoNullSpaceMotionCartesian)(int state);//
    int (*MyActivateCollisionAutomaticAvoidance)(int state);//
    int (*MyActivateExtraProtectionPinchingWrist)(int state);//
    int (*MyActivateSingularityAutomaticAvoidance)(int state);//
    int (*MyClearErrorLog)();
    int (*MyCloseAPI)();
    int (*MyEraseAllProtectionZones)();
    int (*MyEraseAllTrajectories)();
    int (*MyGetActualTrajectoryInfo)(TrajectoryPoint &);
    int (*MyGetActuatorAcceleration)(AngularAcceleration &Response);//
    int (*MyGetAngularCurrent)(AngularPosition &);
    int (*MyGetAngularCurrentMotor)(AngularPosition &);
    int (*MyGetAngularForce)(AngularPosition &);
    int (*MyGetAngularForceGravityFree)(AngularPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetAngularTorqueCommand)(float Command[COMMAND_SIZE]);//
    int (*MyGetAngularTorqueGravityEstimation)(float Command[COMMAND_SIZE]);//
    int (*MyGetAngularVelocity)(AngularPosition &);
    int (*MyGetAPIVersion)(std::vector<int> &);
    int (*MyGetCartesianCommand)(CartesianPosition &);
    int (*MyGetCartesianForce)(CartesianPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
    int (*MyGetClientConfigurations)(ClientConfigurations &command);
    int (*MyGetCodeVersion)(std::vector<int> &);
    int (*MyGetControlMapping)(ControlMappingCharts &);
    int (*MyGetControlType)(int &);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);//
    int (*MyGetEndEffectorOffset)(unsigned int &status, float &x, float &y, float &z);//
    //int (*MyGetEthernetConfiguration)(EthernetConfiguration *config);//
    int (*MyGetForcesInfo)(ForcesInfo &Response);
    int (*MyGetGeneralInformations)(GeneralInformations &);
    int (*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*MyGetGripperStatus)(Gripper &);
    int (*MyGetPositionCurrentActuators)(std::vector<float> &data);
    int (*MyGetProtectionZone)(ZoneList &Response);
    int (*MyGetQuickStatus)(QuickStatus &Response);
    int (*MyGetSensorsInfo)(SensorsInfo &);
    int (*MyGetSingularityVector)(SingularityVector &);
    int (*MyGetSpasmFilterValues)(float Response[SPASM_FILTER_COUNT], int &activationStatus);//
    int (*MyGetSystemError)(unsigned int, SystemError &);
    int (*MyGetSystemErrorCount)(unsigned int &);
    int (*MyGetTrajectoryTorqueMode)(int &);//
    int (*MyInitAPI)();
    int (*MyInitFingers)();
    int (*MyMoveHome)(int &);
    int (*MyProgramFlash)(const char *filename);//
    int (*MyRefresDevicesList)(void);//
    int (*MyRestoreFactoryDefault)();
    int (*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);//
    int (*MySendAdvanceTrajectory)(TrajectoryPoint);
    int (*MySendAngularTorqueCommand)(float Command[6]);
    int (*MySendBasicTrajectory)(TrajectoryPoint);
    int (*MySendCartesianForceCommand)(float Command[6]);
    int (*MySendJoystickCommand)(JoystickCommand command);
    int (*MySetActiveDevice)(KinovaDevice device);//
    int (*MySetActuatorPID)(unsigned int address, float P, float I, float D);
    int (*MySetActuatorPIDFilter)(int ActuatorAdress, float filterP, float filterI, float filterD);
    int (*MySetAngularControl)();
    int (*MySetAngularInertiaDamping)(AngularInfo, AngularInfo);
    int (*MySetAngularTorqueMinMax)(AngularInfo, AngularInfo);
    int (*MySetCartesianControl)();
    int (*MySetCartesianForceMinMax)(CartesianInfo, CartesianInfo);
    int (*MySetCartesianInertiaDamping)(CartesianInfo inertia, CartesianInfo damping);//
    int (*MySetClientConfigurations)(ClientConfigurations config);//
    int (*MySetControlMapping)(ControlMappingCharts Command);//
    int (*MySetEndEffectorOffset)(unsigned int status, float x, float y, float z);//
    //int (*MySetEthernetConfiguration)(EthernetConfiguration *config);//
    int (*MySetFrameType)(int);
    int (*MySetGravityManualInputParam)(float Command[GRAVITY_PARAM_SIZE]);//
    int (*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);//
    int (*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);//
    int (*MySetGravityType)(GRAVITY_TYPE type);//
    int (*MySetGravityVector)(float gravityVector[GRAVITY_VECTOR_SIZE]);//
    int (*MySetJointZero)(int ActuatorAdress);//
    int (*MySetLocalMACAddress)(unsigned char mac[MAC_ADDRESS_LENGTH], char temp[STRING_LENGTH]);//
    int (*MySetModel)(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);//
    int (*MySetPositionLimitDistance)(float Command[COMMAND_SIZE]);//
    int (*MySetProtectionZone)(ZoneList);
    int (*MySetSerialNumber)(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);//
    int (*MySetSpasmFilterValues)(float Command[SPASM_FILTER_COUNT], int activationStatus);//
    int (*MySetSwitchThreshold)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueActuatorDamping)(float* Command);
    int (*MySetTorqueActuatorGain)(float Command[6]);
    int (*MySetTorqueBrake)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueCommandMax)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
    int (*MySetTorqueDampingMax)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueErrorDeadband)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueErrorResend)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFeedCurrent)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFeedCurrentVoltage)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFeedFilter)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFeedVelocity)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFeedVelocityUnderGain)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFilterControlEffort)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFilterError)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFilterMeasuredTorque)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueFilterVelocity)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueGainMax)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueInactivityTimeActuator)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueInactivityTimeMainController)(int time);//
    int (*MySetTorqueInactivityType)(int);//
    int (*MySetTorquePositionLimitDampingGain)(float Command[COMMAND_SIZE]);//
    int (*MySetTorquePositionLimitDampingMax)(float Command[COMMAND_SIZE]);//
    int (*MySetTorquePositionLimitRepulsGain)(float Command[COMMAND_SIZE]);//
    int (*MySetTorquePositionLimitRepulsMax)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueRateLimiter)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueRobotProtection)(int protectionLevel);//
    int (*MySetTorqueSafetyFactor)(float factor);
    int (*MySetTorqueStaticFriction)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueStaticFrictionMax)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueVelocityLimitFilter)(float Command[COMMAND_SIZE]);//
    int (*MySetTorqueVibrationController)(float value);
    int (*MySetTorqueZero)(int);
    int (*MyStartControlAPI)();
    int (*MyStartCurrentLimitation)();
    int (*MyStartForceControl)();
    int (*MyStartRedundantJointNullSpaceMotion)();//
    int (*MyStopControlAPI)();
    int (*MyStopCurrentLimitation)();
    int (*MyStopForceControl)();
    int (*MyStopRedundantJointNullSpaceMotion)();//
    int (*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    
};

#endif

