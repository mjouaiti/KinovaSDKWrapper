#include <dlfcn.h>
#include <iostream>
#include <map>
#include <typeindex>

#include "Robot.h"

/**
 * This function must called when your application stops using the API. It closes the USB link and the library properly.
 */
void Robot::closeAPI()
{
	(*MyStopControlAPI)();
	(*MyCloseAPI)();
}
typedef void (*voidFunctionType)(void);

struct FunctionsMap{
    
    std::map<std::string,std::pair<voidFunctionType,std::type_index>> m1;
    
    template<typename T>
    void insert(std::string s1, T f1){
        auto tt = std::type_index(typeid(f1));
        m1.insert(std::make_pair(s1,
                                 std::make_pair((voidFunctionType)f1,tt)));
    }
    
    template<typename T,typename... Args>
    T searchAndCall(std::string s1, Args&&... args){
        auto mapIter = m1.find(s1);
        auto mapVal = mapIter->second;
        
        // auto typeCastedFun = reinterpret_cast<T(*)(Args ...)>(mapVal.first);
        auto typeCastedFun = (T(*)(Args ...))(mapVal.first);
        
        //compare the types is equal or not
        assert(mapVal.second == std::type_index(typeid(typeCastedFun)));
        return typeCastedFun(std::forward<Args>(args)...);
    }
};

/**
 * Loads ALL the functions and initialises the API
 */
void Robot::initializeAPI() {
    FunctionsMap map;
    map.insert("ActivateAutoNullSpaceMotionCartesian", (*MyActivateAutoNullSpaceMotionCartesian));
    map.insert("ActivateCollisionAutomaticAvoidance", (*MyActivateCollisionAutomaticAvoidance));
    map.insert("ActivateExtraProtectionPinchingWrist", (*MyActivateExtraProtectionPinchingWrist));
    map.insert("ActivateSingularityAutomaticAvoidance", (*MyActivateSingularityAutomaticAvoidance));
    map.insert("ClearErrorLog", (*MyClearErrorLog));
    map.insert("EraseAllProtectionZones", (*MyEraseAllProtectionZones));
    map.insert("EraseAllTrajectories", (*MyEraseAllTrajectories));
    map.insert("GetActualTrajectoryInfo", (*MyGetActualTrajectoryInfo));
    map.insert("GetActuatorAcceleration", (*MyGetActuatorAcceleration));
    map.insert("GetAngularCurrent", (*MyGetAngularCurrent));
    map.insert("GetAngularCurrentMotor", (*MyGetAngularCurrentMotor));
    map.insert("GetAngularForce", (*MyGetAngularForce));
    map.insert("GetAngularForceGravityFree", (*MyGetAngularForceGravityFree));
    map.insert("GetAngularPosition", (*MyGetAngularPosition));
    map.insert("GetAngularTorqueCommand", (*MyGetAngularTorqueCommand));
    map.insert("GetAngularTorqueGravityEstimation", (*MyGetAngularTorqueGravityEstimation));
    map.insert("GetAngularVelocity", (*MyGetAngularVelocity));
    map.insert("GetAPIVersion", (*MyGetAPIVersion));
    map.insert("GetCartesianCommand", (*MyGetCartesianCommand));
    map.insert("GetCartesianForce", (*MyGetCartesianForce));
    map.insert("GetCartesianPosition", (*MyGetCartesianPosition));
    map.insert("GetClientConfigurations", (*MyGetClientConfigurations));
    map.insert("GetCodeVersion", (*MyGetCodeVersion));
    map.insert("GetControlMapping", (*MyGetControlMapping));
    map.insert("GetControlType", (*MyGetControlType));
    map.insert("GetDevices", (*MyGetDevices));
    map.insert("GetEndEffectorOffset", (*MyGetEndEffectorOffset));
    //map.insert("GetEthernetConfiguration", (*MyGetEthernetConfiguration));
    map.insert("GetForcesInfo", (*MyGetForcesInfo));
    map.insert("GetGeneralInformations", (*MyGetGeneralInformations));
    map.insert("GetGlobalTrajectoryInfo", (*MyGetGlobalTrajectoryInfo));
    map.insert("GetGripperStatus", (*MyGetGripperStatus));
    map.insert("GetPositionCurrentActuators", (*MyGetPositionCurrentActuators));
    map.insert("GetProtectionZone", (*MyGetProtectionZone));
    map.insert("GetQuickStatus", (*MyGetQuickStatus));
    map.insert("GetSensorsInfo", (*MyGetSensorsInfo));
    map.insert("GetSingularityVector", (*MyGetSingularityVector));
    map.insert("GetSpasmFilterValues", (*MyGetSpasmFilterValues));
    map.insert("GetSystemError", (*MyGetSystemError));
    map.insert("GetSystemErrorCount", (*MyGetSystemErrorCount));
    map.insert("GetTrajectoryTorqueMode", (*MyGetTrajectoryTorqueMode));
    map.insert("InitFingers", (*MyInitFingers));
    map.insert("MoveHome", (*MyMoveHome));
    map.insert("ProgramFlash", (*MyProgramFlash));
    map.insert("RefresDevicesList", (*MyRefresDevicesList));
    map.insert("RestoreFactoryDefault", (*MyRestoreFactoryDefault));
    map.insert("RunGravityZEstimationSequence", (*MyRunGravityZEstimationSequence));
    map.insert("SendAdvanceTrajectory", (*MySendAdvanceTrajectory));
    map.insert("SendAngularTorqueCommand", (*MySendAngularTorqueCommand));
    map.insert("SendBasicTrajectory", (*MySendBasicTrajectory));
    map.insert("SendCartesianForceCommand", (*MySendCartesianForceCommand));
    map.insert("SendJoystickCommand", (*MySendJoystickCommand));
    map.insert("SetActiveDevice", (*MySetActiveDevice));
    map.insert("SetActuatorPID", (*MySetActuatorPID));
    map.insert("SetActuatorPIDFilter", (*MySetActuatorPIDFilter));
    map.insert("SetAngularControl", (*MySetAngularControl));
    map.insert("SetAngularInertiaDamping", (*MySetAngularInertiaDamping));
    map.insert("SetAngularTorqueMinMax", (*MySetAngularTorqueMinMax));
    map.insert("SetCartesianControl", (*MySetCartesianControl));
    map.insert("SetCartesianForceMinMax", (*MySetCartesianForceMinMax));
    map.insert("SetCartesianInertiaDamping", (*MySetCartesianInertiaDamping));
    map.insert("SetClientConfigurations", (*MySetClientConfigurations));
    map.insert("SetControlMapping", (*MySetControlMapping));
    map.insert("SetEndEffectorOffset", (*MySetEndEffectorOffset));
    //map.insert("SetEthernetConfiguration", (*MySetEthernetConfiguration));
    map.insert("SetFrameType", (*MySetFrameType));
    map.insert("SetGravityManualInputParam", (*MySetGravityManualInputParam));
    map.insert("SetGravityOptimalZParam", (*MySetGravityOptimalZParam));
    map.insert("SetGravityPayload", (*MySetGravityPayload));
    map.insert("SetGravityType", (*MySetGravityType));
    map.insert("SetGravityVector", (*MySetGravityVector));
    map.insert("SetJointZero", (*MySetJointZero));
    map.insert("SetLocalMACAddress", (*MySetLocalMACAddress));
    map.insert("SetModel", (*MySetModel));
    map.insert("SetPositionLimitDistance", (*MySetPositionLimitDistance));
    map.insert("SetProtectionZone", (*MySetProtectionZone));
    map.insert("SetSerialNumber", (*MySetSerialNumber));
    map.insert("SetSpasmFilterValues", (*MySetSpasmFilterValues));
    map.insert("SetSwitchThreshold", (*MySetSwitchThreshold));
    map.insert("SetTorqueActuatorDamping", (*MySetTorqueActuatorDamping));
    map.insert("SetTorqueActuatorGain", (*MySetTorqueActuatorGain));
    map.insert("SetTorqueBrake", (*MySetTorqueBrake));
    map.insert("SetTorqueCommandMax", (*MySetTorqueCommandMax));
    map.insert("SetTorqueControlType", (*MySetTorqueControlType));
    map.insert("SetTorqueDampingMax", (*MySetTorqueDampingMax));
    map.insert("SetTorqueErrorDeadband", (*MySetTorqueErrorDeadband));
    map.insert("SetTorqueErrorResend", (*MySetTorqueErrorResend));
    map.insert("SetTorqueFeedCurrent", (*MySetTorqueFeedCurrent));
    map.insert("SetTorqueFeedCurrentVoltage", (*MySetTorqueFeedCurrentVoltage));
    map.insert("SetTorqueFeedFilter", (*MySetTorqueFeedFilter));
    map.insert("SetTorqueFeedVelocity", (*MySetTorqueFeedVelocity));
    map.insert("SetTorqueFeedVelocityUnderGain", (*MySetTorqueFeedVelocityUnderGain));
    map.insert("SetTorqueFilterControlEffort", (*MySetTorqueFilterControlEffort));
    map.insert("SetTorqueFilterError", (*MySetTorqueFilterError));
    map.insert("SetTorqueFilterMeasuredTorque", (*MySetTorqueFilterMeasuredTorque));
    map.insert("SetTorqueFilterVelocity", (*MySetTorqueFilterVelocity));
    map.insert("SetTorqueGainMax", (*MySetTorqueGainMax));
    map.insert("SetTorqueInactivityTimeActuator", (*MySetTorqueInactivityTimeActuator));
    map.insert("SetTorqueInactivityTimeMainController", (*MySetTorqueInactivityTimeMainController));
    map.insert("SetTorqueInactivityType", (*MySetTorqueInactivityType));
    map.insert("SetTorquePositionLimitDampingGain", (*MySetTorquePositionLimitDampingGain));
    map.insert("SetTorquePositionLimitDampingMax", (*MySetTorquePositionLimitDampingMax));
    map.insert("SetTorquePositionLimitRepulsGain", (*MySetTorquePositionLimitRepulsGain));
    map.insert("SetTorquePositionLimitRepulsMax", (*MySetTorquePositionLimitRepulsMax));
    map.insert("SetTorqueRateLimiter", (*MySetTorqueRateLimiter));
    map.insert("SetTorqueRobotProtection", (*MySetTorqueRobotProtection));
    map.insert("SetTorqueSafetyFactor", (*MySetTorqueSafetyFactor));
    map.insert("SetTorqueStaticFriction", (*MySetTorqueStaticFriction));
    map.insert("SetTorqueStaticFrictionMax", (*MySetTorqueStaticFrictionMax));
    map.insert("SetTorqueVelocityLimitFilter", (*MySetTorqueVelocityLimitFilter));
    map.insert("SetTorqueVibrationController", (*MySetTorqueVibrationController));
    map.insert("SetTorqueZero", (*MySetTorqueZero));
    map.insert("StartControlAPI", (*MyStartControlAPI));
    map.insert("StartCurrentLimitation", (*MyStartCurrentLimitation));
    map.insert("StartForceControl", (*MyStartForceControl));
    map.insert("StartRedundantJointNullSpaceMotion", (*MyStartRedundantJointNullSpaceMotion));
    map.insert("StopControlAPI", (*MyStopControlAPI));
    map.insert("StopCurrentLimitation", (*MyStopCurrentLimitation));
    map.insert("StopForceControl", (*MyStopForceControl));
    map.insert("StopRedundantJointNullSpaceMotion", (*MyStopRedundantJointNullSpaceMotion));
    map.insert("SwitchTrajectoryTorque", (*MySwitchTrajectoryTorque));
    
    
    for(auto mapIter: map)
    {
        auto mapVal = mapIter->second;
        auto typeCastedFun = (int(*)())(mapVal.first);
        typeCastedFun = (int (*)()) dlsym(commandLayerHandle, line.c_str());
        if(typeCastedFun == NULL)
            std::cout << "Cannot load " << line << std::endl;
    }
    
    MyCloseAPI = (int (*)()) dlsym(commandLayerHandle, "CloseAPI");
    if (MyCloseAPI == NULL) {
        std::cout << "Cannot load CloseAPI" << std::endl;
    }
    
    MyInitAPI = (int (*)()) dlsym(commandLayerHandle, "InitAPI");
    if (MyInitAPI == NULL) {
        std::cout << "Cannot load InitAPI" << std::endl;
    }
    
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        CartForceCommand[i] = 0;
        TorqueCommand[i] = 0;
    }
    
    
    (*MyInitAPI)();
    std::cout << "API Initialized" << std::endl;
}

/**
 * Loads the functions and initialises the API
 * @param optionFile path of the file containing the setup
 */
void Robot::initializeAPI(const std::string& optionFile) {
    FunctionsMap map;
    map.insert("ActivateAutoNullSpaceMotionCartesian", (*MyActivateAutoNullSpaceMotionCartesian));
    map.insert("ActivateCollisionAutomaticAvoidance", (*MyActivateCollisionAutomaticAvoidance));
    map.insert("ActivateExtraProtectionPinchingWrist", (*MyActivateExtraProtectionPinchingWrist));
    map.insert("ActivateSingularityAutomaticAvoidance", (*MyActivateSingularityAutomaticAvoidance));
    map.insert("ClearErrorLog", (*MyClearErrorLog));
    map.insert("EraseAllProtectionZones", (*MyEraseAllProtectionZones));
    map.insert("EraseAllTrajectories", (*MyEraseAllTrajectories));
    map.insert("GetActualTrajectoryInfo", (*MyGetActualTrajectoryInfo));
    map.insert("GetActuatorAcceleration", (*MyGetActuatorAcceleration));
    map.insert("GetAngularCurrent", (*MyGetAngularCurrent));
    map.insert("GetAngularCurrentMotor", (*MyGetAngularCurrentMotor));
    map.insert("GetAngularForce", (*MyGetAngularForce));
    map.insert("GetAngularForceGravityFree", (*MyGetAngularForceGravityFree));
    map.insert("GetAngularPosition", (*MyGetAngularPosition));
    map.insert("GetAngularTorqueCommand", (*MyGetAngularTorqueCommand));
    map.insert("GetAngularTorqueGravityEstimation", (*MyGetAngularTorqueGravityEstimation));
    map.insert("GetAngularVelocity", (*MyGetAngularVelocity));
    map.insert("GetAPIVersion", (*MyGetAPIVersion));
    map.insert("GetCartesianCommand", (*MyGetCartesianCommand));
    map.insert("GetCartesianForce", (*MyGetCartesianForce));
    map.insert("GetCartesianPosition", (*MyGetCartesianPosition));
    map.insert("GetClientConfigurations", (*MyGetClientConfigurations));
    map.insert("GetCodeVersion", (*MyGetCodeVersion));
    map.insert("GetControlMapping", (*MyGetControlMapping));
    map.insert("GetControlType", (*MyGetControlType));
    map.insert("GetDevices", (*MyGetDevices));
    map.insert("GetEndEffectorOffset", (*MyGetEndEffectorOffset));
    //map.insert("GetEthernetConfiguration", (*MyGetEthernetConfiguration));
    map.insert("GetForcesInfo", (*MyGetForcesInfo));
    map.insert("GetGeneralInformations", (*MyGetGeneralInformations));
    map.insert("GetGlobalTrajectoryInfo", (*MyGetGlobalTrajectoryInfo));
    map.insert("GetGripperStatus", (*MyGetGripperStatus));
    map.insert("GetPositionCurrentActuators", (*MyGetPositionCurrentActuators));
    map.insert("GetProtectionZone", (*MyGetProtectionZone));
    map.insert("GetQuickStatus", (*MyGetQuickStatus));
    map.insert("GetSensorsInfo", (*MyGetSensorsInfo));
    map.insert("GetSingularityVector", (*MyGetSingularityVector));
    map.insert("GetSpasmFilterValues", (*MyGetSpasmFilterValues));
    map.insert("GetSystemError", (*MyGetSystemError));
    map.insert("GetSystemErrorCount", (*MyGetSystemErrorCount));
    map.insert("GetTrajectoryTorqueMode", (*MyGetTrajectoryTorqueMode));
    map.insert("InitFingers", (*MyInitFingers));
    map.insert("MoveHome", (*MyMoveHome));
    map.insert("ProgramFlash", (*MyProgramFlash));
    map.insert("RefresDevicesList", (*MyRefresDevicesList));
    map.insert("RestoreFactoryDefault", (*MyRestoreFactoryDefault));
    map.insert("RunGravityZEstimationSequence", (*MyRunGravityZEstimationSequence));
    map.insert("SendAdvanceTrajectory", (*MySendAdvanceTrajectory));
    map.insert("SendAngularTorqueCommand", (*MySendAngularTorqueCommand));
    map.insert("SendBasicTrajectory", (*MySendBasicTrajectory));
    map.insert("SendCartesianForceCommand", (*MySendCartesianForceCommand));
    map.insert("SendJoystickCommand", (*MySendJoystickCommand));
    map.insert("SetActiveDevice", (*MySetActiveDevice));
    map.insert("SetActuatorPID", (*MySetActuatorPID));
    map.insert("SetActuatorPIDFilter", (*MySetActuatorPIDFilter));
    map.insert("SetAngularControl", (*MySetAngularControl));
    map.insert("SetAngularInertiaDamping", (*MySetAngularInertiaDamping));
    map.insert("SetAngularTorqueMinMax", (*MySetAngularTorqueMinMax));
    map.insert("SetCartesianControl", (*MySetCartesianControl));
    map.insert("SetCartesianForceMinMax", (*MySetCartesianForceMinMax));
    map.insert("SetCartesianInertiaDamping", (*MySetCartesianInertiaDamping));
    map.insert("SetClientConfigurations", (*MySetClientConfigurations));
    map.insert("SetControlMapping", (*MySetControlMapping));
    map.insert("SetEndEffectorOffset", (*MySetEndEffectorOffset));
    //map.insert("SetEthernetConfiguration", (*MySetEthernetConfiguration));
    map.insert("SetFrameType", (*MySetFrameType));
    map.insert("SetGravityManualInputParam", (*MySetGravityManualInputParam));
    map.insert("SetGravityOptimalZParam", (*MySetGravityOptimalZParam));
    map.insert("SetGravityPayload", (*MySetGravityPayload));
    map.insert("SetGravityType", (*MySetGravityType));
    map.insert("SetGravityVector", (*MySetGravityVector));
    map.insert("SetJointZero", (*MySetJointZero));
    map.insert("SetLocalMACAddress", (*MySetLocalMACAddress));
    map.insert("SetModel", (*MySetModel));
    map.insert("SetPositionLimitDistance", (*MySetPositionLimitDistance));
    map.insert("SetProtectionZone", (*MySetProtectionZone));
    map.insert("SetSerialNumber", (*MySetSerialNumber));
    map.insert("SetSpasmFilterValues", (*MySetSpasmFilterValues));
    map.insert("SetSwitchThreshold", (*MySetSwitchThreshold));
    map.insert("SetTorqueActuatorDamping", (*MySetTorqueActuatorDamping));
    map.insert("SetTorqueActuatorGain", (*MySetTorqueActuatorGain));
    map.insert("SetTorqueBrake", (*MySetTorqueBrake));
    map.insert("SetTorqueCommandMax", (*MySetTorqueCommandMax));
    map.insert("SetTorqueControlType", (*MySetTorqueControlType));
    map.insert("SetTorqueDampingMax", (*MySetTorqueDampingMax));
    map.insert("SetTorqueErrorDeadband", (*MySetTorqueErrorDeadband));
    map.insert("SetTorqueErrorResend", (*MySetTorqueErrorResend));
    map.insert("SetTorqueFeedCurrent", (*MySetTorqueFeedCurrent));
    map.insert("SetTorqueFeedCurrentVoltage", (*MySetTorqueFeedCurrentVoltage));
    map.insert("SetTorqueFeedFilter", (*MySetTorqueFeedFilter));
    map.insert("SetTorqueFeedVelocity", (*MySetTorqueFeedVelocity));
    map.insert("SetTorqueFeedVelocityUnderGain", (*MySetTorqueFeedVelocityUnderGain));
    map.insert("SetTorqueFilterControlEffort", (*MySetTorqueFilterControlEffort));
    map.insert("SetTorqueFilterError", (*MySetTorqueFilterError));
    map.insert("SetTorqueFilterMeasuredTorque", (*MySetTorqueFilterMeasuredTorque));
    map.insert("SetTorqueFilterVelocity", (*MySetTorqueFilterVelocity));
    map.insert("SetTorqueGainMax", (*MySetTorqueGainMax));
    map.insert("SetTorqueInactivityTimeActuator", (*MySetTorqueInactivityTimeActuator));
    map.insert("SetTorqueInactivityTimeMainController", (*MySetTorqueInactivityTimeMainController));
    map.insert("SetTorqueInactivityType", (*MySetTorqueInactivityType));
    map.insert("SetTorquePositionLimitDampingGain", (*MySetTorquePositionLimitDampingGain));
    map.insert("SetTorquePositionLimitDampingMax", (*MySetTorquePositionLimitDampingMax));
    map.insert("SetTorquePositionLimitRepulsGain", (*MySetTorquePositionLimitRepulsGain));
    map.insert("SetTorquePositionLimitRepulsMax", (*MySetTorquePositionLimitRepulsMax));
    map.insert("SetTorqueRateLimiter", (*MySetTorqueRateLimiter));
    map.insert("SetTorqueRobotProtection", (*MySetTorqueRobotProtection));
    map.insert("SetTorqueSafetyFactor", (*MySetTorqueSafetyFactor));
    map.insert("SetTorqueStaticFriction", (*MySetTorqueStaticFriction));
    map.insert("SetTorqueStaticFrictionMax", (*MySetTorqueStaticFrictionMax));
    map.insert("SetTorqueVelocityLimitFilter", (*MySetTorqueVelocityLimitFilter));
    map.insert("SetTorqueVibrationController", (*MySetTorqueVibrationController));
    map.insert("SetTorqueZero", (*MySetTorqueZero));
    map.insert("StartControlAPI", (*MyStartControlAPI));
    map.insert("StartCurrentLimitation", (*MyStartCurrentLimitation));
    map.insert("StartForceControl", (*MyStartForceControl));
    map.insert("StartRedundantJointNullSpaceMotion", (*MyStartRedundantJointNullSpaceMotion));
    map.insert("StopControlAPI", (*MyStopControlAPI));
    map.insert("StopCurrentLimitation", (*MyStopCurrentLimitation));
    map.insert("StopForceControl", (*MyStopForceControl));
    map.insert("StopRedundantJointNullSpaceMotion", (*MyStopRedundantJointNullSpaceMotion));
    map.insert("SwitchTrajectoryTorque", (*MySwitchTrajectoryTorque));
    
    
    std::string filename = "", line = "";
    std::ifstream file(filename);
    
    if(file.is_open())
    {
        while(getline(file, line))
        {
            auto mapIter = map.m1.find(line);
            auto mapVal = mapIter->second;
            auto typeCastedFun = (int(*)())(mapVal.first);
            typeCastedFun = (int (*)()) dlsym(commandLayerHandle, line.c_str());
            if(typeCastedFun == NULL)
               std::cout << "Cannot load " << line << std::endl;
        }
        file.close();
    }
               
	MyCloseAPI = (int (*)()) dlsym(commandLayerHandle, "CloseAPI");
	if (MyCloseAPI == NULL) {
		std::cout << "Cannot load CloseAPI" << std::endl;
	}

	MyInitAPI = (int (*)()) dlsym(commandLayerHandle, "InitAPI");
	if (MyInitAPI == NULL) {
		std::cout << "Cannot load InitAPI" << std::endl;
	}

    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        CartForceCommand[i] = 0;
        TorqueCommand[i] = 0;
    }
    
    
    (*MyInitAPI)();
    std::cout << "API Initialized" << std::endl;
}

/*EthernetConfiguration* Robot::getEthernetConfiguration()
{
    EthernetConfiguration* eConfig;
    (*MyGetEthernetConfig)(eConfig);
    return eConfig;
}*/

/**
 * This function programs a new version of the robotical arm's firmware.
 * @param filename The name of the .hex file with the the path if needed.
 **/
void Robot::programFlash(const std::string& filename)
{
    std::cout << "WARNING::THIS WILL INSTALL A NEW FIRMWARE" << std::endl;
    std::cout << "Are you sure you really want to do that? (yes/no)" << std::endl;
    std::string answer = ""
    std::cin >> answer;
    while(answer != "yes" && answer != "no")
    {
        std::cout << "Are you sure you really want to do that? (yes/no)" << std::endl;
        std::cin >> answer;
    }
    if(answer == "no")
        return;
    else
    {
        std::cout << "Really really sure? (yes/no)" << std::endl;
        answer = ""
        std::cin >> answer;
        while(answer != "yes" && answer != "no")
        {
            std::cout << "Are you sure you really want to do that? (yes/no)" << std::endl;
            std::cin >> answer;
        }
        if(answer == "no")
        return;
    }
    (*MyProgramFlash)(filename.c_str());
}

/**
 * This function returns a list of devices accessible by this API.
 * @param result Result of the operation:
 NO_ERROR_KINOVA If operation is a success
 ERROR_NO_DEVICE_FOUND If no kinova device is found on the bus
 ERROR_API_NOT_INITIALIZED If the function InitAPI() has not been called previously.
 * @return list of devices (std::vector<KinovaDevice>)
 */
std::vector<KinovaDevice> Robot::getDevices(int &result)
{
    std::vector<KinovaDevice> list;
    KinovaDevice devicesList[MAX_KINOVA_DEVICE];
    int deviceCount = (*MyGetDevices)(devicesList, result);
    for(int i = 0; i < deviceCount; i++)
    {
        list.push_back(devicesList[i]);
    }
}

/**
 * This function refresh the devices list connected on the USB bus.
 
 Returns
 NO_ERROR_KINOVA if operation is a success
 -1 Input/output error.
 -2 Invalid parameter.
 -3 Access denied (insufficient permissions)
 -4 No such device (it may have been disconnected)
 -5 Entity not found.
 -6 Resource busy.
 -7 Operation timed out.
 -8 Overflow.
 -9 Pipe error.
 -10 System call interrupted (perhaps due to signal)
 -11 Insufficient memory.
 -12 Operation not supported or unimplemented on this platform.
 -99 Other error.
 */
void Robot::refresDevicesList()
{
    (*MyRefresDevicesList)();
}

/**
 * This function sets the current active device. The active device is the device that will receive the command send by this API. If no active device is set, the first one discovered is the default active device.
 * @param device
 */
void Robot::setActiveDevice(KinovaDevice device)
{
    (*MySetActiveDevice)(device);
}

/**
 * This function sets the client configurations of the robotical arm.
 * @param clientConfigurations The new configurations.
 */
void Robot::setClientConfigurations(ClientConfigurations clientConfigurations)
{
    (*MySetClientConfigurations)(clientConfigurations);
}

/*void Robot::setEthernetConfiguration(EthernetConfiguration *config)
{
    (*MySetEthernetConfiguration)();
}*/

/**
 * This function sets the MAC address of the robot.
 * @param mac New MAC address
 * @param temp Password
 */
void Robot::setLocalMACAddress(unsigned char* mac, std::string& temp)
{
    char *t = new char[temp.length() + 1];
    strcpy(t, temp.c_str());
    (*MySetLocalMACAddress)(mac, t);
}

/**
 * Internal use only.
 */
void Robot::setModel(const std::string& command, const std::string& temp)
{
    char *t = new char[temp.length() + 1];
    strcpy(t, temp.c_str());
    char *c = new char[command.length() + 1];
    strcpy(c, command.c_str());
    (*MySetModel)(c, t);
}

/**
 * Internal use only.
 */
void Robot::setSerialNumber(const std::string& command, const std::string& temp)
{
    char *t = new char[temp.length() + 1];
    strcpy(t, temp.c_str());
    char *c = new char[command.length() + 1];
    strcpy(c, command.c_str());
    (*MySetSerialNumber)(c, t);
}

