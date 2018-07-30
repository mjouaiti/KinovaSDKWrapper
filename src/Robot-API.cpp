//
//  Robot-API.cpp
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

#include <dlfcn.h>
#include <iostream>
#include <map>
#include <typeindex>

#include "Robot.h"
#include <cstring>

/**
 * This function must called when your application stops using the API. It closes the USB link and the library properly.
 */
void Robot::closeAPI()
{
	(*MyStopControlAPI)();
	(*MyCloseAPI)();
}

/**
 * Loads ALL the functions and initialises the API
 */
void Robot::initializeAPI() {
    MyClearErrorLog = (int (*)()) dlsym(commandLayerHandle, "ClearErrorLog");
    if(MyClearErrorLog == NULL) {
        std::cout << "Cannot load ClearErrorLog" << std::endl;
    }
    
    MyCloseAPI = (int (*)()) dlsym(commandLayerHandle, "CloseAPI");
    if(MyCloseAPI == NULL) {
        std::cout << "Cannot load CloseAPI" << std::endl;
    }
    
    MyInitAPI = (int (*)()) dlsym(commandLayerHandle, "InitAPI");
    if(MyInitAPI == NULL) {
        std::cout << "Cannot load InitAPI" << std::endl;
    }
    
    MyInitFingers = (int (*)()) dlsym(commandLayerHandle, "InitFingers");
    if(MyInitFingers == NULL) {
        std::cout << "Cannot load InitFingers" << std::endl;
    }
    
    MyEraseAllProtectionZones = (int (*)()) dlsym(commandLayerHandle, "EraseAllProtectionZones");
    if(MyEraseAllProtectionZones == NULL) {
        std::cout << "Cannot load EraseAllProtectionZones" << std::endl;
    }
    
    MyEraseAllTrajectories = (int (*)()) dlsym(commandLayerHandle, "EraseAllTrajectories");
    if(MyEraseAllTrajectories == NULL) {
        std::cout << "Cannot load EraseAllTrajectories" << std::endl;
    }
    
    MyGetActualTrajectoryInfo = (int (*)(TrajectoryPoint &)) dlsym(commandLayerHandle, "GetActualTrajectoryInfo");
    if(MyGetActualTrajectoryInfo == NULL) {
        std::cout << "Cannot load GetActualTrajectoryInfo" << std::endl;
    }
    
    MyGetAngularCurrent = (int (*)(AngularPosition &)) dlsym(commandLayerHandle, "GetAngularCurrent");
    if(MyGetAngularCurrent == NULL) {
        std::cout << "Cannot load GetAngularCurrent" << std::endl;
    }
    
    MyGetAngularCurrentMotor = (int (*)(AngularPosition &)) dlsym(commandLayerHandle, "GetAngularCurrentMotor");
    if(MyGetAngularCurrentMotor == NULL) {
        std::cout << "Cannot load GetAngularCurrentMotor" << std::endl;
    }
    
    MyGetAngularForce = (int (*)(AngularPosition &)) dlsym(commandLayerHandle, "GetAngularForce");
    if(MyGetAngularForce == NULL) {
        std::cout << "Cannot load GetAngularForce" << std::endl;
    }
    
    MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayerHandle, "GetAngularPosition");
    if(MyGetAngularPosition == NULL) {
        std::cout << "Cannot load GetAngularPosition" << std::endl;
    }
    
    MyGetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayerHandle, "GetAngularVelocity");
    if(MyGetAngularVelocity == NULL) {
        std::cout << "Cannot load GetAngularVelocity" << std::endl;
    }
    
    MyGetAPIVersion = (int (*)(int Response[API_VERSION_COUNT])) dlsym(commandLayerHandle, "GetAPIVersion");
    if(MyGetAPIVersion == NULL) {
        std::cout << "Cannot load GetAPIVersion" << std::endl;
    }
    
    MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayerHandle, "GetCartesianCommand");
    if(MyGetCartesianCommand == NULL) {
        std::cout << "Cannot load GetCartesianCommand" << std::endl;
    }
    
    MyGetCartesianForce = (int (*)(CartesianPosition &)) dlsym(commandLayerHandle, "GetCartesianForce");
    if(MyGetCartesianForce == NULL) {
        std::cout << "Cannot load GetCartesianForce" << std::endl;
    }
    
    MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayerHandle, "GetCartesianPosition");
    if(MyGetCartesianPosition == NULL) {
        std::cout << "Cannot load GetCartesianPosition" << std::endl;
    }
    
    MyGetClientConfigurations = (int (*)(ClientConfigurations &)) dlsym(commandLayerHandle, "GetClientConfigurations");
    if(MyGetClientConfigurations == NULL) {
        std::cout << "Cannot load GetClientConfigurations" << std::endl;
    }
    
    MyGetClientConfigurations = (int (*)(ClientConfigurations &)) dlsym(commandLayerHandle, "GetClientConfigurations");
    if(MyGetClientConfigurations == NULL) {
        std::cout << "Cannot load GetClientConfigurations" << std::endl;
    }
    
    MyGetCodeVersion = (int (*)(int Response[CODE_VERSION_COUNT])) dlsym(commandLayerHandle, "GetCodeVersion");
    if(MyGetCodeVersion == NULL) {
        std::cout << "Cannot load GetCodeVersion" << std::endl;
    }
    
    MyGetControlMapping = (int (*)(ControlMappingCharts &)) dlsym(commandLayerHandle, "GetControlMapping");
    if(MyGetControlMapping == NULL) {
        std::cout << "Cannot load GetControlMapping" << std::endl;
    }
    
    MyGetControlType = (int (*)(int &)) dlsym(commandLayerHandle, "GetControlType");
    if(MyGetControlType == NULL) {
        std::cout << "Cannot load GetControlType" << std::endl;
    }
    
    MyGetDevices = (int (*)(KinovaDevice *, int &)) dlsym(commandLayerHandle, "GetDevices");
    if(MyGetDevices == NULL) {
        std::cout << "Cannot load GetDevices" << std::endl;
    }
    
    MyGetGeneralInformations = (int (*)(GeneralInformations &)) dlsym(commandLayerHandle, "GetGeneralInformations");
    if(MyGetGeneralInformations == NULL) {
        std::cout << "Cannot load GetGeneralInformations" << std::endl;
    }
    
    MyGetGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &)) dlsym(commandLayerHandle, "GetGlobalTrajectoryInfo");
    if(MyGetGlobalTrajectoryInfo == NULL) {
        std::cout << "Cannot load GetGlobalTrajectoryInfo" << std::endl;
    }
    
    MyGetGripperStatus = (int (*)(Gripper &)) dlsym(commandLayerHandle, "GetGripperStatus");
    if(MyGetGripperStatus == NULL) {
        std::cout << "Cannot load GetGripperStatus" << std::endl;
    }
    
    MyGetProtectionZone = (int (*)(ZoneList &)) dlsym(commandLayerHandle, "GetProtectionZone");
    if(MyGetProtectionZone == NULL) {
        std::cout << "Cannot load GetProtectionZone" << std::endl;
    }
    
    MyGetQuickStatus = (int (*)(QuickStatus &)) dlsym(commandLayerHandle, "GetQuickStatus");
    if(MyGetQuickStatus == NULL) {
        std::cout << "Cannot load GetQuickStatus" << std::endl;
    }
    
    MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayerHandle, "GetSensorsInfo");
    if(MyGetSensorsInfo == NULL) {
        std::cout << "Cannot load GetSensorsInfo" << std::endl;
    }
    
    MyGetSingularityVector = (int (*)(SingularityVector &)) dlsym(commandLayerHandle, "GetSingularityVector");
    if(MyGetSingularityVector == NULL) {
        std::cout << "Cannot load GetSingularityVector" << std::endl;
    }
    
    MyGetSpasmFilterValues = (int (*)(float *, int &)) dlsym(commandLayerHandle, "GetSpasmFilterValues");
    if(MyGetSpasmFilterValues == NULL) {
        std::cout << "Cannot load GetSpasmFilterValues" << std::endl;
    }
    
    MyGetSystemError = (int (*)(unsigned int, SystemError
                                &)) dlsym(commandLayerHandle, "GetSystemError");
    if(MyGetSystemError == NULL) {
        std::cout << "Cannot load GetSystemError" << std::endl;
    }
    
    MyGetSystemErrorCount = (int (*)(unsigned int &)) dlsym(commandLayerHandle, "GetSystemErrorCount");
    if(MyGetSystemErrorCount == NULL) {
        std::cout << "Cannot load GetSystemErrorCount" << std::endl;
    }
    
    MyMoveHome = (int (*)(int &)) dlsym(commandLayerHandle, "MoveHome");
    if(MyMoveHome == NULL) {
        std::cout << "Cannot load MoveHome" << std::endl;
    }
    
    MyProgramFlash = (int (*)(const char *)) dlsym(commandLayerHandle, "ProgramFlash");
    if(MyProgramFlash == NULL) {
        std::cout << "Cannot load ProgramFlash" << std::endl;
    }
    
    MyRefresDevicesList = (int (*)()) dlsym(commandLayerHandle, "RefresDevicesList");
    if(MyRefresDevicesList == NULL) {
        std::cout << "Cannot load RefresDevicesList" << std::endl;
    }
    
    MyRestoreFactoryDefault = (int (*)()) dlsym(commandLayerHandle, "RestoreFactoryDefault");
    if(MyRestoreFactoryDefault == NULL) {
        std::cout << "Cannot load RestoreFactoryDefault" << std::endl;
    }
    
    MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayerHandle, "SendAdvanceTrajectory");
    if(MySendAdvanceTrajectory == NULL) {
        std::cout << "Cannot load SendAdvanceTrajectory" << std::endl;
    }
    
    MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayerHandle, "SendBasicTrajectory");
    if(MySendBasicTrajectory == NULL) {
        std::cout << "Cannot load SendBasicTrajectory" << std::endl;
    }
    
    MySendJoystickCommand = (int (*)(JoystickCommand)) dlsym(commandLayerHandle, "SendJoystickCommand");
    if(MySendJoystickCommand == NULL) {
        std::cout << "Cannot load SendJoystickCommand" << std::endl;
    }
    
    MySetActiveDevice = (int (*)(KinovaDevice)) dlsym(commandLayerHandle, "SetActiveDevice");
    if(MySetActiveDevice == NULL) {
        std::cout << "Cannot load SetActiveDevice" << std::endl;
    }
    
    MySetActuatorAdress = (int (*)(int, int)) dlsym(commandLayerHandle, "SetActuatorAdress");
    if(MySetActuatorAdress == NULL) {
        std::cout << "Cannot load SetActuatorAdress" << std::endl;
    }
    
    MySetActuatorPID = (int (*)(unsigned int, float,
                                float, float)) dlsym(commandLayerHandle, "SetActuatorPID");
    if(MySetActuatorPID == NULL) {
        std::cout << "Cannot load SetActuatorPID" << std::endl;
    }
    
    MySetActuatorPIDFilter = (int (*)(int, float, float, float)) dlsym(commandLayerHandle, "SetActuatorPIDFilter");
    if(MySetActuatorPIDFilter == NULL) {
        std::cout << "Cannot load SetActuatorPIDFilter" << std::endl;
    }
    
    MySetAngularControl = (int (*)()) dlsym(commandLayerHandle, "SetAngularControl");
    if(MySetAngularControl == NULL) {
        std::cout << "Cannot load SetAngularControl" << std::endl;
    }
    
    MySetCartesianControl = (int (*)()) dlsym(commandLayerHandle, "SetCartesianControl");
    if(MySetCartesianControl == NULL) {
        std::cout << "Cannot load SetCartesianControl" << std::endl;
    }
    
    MySetClientConfigurations = (int (*)(ClientConfigurations)) dlsym(commandLayerHandle, "SetClientConfigurations");
    if(MySetClientConfigurations == NULL) {
        std::cout << "Cannot load SetClientConfigurations" << std::endl;
    }
    
    MySetControlMapping = (int (*)(ControlMappingCharts)) dlsym(commandLayerHandle, "SetControlMapping");
    if(MySetControlMapping == NULL) {
        std::cout << "Cannot load SetControlMapping" << std::endl;
    }
    
    MySetJointZero = (int (*)(int)) dlsym(commandLayerHandle, "SetJointZero");
    if(MySetJointZero == NULL) {
        std::cout << "Cannot load SetJointZero" << std::endl;
    }
    
    MySetProtectionZone = (int (*)(ZoneList)) dlsym(commandLayerHandle, "SetProtectionZone");
    if(MySetProtectionZone == NULL) {
        std::cout << "Cannot load SetProtectionZone" << std::endl;
    }
    
    MySetSerialNumber = (int (*)(char *, char *)) dlsym(commandLayerHandle, "SetSerialNumber");
    if(MySetSerialNumber == NULL) {
        std::cout << "Cannot load SetSerialNumber" << std::endl;
    }
    
    MySetSpasmFilterValues = (int (*)(float *, int)) dlsym(commandLayerHandle, "SetSpasmFilterValues");
    if(MySetSpasmFilterValues == NULL) {
        std::cout << "Cannot load SetSpasmFilterValues" << std::endl;
    }
    
    MyStartControlAPI = (int (*)()) dlsym(commandLayerHandle, "StartControlAPI");
    if(MyStartControlAPI == NULL) {
        std::cout << "Cannot load StartControlAPI" << std::endl;
    }
    
    MyStartCurrentLimitation = (int (*)()) dlsym(commandLayerHandle, "StartCurrentLimitation");
    if(MyStartCurrentLimitation == NULL) {
        std::cout << "Cannot load StartCurrentLimitation" << std::endl;
    }
    
    MyStopControlAPI = (int (*)()) dlsym(commandLayerHandle, "StopControlAPI");
    if(MyStopControlAPI == NULL) {
        std::cout << "Cannot load StopControlAPI" << std::endl;
    }
    
    MyStopCurrentLimitation = (int (*)()) dlsym(commandLayerHandle, "StopCurrentLimitation");
    if(MyStopCurrentLimitation == NULL) {
        std::cout << "Cannot load StopCurrentLimitation" << std::endl;
    }
    
    MyStartControlAPI = (int (*)()) dlsym(commandLayerHandle, "StartControlAPI");
    if(MyStartControlAPI == NULL) {
        std::cout << "Cannot load StartControlAPI" << std::endl;
    }
    
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        CartForceCommand[i] = 0;
    }
    
    
    (*MyInitAPI)();
    int result = 0;
    std::vector<KinovaDevice> devices = getDevices(result);
    if(devices.size() < 1)
    {
        std::cerr << "No Device Connected. Check connection and make sure your device is on" << std::endl;
        closeAPI();
        std::exit(0);
    }
    (*MyInitFingers)();
    std::cout << "API Initialized" << std::endl;
}

/**
 * This function programs a new version of the robotical arm's firmware.
 * @param filename The name of the .hex file with the the path if needed.
 **/
void Robot::programFlash(const std::string& filename)
{
    std::cout << "WARNING::THIS WILL INSTALL A NEW FIRMWARE" << std::endl;
    std::cout << "Are you sure you really want to do that? (yes/no)" << std::endl;
    std::string answer = "";
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
        answer = "";
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
    return list;
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
 * @param device to activate
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

