//
//  Robot.cpp
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

#include "Robot.h"
/**
 * Robot constructor
 * @param libPath path of Kinova.API.USBCommandLayerUbuntu.so
 * @param optionFile path of the setup file
 */
void Robot::tryAPI()
{
    int state = 0;
    float Command[COMMAND_SIZE];
    AngularPosition ap;
    CartesianPosition cp;
    AngularAcceleration aa;
    AngularInfo ai;
    CartesianInfo ci;
    (*MyActivateAutoNullSpaceMotionCartesian)(state);
    (*MyActivateCollisionAutomaticAvoidance)(state);
    (*MyActivateExtraProtectionPinchingWrist)(state);
    (*MyActivateSingularityAutomaticAvoidance)(state);
    (*MyClearErrorLog)();
    (*MyCloseAPI)();
    (*MyEraseAllProtectionZones)();
    (*MyEraseAllTrajectories)();
    TrajectoryPoint trajectory;
    (*MyGetActualTrajectoryInfo)(trajectory);
    (*MyGetActuatorAcceleration)(aa);
    (*MyGetAngularCurrent)(ap);
    (*MyGetAngularCurrentMotor)(ap);
    (*MyGetAngularForce)(ap);
    (*MyGetAngularForceGravityFree)(ap);
    (*MyGetAngularPosition)(ap);
    (*MyGetAngularTorqueCommand)(Command);
    (*MyGetAngularTorqueGravityEstimation)(Command);
    (*MyGetAngularVelocity)(ap);
    std::vector<int> vec;
    (*MyGetAPIVersion)(vec);
    (*MyGetCartesianCommand)(cp);
    (*MyGetCartesianForce)(cp);
    (*MyGetCartesianPosition)(cp);
    ClientConfigurations cc;
    (*MyGetClientConfigurations)(cc);
    (*MyGetCodeVersion)(vec);
    ControlMappingCharts cmc;
    (*MyGetControlMapping)(cmc);
    int type;
    unsigned int count;
    (*MyGetControlType)(type);
    KinovaDevice devices[MAX_KINOVA_DEVICE];
    (*MyGetDevices)(devices, type);
    float x;
    (*MyGetEndEffectorOffset)(count, x, x, x);
    EthernetConfiguration *ec;
    (*MyGetEthernetConfiguration)(ec);
    ForcesInfo fResponse;
    (*MyGetForcesInfo)(fResponse);
    GeneralInformations gi;
    (*MyGetGeneralInformations)(gi);
    TrajectoryFIFO tfifo;
    (*MyGetGlobalTrajectoryInfo)(tfifo);
    Gripper g;
    (*MyGetGripperStatus)(g);
    std::vector<float> data;
    (*MyGetPositionCurrentActuators)(data);
    ZoneList zl;
    (*MyGetProtectionZone)(zl);
    QuickStatus qs;
    (*MyGetQuickStatus)(qs);
    SensorsInfo si;
    (*MyGetSensorsInfo)(si);
    SingularityVector sv;
    (*MyGetSingularityVector)(sv);
    float sfc[SPASM_FILTER_COUNT];
    (*MyGetSpasmFilterValues)(sfc, state);
    SystemError se;
    (*MyGetSystemError)(0, se);
    (*MyGetSystemErrorCount)(count);
    (*MyGetTrajectoryTorqueMode)(state);
    (*MyInitAPI)();
    (*MyInitFingers)();
    (*MyMoveHome)(state);
    (*MyProgramFlash)("hello");
    (*MyRefresDevicesList)();
    (*MyRestoreFactoryDefault)();
    ROBOT_TYPE rt;
    double OptimalzParam[OPTIMAL_Z_PARAM_SIZE];
    (*MyRunGravityZEstimationSequence)(rt, OptimalzParam);
    (*MySendAdvanceTrajectory)(trajectory);
    (*MySendAngularTorqueCommand)(Command);
    (*MySendBasicTrajectory)(trajectory);
    (*MySendCartesianForceCommand)(Command);
    JoystickCommand jc;
    (*MySendJoystickCommand)(jc);
    KinovaDevice device;
    (*MySetActiveDevice)(device);
    (*MySetActuatorPID)(0, x, x, x);
    (*MySetActuatorPIDFilter)(0, x, x, x);
    (*MySetAngularControl)();
    (*MySetAngularInertiaDamping)(ai, ai);
    (*MySetAngularTorqueMinMax)(ai, ai);
    (*MySetCartesianControl)();
    (*MySetCartesianForceMinMax)(ci, ci);
    (*MySetCartesianInertiaDamping)(ci, ci);
    (*MySetClientConfigurations)(cc);
    (*MySetControlMapping)(cmc);
    (*MySetEndEffectorOffset)(0, x, x, x);
    (*MySetEthernetConfiguration)(ec);
    (*MySetFrameType)(0);
    float gps[GRAVITY_PARAM_SIZE];
    (*MySetGravityManualInputParam)(gps);
    (*MySetGravityOptimalZParam)(gps);
    float gpsf[GRAVITY_PAYLOAD_SIZE];
    (*MySetGravityPayload)(gpsf);
    GRAVITY_TYPE gt;
    (*MySetGravityType)(gt);
    float gvs[GRAVITY_VECTOR_SIZE];
    (*MySetGravityVector)(gvs);
    (*MySetJointZero)(0);
    unsigned char mac[MAC_ADDRESS_LENGTH];
    char charC[STRING_LENGTH];
    (*MySetLocalMACAddress)(mac, charC);
    (*MySetSerialNumber)(charC, charC);
    (*MySetModel)(charC, charC);
    (*MySetPositionLimitDistance)(Command);
    (*MySetProtectionZone)(zl);
    float sfcf[SPASM_FILTER_COUNT];
    (*MySetSpasmFilterValues)(sfcf, 0);
    (*MySetSwitchThreshold)(Command);
    (*MySetTorqueActuatorDamping)(Command);
    (*MySetTorqueActuatorGain)(Command);
    (*MySetTorqueBrake)(Command);
    (*MySetTorqueCommandMax)(Command);
    TORQUECONTROL_TYPE tct;
    (*MySetTorqueControlType)(tct);
    (*MySetTorqueDampingMax)(Command);
    (*MySetTorqueErrorDeadband)(Command);
    (*MySetTorqueErrorResend)(Command);
    (*MySetTorqueFeedCurrent)(Command);
    (*MySetTorqueFeedCurrentVoltage)(Command);
    (*MySetTorqueFeedFilter)(Command);
    (*MySetTorqueFeedVelocity)(Command);
    (*MySetTorqueFeedVelocityUnderGain)(Command);
    (*MySetTorqueFilterControlEffort)(Command);
    (*MySetTorqueFilterError)(Command);
    (*MySetTorqueFilterMeasuredTorque)(Command);
    (*MySetTorqueFilterVelocity)(Command);
    (*MySetTorqueGainMax)(Command);
    (*MySetTorqueInactivityTimeActuator)(Command);
    (*MySetTorqueInactivityTimeMainController)(0);
    (*MySetTorqueInactivityType)(0);
    (*MySetTorquePositionLimitDampingGain)(Command);
    (*MySetTorquePositionLimitDampingMax)(Command);
    (*MySetTorquePositionLimitRepulsGain)(Command);
    (*MySetTorquePositionLimitRepulsMax)(Command);
    (*MySetTorqueRateLimiter)(Command);
    (*MySetTorqueRobotProtection)(0);
    (*MySetTorqueSafetyFactor)(x);
    (*MySetTorqueStaticFriction)(Command);
    (*MySetTorqueStaticFrictionMax)(Command);
    (*MySetTorqueVelocityLimitFilter)(Command);
    (*MySetTorqueVibrationController)(x);
    (*MySetTorqueZero)(0);
    (*MyStartControlAPI)();
    (*MyStartCurrentLimitation)();
    (*MyStartForceControl)();
    (*MyStartRedundantJointNullSpaceMotion)();
    (*MyStopControlAPI)();
    (*MyStopCurrentLimitation)();
    (*MyStopForceControl)();
    (*MyStopRedundantJointNullSpaceMotion)();
    GENERALCONTROL_TYPE gct;
    (*MySwitchTrajectoryTorque)(gct);
}
