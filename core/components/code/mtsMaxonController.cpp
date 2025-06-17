/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
  Author(s): Peter Kazanzides, Dimitri Lezcano, Anton Deguet
  (C) Copyright 2024-2025 Johns Hopkins University (JHU)
--- begin cisst license - do not edit ---
This software is provided "as is" under an open source license, with no warranty.
--- end cisst license ---
*/

#include "EposCmd.h"  // EPOS Command Library
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnAssert.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawMaxonController/mtsMaxonController.h>

// 如果需要 Homing，则引入 EPOS Homing 接口
// extern "C" {
//    BOOL VCS_ActivateHomingMode(HANDLE, WORD, DWORD*);
//    BOOL VCS_FindHomePosition(HANDLE, WORD, DWORD*);
// }

enum OP_STATES { ST_PPM, ST_PVM, ST_PM, ST_VM, ST_CM, ST_HM, ST_MEM, ST_SDM, ST_IPM };

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsMaxonController, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

mtsMaxonController::mtsMaxonController(const std::string &name) :
    mtsTaskContinuous(name, 1024, true),
{}

mtsMaxonController::mtsMaxonController(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, 1024, true),
{}

mtsMaxonController::~mtsMaxonController()
{
    Close();
}

void mtsMaxonController::SetupInterfaces(void)
{
    for (size_t i = 0; i < mRobots.size(); i++) {
        StateTable.AddData(mRobots[i].m_measured_js, "measured_js");
        StateTable.AddData(mRobots[i].m_setpoint_js, "setpoint_js");
        StateTable.AddData(mRobots[i].mActuatorState, "actuator_state");
        StateTable.AddData(mRobots[i].mErrorCode, "error_code");
        
        mtsInterfaceProvided *prov = AddInterfaceProvided(mRobots[i].name);
        mRobots[i].mInterface = prov;
        if (prov) {
            prov->AddMessageEvents();
            prov->AddCommandReadState(this->StateTable, mRobots[i].m_measured_js, "measured_js");
            prov->AddCommandReadState(this->StateTable, mRobots[i].m_setpoint_js, "setpoint_js");
            // prov->AddCommandReadState(this->StateTable, mRobots[i].m_op_state, "operating_state");

            prov->AddCommandWrite(&mtsMaxonController::RobotData::servo_jp, &mRobots[i], "servo_jp");
            prov->AddCommandWrite(&mtsMaxonController::RobotData::move_jp,  &mRobots[i], "move_jp");

            prov->AddCommandWrite(&mtsMaxonController::RobotData::servo_jv, &mRobots[i], "servo_jv");
            prov->AddCommandWrite(&mtsMaxonController::RobotData::hold,     &mRobots[i], "hold");

            prov->AddCommandWrite(&mtsMaxonController::RobotData::EnableMotorPower,  &mRobots[i], "EnableMotorPower");
            prov->AddCommandWrite(&mtsMaxonController::RobotData::DisableMotorPower, &mRobots[i], "DisableMotorPower");


            prov->AddCommandReadState(this->StateTable, mRobots[i].mSpeed, "GetSpeed");
            prov->AddCommandReadState(this->StateTable, mRobots[i].mAccel, "GetAccel");
            prov->AddCommandReadState(this->StateTable, mRobots[i].mDecel, "GetDecel");
        }
    }
}

void mtsGalilController::Configure(const std::string& fileName)
{
    mConfigPath.Set(cmnPath::GetWorkingDirectory());
    std::string fullname = mConfigPath.Find(fileName);
    // Handle either forward slash or backslash for directory separator,
    // since on Windows there can be a mix of them.
    size_t last_sep = fullname.find_last_of('/');
    size_t last_sep2 = fullname.find_last_of('\\');
    if (last_sep == std::string::npos)
        last_sep = last_sep2;
    else if ((last_sep2 != std::string::npos) && (last_sep2 > last_sep))
        last_sep = last_sep2;
    if (last_sep != std::string::npos) {
        std::string configDir = fullname.substr(0, last_sep);
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting mConfigPath to " << configDir
                                   << " for file " << fileName << std::endl;
        mConfigPath.Add(configDir, cmnPath::HEAD);
    }

    std::ifstream jsonStream;
    jsonStream.open(fileName.c_str());
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse " << fileName << " for Galil config" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }

    if (jsonConfig["robots"].size() < 1) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: no robots or analog inputs specified!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Process the robot configuration data
    mRobots.resize(jsonConfig["robots"].size());
    for (unsigned int i = 0; i < jsonConfig["robots"].size(); i++) {
        mRobots[i].name = jsonConfig["robots"][i][name].asString();
        mRobots[i].deviceName = jsonConfig["robots"][i][device_name].asString();
        mRobots[i].protocolStackName = jsonConfig["robots"][i][protocol_stack_name].asString();
        mRobots[i].interfaceName = jsonConfig["robots"][i][interface_name].asString();
        mRobots[i].portName = jsonConfig["robots"][i][port_name].asString();
        mRobots[i].mTimeout = jsonConfig["robots"][i][timeout].asUInt();

        mRobots[i].mParent = this;
        // Size of array determines number of axes
        size_t numAxes = jsonConfig["robots"][i]["axes"].size();
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: robot " << mRobots[i].name
                                   << " has " << numAxes << " axes" << std::endl;
        mRobots[i].mNumAxes = static_cast<unsigned int>(numAxes);

        // We have position and velocity for measured_js
        mRobots[i].m_measured_js.Name().resize(numAxes);
        mRobots[i].m_measured_js.Position().SetSize(numAxes);
        mRobots[i].m_measured_js.Velocity().SetSize(numAxes);
        mRobots[i].m_measured_js.Position().SetAll(0.0);
        mRobots[i].m_measured_js.Velocity().SetAll(0.0);
        // We have position and effort for setpoint_js
        mRobots[i].m_setpoint_js.Name().resize(numAxes);
        mRobots[i].m_setpoint_js.Position().SetSize(numAxes);
        mRobots[i].m_setpoint_js.Effort().SetSize(numAxes);
        mRobots[i].m_setpoint_js.Position().SetAll(0.);
        mRobots[i].m_setpoint_js.Effort().SetAll(0.0);

        mRobots[i].mActuatorState.SetSize(static_cast<prmActuatorState::size_type>(numAxes));
        mRobots[i].mActuatorState.Position().SetAll(0.0);
        mRobots[i].mActuatorState.Velocity().SetAll(0.0);

        mRobots[i].mAxisToNodeIDMap.SetSize(numAxes);

        mRobots[i].mState.SetSize(numAxes);
        mRobots[i].mState.SetAll(ST_PPM);

        mHandles.resize(numAxes);
        for (unsigned int axis = 0; axis < numAxes; axis++){
            mRobots[i].mAxisToNodeIDMap[axis] = jsonConfig["robots"][i]["axes"][axis]["nodeid"].asInt();
            // mRobots[i].mCalibrationFile.push_back(jsonConfig["robots"][i]["axes"][axis]["calibration_file"].asString(););
        }
    }

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    // These sizes should be set before calling StateTable.AddData and AddCommandReadState;
    // in the latter case, this ensures that the argument prototype has the correct size.
    SetupInterfaces();
}


void mtsMaxonController::Startup()//const std::string & fileName
{

    for (unsigned int i = 0; i < mRobots.size(); i++) {
        // Zero Error Code
        mRobots[i].mErrorCode = 0;
        
        mRobots[i].mHandle[0] = VCS_OpenDevice(const_cast<char*>(mRobots[i].deviceName.c_str()),
                                 const_cast<char*>(mRobots[i].protocolStackName.c_str()),
                                 const_cast<char*>(mRobots[i].interfaceName.c_str()),
                                 const_cast<char*>(mRobots[i].portName.c_str()),
                                 &mRobots[i].mErrorCode);
        if (mRobots[i].mHandle[0] == nullptr || mRobots[i].mErrorCode != 0){
            for (unsigned int j = 1; j < mRobots[i].mNumAxes; j++){
                mRobots[i].mHandle[j] = VCS_OpenSubDevice(mRobots[i].mHandle[0],
                                 const_cast<char*>(mRobots[i].deviceName.c_str()),
                                 const_cast<char*>("CANopen"),
                                 &mRobots[i].mErrorCode)
                if(mRobots[i].mHandle[j]==0){
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_OpenSubDevice " << j << " failed (errorCode = " << mRobots[i].mErrorCode << ")" << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
        else(
            CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_OpenDevice failed (errorCode = " << mRobots[i].mErrorCode << ")" << std::endl;
            exit(EXIT_FAILURE);
        )
        
        unsigned int* oldTImeout;
        if (!VCS_GetProtocolStackSettings(mRobots[i].mHandle[0], &mRobots[i].baudrate, oldTImeout, &mRobots[i].mErrorCode)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_GetProtocolStackSettings failed (errorCode = "
                                    << mRobots[i].mErrorCode << ")\n";
            exit(EXIT_FAILURE);
        }
        if (!VCS_SetProtocolStackSettings(mRobots[i].mHandle[0], mRobots[i].baudrate, mRobots[i].mTimeout, &mRobots[i].mErrorCode)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_SetProtocolStackSettings failed (errorCode = "
                                    << mRobots[i].mErrorCode << ")\n";
            exit(EXIT_FAILURE);
        }
    }

    SetupInterfaces();
}

void mtsMaxonController::Run()
{
    for (size_t i = 0; i < mRobots.size(); ++i) {
        // 假设第一个轴 handle 是 USB 直连，后面所有都是通过 CANopen 网关
        for (size_t axis = 0; axis < mRobots[i].mNumAxes; ++axis) {
            // Zero errorCode
            mRobots[i].mErrorCode = 0;

            bool isFault = false;
            if (VCS_GetFaultState(mRobots[i].mHandles[axis],  mRobots[i].mAxisToNodeIDMap[axis], &isFault,  &mRobots[i].mErrorCode)) {
                if(isFault){
                    mRobots[i].mActuatorState.MotorOff()[axis] = false;
                    mRobots[i].mActuatorState.InMotion()[axis] = false;
                    mRobots[i].mInterface->SendError(mRobots[i].name + ": Axis: " + std::to_string(axis) + " is in fault state");
                    continue;
                }
            } else {
                mRobots[i].mInterface->SendError(mRobots[i].name + ": GetFaultState failed (err=" + std::to_string(mRobots[i].mErrorCode) + ")");
            };
            
            
            // 1) 读取实际位置（encoder counts）
            long positionCounts = 0;
            if (VCS_GetPositionIs(mRobots[i].mHandles[axis], mRobots[i].mAxisToNodeIDMap[axis], &positionCounts, &mRobots[i].mErrorCode)) {
                mRobots[i].m_measured_js.Position()[axis] = static_cast<double>(positionCounts);
                mRobots[i].mActuatorState.Position()[axis] = static_cast<double>(positionCounts);
            } else {
                mRobots[i].mInterface->SendError(mRobots[i].name + ": GetPositionIs failed (err=" + std::to_string(mRobots[i].mErrorCode) + ")");
            }

            // 2) 读取实际速度（counts/sec）
            long velocityCounts = 0;
            if (VCS_GetVelocityIs(mRobots[i].mHandles[axis], mRobots[i].mAxisToNodeIDMap[axis], &velocityCounts, &mRobots[i].mErrorCode)) {
                mRobots[i].m_measured_js.Velocity()[axis] = static_cast<double>(velocityCounts);
                mRobots[i].mActuatorState.Velocity()[axis] = static_cast<double>(velocityCounts);
            } else {
                mRobots[i].mInterface->SendError(mRobots[i].name + ": GetVelocityIs failed (err=" + std::to_string(mRobots[i].mErrorCode) + ")");
            }

            // 4) 读取使能/故障状态
            VCS_GetEnableState(mRobots[i].mHandles[axis], mRobots[i].mAxisToNodeIDMap[axis], &mRobots[i].mMotorPowerOn, &mRobots[i].mErrorCode);
            mRobots[i].mActuatorState.MotorOff()[axis] = !mRobots[i].mMotorPowerOn;

            // 5) 更新运动状态标志
            mRobots[i].mMotionActive = (velocityCounts != 0);
            mRobots[i].mActuatorState.InMotion()[axis] = mRobots[i].mMotionActive;
        }
    }


    // Advance the state table now, so that any connected components can get
    // the latest data.
    StateTable.Advance();

    // Call any connected components
    RunEvent();

    // Could instead loop through each provided interface, call ProcessMailBoxes,
    // catch exceptions and call the appropriate mInterface->SendError.
    try {
        ProcessQueuedCommands();
    }
    catch (const std::runtime_error &e) {
        CMN_LOG_CLASS_RUN_ERROR << this->GetName() << ": ProcessQueuedCommands " << e.what() << std::endl;
    }
}

// Fixed
void mtsMaxonController::Close()
{
    // 1) 先关闭所有通过 VCS_OpenSubDevice 打开的子设备
    for (size_t i = 0; i < mRobots.size(); ++i) {
        for (size_t axis = 1; axis < mRobots[i].mHandles.size(); ++axis) {
            if (mRobots[i].mHandles[axis]) {
                if (!VCS_CloseSubDevice(mRobots[i].mHandles[axis], &mRobots[i].mErrorCode) || mRobots[i].mErrorCode != 0) {
                    CMN_LOG_CLASS_RUN_ERROR 
                      << mRobots[i].name 
                      << "[axis " << axis 
                      << "] CloseSubDevice failed (errorCode=" << mRobots[i].mErrorCode << ")\n";
                }
                mRobots[i].mHandles[axis] = nullptr;
            }
        }
        if (!VCS_CloseDevice(mRobots[i].mHandles[0], &mRobots[i].mErrorCode) || mRobots[i].mErrorCode != 0) {
            CMN_LOG_CLASS_RUN_ERROR 
              << mRobots[0].name 
              << "[gateway] CloseDevice failed (errorCode=" << mRobots[i].mErrorCode << ")\n";
        }
        mRobots[0].mHandles[0] = nullptr;
    }
}

// Fixed
void mtsMaxonController::RobotData::EnableMotorPower(void)
{
    if (!mParent || !mParent->mEposHandle) {
        return;
    }

    mErrorCode = 0;
    try {

        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            bool   isFault = false, 
            bool isEnabled = false;

            // 2.1) Clear fault
            if (!VCS_GetFaultState(mHandles[axis], mAxisToNodeIDMap[axis], &isFault, &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " GetFaultState failed (err=" + std::to_string(mErrorCode) + ")"
                );
            }
            if (isFault) {
                if (!VCS_ClearFault(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " ClearFault failed (err=" + std::to_string(mErrorCode) + ")"
                    );
                }
            }

            // 2.2) Enable power
            if (!VCS_GetEnableState(mHandles[axis], mAxisToNodeIDMap[axis], &isEnabled, &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " GetEnableState failed (err=" + std::to_string(mErrorCode) + ")"
                );
            }
            if (!isEnabled) {
                if (!VCS_SetEnableState(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " SetEnableState failed (err=" + std::to_string(mErrorCode) + ")"
                    );
                }
            }
        }

        mActuatorState.MotorOff().SetAll(false);
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": EnableMotorPower (" + e.what() + ")");
    }
}

// Fixed
void mtsMaxonController::RobotData::DisableMotorPower(void)
{
    if (!mParent || !mParent->mEposHandle) {
        return;
    }

    mErrorCode = 0;
    try {
        for (size_t axis = 0; axis < mNumAxes; ++axis) {

            // 2.1) 查询当前使能状态
            if (!VCS_GetEnableState(mHandles[axis], mAxisToNodeIDMap[axis], &mMotorPowerOn, &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " GetEnableState failed (err=" + std::to_string(mErrorCode) + ")"
                );
            }
            // 2.2) 若已使能则调用 Disable
            if (mMotorPowerOn) {
                if (!VCS_SetDisableState(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " SetDisableState failed (err=" + std::to_string(mErrorCode) + ")"
                    );
                }
            }
        }

        mActuatorState.MotorOff().SetAll(true);
    }
    catch (const std::runtime_error & e) {
        // 4) 捕获并上报
        mHadError = true;
        mInterface->SendError(name + ": DisableMotorPower (" + e.what() + ")");
    }
}

// VM
void mtsMaxonController::RobotData::servo_jv(const prmVelocityJointSet & jtvel)
{
    if (!mParent || !mParent->mEposHandle) return;

    mErrorCode = 0;

    try {
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            if (mActuatorState.MotorOff()[axis]) {
                continue;
            }

            // 2.1) Active Velocity Mode.
            if (mState[axis] != ST_VM) {
                if (!VCS_ActivateVelocityMode(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " ActivateVelocityMode failed (err=" +
                        std::to_string(mErrorCode) + ")"
                    );
                }
                mState[axis] = ST_VM;
            }

            // 2.2) Velocity set‐point
            if (!VCS_SetVelocityMust(mHandles[axis], mAxisToNodeIDMap[axis], jtvel.Goal()[axis], &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " SetVelocityMust failed (err=" +
                    std::to_string(mErrorCode) + ")"
                );
            }
        }

    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": servo_jv (" + e.what() + ")");
    }
}

// PM
void mtsMaxonController::RobotData::servo_jp(const prmPositionJointSet & jtpos)
{
    if (!mParent || !mParent->mHandles) return;
    mErrorCode = 0;

    try {
        // Iterate through each axis，Position mode direct control.
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            if (mActuatorState.MotorOff()[axis]) {
                continue;
            }

            // 2.1 Position Mode（CSP）
            if(mState[axis] != ST_PM){
                if (!VCS_ActivatePositionMode(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "ActivatePositionMode failed on axis " + std::to_string(axis) +
                        " (err=" + std::to_string(mErrorCode) + ")"
                    );
                }
                mState[axis] = ST_PM;
            }

            // 2.2 Position Must
            if (!VCS_SetPositionMust(mHandles[axis], mAxisToNodeIDMap[axis], jtpos.Goal()[axis], &mErrorCode)) {
                throw std::runtime_error(
                    "SetPositionMust failed on axis " + std::to_string(axis) +
                    " (err=" + std::to_string(mErrorCode) + ")"
                );
            }
        }

    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": servo_jp (" + e.what() + ")");
    }
}

// PPM
void mtsMaxonController::RobotData::move_jp(const prmPositionJointSet & jtpos)
{
    if (!mParent || !mParent->mHandles) return;

    mErrorCode = 0;
    try {
        // Stop moving
        hold();

        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            
            if (mActuatorState.MotorOff()[axis]){continue;}
            
            // 1) Active Profile Position Mode
            if(mState[axis] != ST_PPM){
                if (!VCS_ActivateProfilePositionMode(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " ActivateProfilePositionMode failed (err=" +
                        std::to_string(mErrorCode) + ")"
                    );
                }
                mState[axis] = ST_PPM;
            }
            
            // 2) Send command
            if (!VCS_MoveToPosition(mHandles[axis], mAxisToNodeIDMap[axis],
                                    jtpos.Goal()[axis],
                                    /*Absolute*/  1,
                                    /*Immediate*/ 1,
                                    &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " MoveToPosition failed (err=" +
                    std::to_string(mErrorCode) + ")"
                );
            }
        }
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": move_jp (" + e.what() + ")");
    }
}

// Fixed
void mtsMaxonController::RobotData::hold(void)
{
    // Return if all stop
    if (!mActuatorState.InMotion().Any()) {
        return;
    }

    // Iterate through each axis，Stop motion
    for (size_t axis = 0; axis < mNumAxes; ++axis) {
        if (mActuatorState.MotorOff()[axis]){continue;}

        switch (mState[axis]) {
            case ST_PVM:
                // Velocity mode
                if (!VCS_HaltVelocityMovement(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    mInterface->SendWarning(name + ": " +
                        " axis " + std::to_string(axis) +
                        " HaltVelocityMovement failed (err=" + std::to_string(mErrorCode) + ")");
                }
                break;

            default:
                // All other mode
                if (!VCS_HaltPositionMovement(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    mInterface->SendWarning(name + ": " +
                        " axis " + std::to_string(axis) +
                        " HaltPositionMovement(default) failed (err=" + std::to_string(mErrorCode) + ")");
                }
                break;
        }
    }
    osaSleep(0.05);
    // Clear moving flag.
    mActuatorState.InMotion().SetAll(false);
}

void mtsMaxonController::RobotData::SetSpeed(const vctDoubleVec & spd)
{
    // 1) 检查父对象和总线句柄
    if (!mParent || !mParent->mEposHandle) {
        return;
    }

    mErrorCode = 0;

    try {

        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            if (mActuatorState.MotorOff()[axis]) {
                continue;
            }
            if (!VCS_SetVelocityMust(mHandles[axis], mAxisToNodeIDMap[axis], spd[axis], &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " SetVelocityMust failed (err=" + std::to_string(mErrorCode) + ")"
                );
            }
        }

    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": SetSpeed (" + e.what() + ")");
    }
}

void mtsMaxonController::RobotData::SetAccel(const vctDoubleVec & accel)
{
    // 1) 参数检查
    if (!mParent || !mParent->mEposHandle) return;

    // 2) 准备错误码
    mErrorCode = 0;

    try {
        // 3) 多轴循环，分别设置最大加速度
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            // 跳过已断电的轴
            if (mActuatorState.MotorOff()[axis]) {
                continue;
            }

            if (!VCS_SetMaxAcceleration(mHandles[axis], mAxisToNodeIDMap[axis], accel[axis], &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " SetMaxAcceleration failed (err=" +
                    std::to_string(mErrorCode) + ")"
                );
            }
        }
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": SetAccel (" + e.what() + ")");
    }
}

void mtsMaxonController::RobotData::SetDecel(const vctDoubleVec & decel)
{
    // 1) 参数检查
    if (!mParent || !mParent->mEposHandle) return;

    // 2) 准备错误码
    mErrorCode = 0;

    try {
        // 3) 多轴循环，分别设置最大减速度
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            // 跳过已断电的轴
            if (mActuatorState.MotorOff()[axis]) {
                continue;
            }
            if (!VCS_SetMaxDeceleration(mHandles[axis], mAxisToNodeIDMap[axis], decel[axis], &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " SetMaxDeceleration failed (err=" +
                    std::to_string(mErrorCode) + ")"
                );
            }
        }
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": SetDecel (" + e.what() + ")");
    }
}