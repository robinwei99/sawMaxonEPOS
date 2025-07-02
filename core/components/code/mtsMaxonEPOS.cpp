/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
  Author(s): Peter Kazanzides, Dimitri Lezcano, Anton Deguet
  (C) Copyright 2024-2025 Johns Hopkins University (JHU)
--- begin cisst license - do not edit ---
This software is provided "as is" under an open source license, with no warranty.
--- end cisst license ---
*/

#include "Definitions.h"  // EPOS Command Library
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnAssert.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawMaxonEPOS/mtsMaxonEPOS.h>

enum OP_STATES { ST_PPM, ST_PVM, ST_PM, ST_VM, ST_CM, ST_HM, ST_MEM, ST_SDM, ST_IPM };

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsMaxonEPOS, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

mtsMaxonEPOS::mtsMaxonEPOS(const std::string &name) :
    mtsTaskContinuous(name, 1024, true)
{}

mtsMaxonEPOS::mtsMaxonEPOS(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread)
{}

mtsMaxonEPOS::mtsMaxonEPOS(const mtsTaskContinuousConstructorArg & arg) :
    mtsTaskContinuous(arg)
{}

mtsMaxonEPOS::~mtsMaxonEPOS()
{
    Close();
}


void mtsMaxonEPOS::SetupInterfaces(void)
{
    StateTable.AddData(mRobot.m_measured_js, "measured_js");
    StateTable.AddData(mRobot.m_setpoint_js, "setpoint_js");
    StateTable.AddData(mRobot.mActuatorState, "actuator_state");
    StateTable.AddData(mRobot.mErrorCode, "error_code");
    mRobot.m_op_state.SetValid(true);
    StateTable.AddData(mRobot.m_op_state, "op_state");
    
    mtsInterfaceProvided *prov = AddInterfaceProvided(mRobot.name);
    mRobot.mInterface = prov;
    if (prov) {
        prov->AddMessageEvents();
        prov->AddCommandReadState(this->StateTable, mRobot.m_measured_js, "measured_js");
        prov->AddCommandReadState(this->StateTable, mRobot.m_setpoint_js, "setpoint_js");
        prov->AddCommandReadState(this->StateTable, mRobot.m_op_state, "operating_state");

        prov->AddCommandWrite(&mtsMaxonEPOS::RobotData::servo_jp, &mRobot, "servo_jp");
        prov->AddCommandWrite(&mtsMaxonEPOS::RobotData::move_jp,  &mRobot, "move_jp");
        prov->AddCommandWrite(&mtsMaxonEPOS::RobotData::servo_jv, &mRobot, "servo_jv");

        prov->AddCommandWrite(&mtsMaxonEPOS::RobotData::state_command, &mRobot, "state_command", std::string(""));

        // prov->AddCommandVoid(&mtsMaxonEPOS::RobotData::EnableMotorPower,  &mRobot, "EnableMotorPower");
        // prov->AddCommandVoid(&mtsMaxonEPOS::RobotData::DisableMotorPower, &mRobot, "DisableMotorPower");
        // prov->AddCommandVoid(&mtsMaxonEPOS::RobotData::hold,     &mRobot, "hold");

        // prov->AddCommandReadState(this->StateTable, mRobot.mSpeed, "GetSpeed");
        // prov->AddCommandReadState(this->StateTable, mRobot.mAccel, "GetAccel");
        // prov->AddCommandReadState(this->StateTable, mRobot.mDecel, "GetDecel");
    }
}

void mtsMaxonEPOS::Configure(const std::string& fileName)
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

    mRobot.name = jsonConfig["name"].asString();
    mRobot.deviceName = jsonConfig["device_name"].asString();
    mRobot.protocolStackName = jsonConfig["protocol_stack_name"].asString();
    mRobot.interfaceName = jsonConfig["interface_name"].asString();
    mRobot.portName = jsonConfig["port_name"].asString();
    mRobot.mTimeout = jsonConfig["timeout"].asUInt();

    mRobot.mParent = this;
    // Size of array determines number of axes
    size_t numAxes = jsonConfig["axes"].size();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: robot " << mRobot.name
                                << " has " << numAxes << " axes" << std::endl;
    mRobot.mNumAxes = static_cast<unsigned int>(numAxes);

    // We have position and velocity for measured_js
    mRobot.m_measured_js.Name().resize(numAxes);
    mRobot.m_measured_js.Position().SetSize(numAxes);
    mRobot.m_measured_js.Velocity().SetSize(numAxes);
    mRobot.m_measured_js.Position().SetAll(0.0);
    mRobot.m_measured_js.Velocity().SetAll(0.0);
    // We have position and effort for setpoint_js
    mRobot.m_setpoint_js.Name().resize(numAxes);
    mRobot.m_setpoint_js.Position().SetSize(numAxes);
    mRobot.m_setpoint_js.Effort().SetSize(numAxes);
    mRobot.m_setpoint_js.Position().SetAll(0.0);
    mRobot.m_setpoint_js.Effort().SetAll(0.0);

    mRobot.mActuatorState.SetSize(static_cast<prmActuatorState::size_type>(numAxes));
    mRobot.mActuatorState.Position().SetAll(0.0);
    mRobot.mActuatorState.Velocity().SetAll(0.0);

    mRobot.mAxisToNodeIDMap.SetSize(numAxes);

    mRobot.mState.SetSize(numAxes);
    mRobot.mState.SetAll(ST_PPM);

    mRobot.mHandles.resize(numAxes);
    for (unsigned int axis = 0; axis < numAxes; axis++){
        mRobot.mAxisToNodeIDMap[axis] = jsonConfig["axes"][axis]["nodeid"].asInt();
        // mRobot.mCalibrationFile.push_back(jsonConfig["axes"][axis]["calibration_file"].asString(););
    }

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    // These sizes should be set before calling StateTable.AddData and AddCommandReadState;
    // in the latter case, this ensures that the argument prototype has the correct size.
    SetupInterfaces();
}


void mtsMaxonEPOS::Startup()//const std::string & fileName
{

    // Zero Error Code
    mRobot.mErrorCode = 0;
    
    mRobot.mHandles[0] = VCS_OpenDevice(const_cast<char*>(mRobot.deviceName.c_str()),
                                const_cast<char*>(mRobot.protocolStackName.c_str()),
                                const_cast<char*>(mRobot.interfaceName.c_str()),
                                const_cast<char*>(mRobot.portName.c_str()),
                                &mRobot.mErrorCode);
    if (mRobot.mHandles[0] == nullptr || mRobot.mErrorCode != 0){
        for (unsigned int j = 1; j < mRobot.mNumAxes; j++){
            mRobot.mHandles[j] = VCS_OpenSubDevice(mRobot.mHandles[0],
                                const_cast<char*>(mRobot.deviceName.c_str()),
                                const_cast<char*>("CANopen"),
                                &mRobot.mErrorCode);
            if(mRobot.mHandles[j]==0){
                CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_OpenSubDevice " << j << " failed (errorCode = " << mRobot.mErrorCode << ")" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    else{
        CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_OpenDevice failed (errorCode = " << mRobot.mErrorCode << ")" << std::endl;
        exit(EXIT_FAILURE);
    }

    unsigned int oldTimeout;
    if (!VCS_GetProtocolStackSettings(mRobot.mHandles[0], &mRobot.baudrate, &oldTimeout, &mRobot.mErrorCode)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_GetProtocolStackSettings failed (errorCode = "
                                << mRobot.mErrorCode << ")\n";
        exit(EXIT_FAILURE);
    }
    if (!VCS_SetProtocolStackSettings(mRobot.mHandles[0], mRobot.baudrate, mRobot.mTimeout, &mRobot.mErrorCode)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: VCS_SetProtocolStackSettings failed (errorCode = "
                                << mRobot.mErrorCode << ")\n";
        exit(EXIT_FAILURE);
    }

    SetupInterfaces();
}

void mtsMaxonEPOS::Run()
{
    int isFault = false;
    // First axis USB, rest of the axis are CAN
    for (size_t axis = 0; axis < mRobot.mNumAxes; ++axis) {
        // Zero errorCode
        mRobot.mErrorCode = 0;

        if (VCS_GetFaultState(mRobot.mHandles[axis],  mRobot.mAxisToNodeIDMap[axis], &isFault,  &mRobot.mErrorCode)) {
            if(isFault){
                mRobot.mActuatorState.MotorOff()[axis] = false;
                mRobot.mActuatorState.InMotion()[axis] = false;
                mRobot.mInterface->SendError(mRobot.name + ": Axis: " + std::to_string(axis) + " is in fault state");
                break;
            }
        } else {
            mRobot.mInterface->SendError(mRobot.name + ": GetFaultState failed (err=" + std::to_string(mRobot.mErrorCode) + ")");
            break;
        };
        
        // Read position
        int positionCounts = 0;
        if (VCS_GetPositionIs(mRobot.mHandles[axis], mRobot.mAxisToNodeIDMap[axis], &positionCounts, &mRobot.mErrorCode)) {
            mRobot.m_measured_js.Position()[axis] = static_cast<double>(positionCounts);
            mRobot.mActuatorState.Position()[axis] = static_cast<double>(positionCounts);
        } else {
            mRobot.mInterface->SendError(mRobot.name + ": GetPositionIs failed (err=" + std::to_string(mRobot.mErrorCode) + ")");
            break;
        }

        // Read Velocity
        int velocityCounts = 0;
        if (VCS_GetVelocityIs(mRobot.mHandles[axis], mRobot.mAxisToNodeIDMap[axis], &velocityCounts, &mRobot.mErrorCode)) {
            mRobot.m_measured_js.Velocity()[axis] = static_cast<double>(velocityCounts);
            mRobot.mActuatorState.Velocity()[axis] = static_cast<double>(velocityCounts);
        } else {
            mRobot.mInterface->SendError(mRobot.name + ": GetVelocityIs failed (err=" + std::to_string(mRobot.mErrorCode) + ")");
            break;
        }

        // Check state
        VCS_GetEnableState(mRobot.mHandles[axis], mRobot.mAxisToNodeIDMap[axis], &mRobot.mMotorPowerOn, &mRobot.mErrorCode);
        mRobot.mActuatorState.MotorOff()[axis] = !mRobot.mMotorPowerOn;

        // Update movement status
        mRobot.mMotionActive = (velocityCounts != 0);
        mRobot.mActuatorState.InMotion()[axis] = mRobot.mMotionActive;
    }

    if(isFault){
        mRobot.newState = prmOperatingState::FAULT;
    }else if(mRobot.mActuatorState.MotorOff().Any()==true){
        mRobot.newState = prmOperatingState::DISABLED;
    }else{
        mRobot.newState = prmOperatingState::ENABLED;
    }
    if (mRobot.newState != mRobot.m_op_state.State()) {
        mRobot.m_op_state.SetState(mRobot.newState);
        // Trigger event
        // operating_state(m_op_state);
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
void mtsMaxonEPOS::Close()
{
    // 1) Close sub device first
    for (size_t axis = 1; axis < mRobot.mHandles.size(); ++axis) {
        if (mRobot.mHandles[axis]) {
            if (!VCS_CloseSubDevice(mRobot.mHandles[axis], &mRobot.mErrorCode) || mRobot.mErrorCode != 0) {
                CMN_LOG_CLASS_RUN_ERROR 
                    << mRobot.name 
                    << "[axis " << axis 
                    << "] CloseSubDevice failed (errorCode=" << mRobot.mErrorCode << ")\n";
            }
            mRobot.mHandles[axis] = nullptr;
        }
    }
    if (!VCS_CloseDevice(mRobot.mHandles[0], &mRobot.mErrorCode) || mRobot.mErrorCode != 0) {
        CMN_LOG_CLASS_RUN_ERROR 
            << mRobot.name 
            << "[gateway] CloseDevice failed (errorCode=" << mRobot.mErrorCode << ")\n";
    }
    mRobot.mHandles[0] = nullptr;
}

void mtsMaxonEPOS::RobotData::state_command(const std::string &command)
{
    std::string humanReadableMessage;
    prmOperatingState::StateType newOperatingState;
    try {
        if (m_op_state.ValidCommand(prmOperatingState::CommandTypeFromString(command),
                                    newOperatingState, humanReadableMessage)) {
            if (command == "enable") {
                EnableMotorPower();
                return;
            }
            if (command == "disable") {
                DisableMotorPower();
                return;
            }
            if (command == "home") {
                // vctBoolVec homingMask(mNumAxes, true);
                // Home(homingMask);
                return;
            }
            if (command == "unhome") {
                // vctBoolVec homingMask(mNumAxes, true);
                // UnHome(homingMask);
                return;
            }
            if (command == "pause") {
                if (m_op_state.State() == prmOperatingState::ENABLED)
                    hold();   // Stop motion
                m_op_state.SetState(newOperatingState);
                return;
            }
            if (command == "resume") {
                m_op_state.SetState(newOperatingState);
                return;
            }
        } else {
            mInterface->SendWarning(name + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error &e) {
        mInterface->SendWarning(name + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}

// Fixed
void mtsMaxonEPOS::RobotData::EnableMotorPower(void)
{
    if (!mParent) {return;}

    mErrorCode = 0;
    try {

        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            int isFault = false; 

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
            if (!VCS_GetEnableState(mHandles[axis], mAxisToNodeIDMap[axis], &mMotorPowerOn, &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " GetEnableState failed (err=" + std::to_string(mErrorCode) + ")"
                );
            }
            if (!mMotorPowerOn) {
                if (!VCS_SetEnableState(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " SetEnableState failed (err=" + std::to_string(mErrorCode) + ")"
                    );
                }
            }
        }

    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": EnableMotorPower (" + e.what() + ")");
    }
}

// Fixed
void mtsMaxonEPOS::RobotData::DisableMotorPower(void)
{
    if (!mParent) {return;}

    mErrorCode = 0;
    try {
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            // Find enable state
            if (!VCS_GetEnableState(mHandles[axis], mAxisToNodeIDMap[axis], &mMotorPowerOn, &mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " GetEnableState failed (err=" + std::to_string(mErrorCode) + ")"
                );
            }
            // Disable only enable state
            if (mMotorPowerOn) {
                if (!VCS_SetDisableState(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    throw std::runtime_error(
                        "Axis " + std::to_string(axis) +
                        " SetDisableState failed (err=" + std::to_string(mErrorCode) + ")"
                    );
                }
            }
        }

        // mActuatorState.MotorOff().SetAll(true);
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": DisableMotorPower (" + e.what() + ")");
    }
}

// VM
void mtsMaxonEPOS::RobotData::servo_jv(const prmVelocityJointSet & jtvel)
{
    if (!mParent) {return;}

    if (!CheckStateEnabled("servo_jv"))
        return;

    mErrorCode = 0;
    try {
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            // 2.1) Active Velocity Mode.
            if (mState[axis] != ST_VM) {
                // Check if hold(); is needed here.
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

            m_setpoint_js.Position()[axis] = 0.0;
        }

    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": servo_jv (" + e.what() + ")");
    }
}

// PM
void mtsMaxonEPOS::RobotData::servo_jp(const prmPositionJointSet & jtpos)
{
    if (!mParent) {return;}

    if (!CheckStateEnabled("servo_jp"))
        return;

    mErrorCode = 0;

    try {
        // Iterate through each axis，Position mode direct control.
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            // 2.1 Position Mode（CSP）
            if(mState[axis] != ST_PM){
                // Check if hold(); is needed here.
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

            m_setpoint_js.Position()[axis] = jtpos.Goal()[axis];
        }

    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": servo_jp (" + e.what() + ")");
    }
}

// PPM
void mtsMaxonEPOS::RobotData::move_jp(const prmPositionJointSet & jtpos)
{
    if (!mParent) {return;}

    if (!CheckStateEnabled("move_jp"))
        return;

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

            m_setpoint_js.Position()[axis] = jtpos.Goal()[axis];
        }
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": move_jp (" + e.what() + ")");
    }
}

// Fixed
void mtsMaxonEPOS::RobotData::hold(void)
{
    if (!mParent) {return;}

    if (!CheckStateEnabled("hold"))
        return;

    // Return if all stop
    if (!mActuatorState.InMotion().Any()) {
        return;
    }

    // Iterate through each axis，Stop motion
    for (size_t axis = 0; axis < mNumAxes; ++axis) {
        if (mActuatorState.MotorOff()[axis]){continue;}

        switch (mState[axis]) {
            case ST_PVM:
                // Velocity Profile mode
                if (!VCS_HaltVelocityMovement(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    mInterface->SendWarning(name + ": " +
                        " axis " + std::to_string(axis) +
                        " HaltVelocityMovement failed (err=" + std::to_string(mErrorCode) + ")");
                }
                break;

            case ST_PPM:
                // Position Profile Mode
                if (!VCS_HaltPositionMovement(mHandles[axis], mAxisToNodeIDMap[axis], &mErrorCode)) {
                    mInterface->SendWarning(name + ": " +
                        " axis " + std::to_string(axis) +
                        " HaltPositionMovement(default) failed (err=" + std::to_string(mErrorCode) + ")");
                }
                break;

            case ST_VM:
                // Velocity mode
                if (!VCS_SetVelocityMust(mHandles[axis], mAxisToNodeIDMap[axis], 0, &mErrorCode)) {
                    mInterface->SendWarning(name + ": " +
                        " axis " + std::to_string(axis) +
                        " HaltVelicty(default) failed (err=" + std::to_string(mErrorCode) + ")");
                }
                break;
        }
    }
    // Clear moving flag.
}

void mtsMaxonEPOS::RobotData::SetPositionProfile(
    const vctDoubleVec & profileVelocity,
    const vctDoubleVec & profileAcceleration,
    const vctDoubleVec & profileDeceleration)
{
    if (!mParent) {return;}

    mErrorCode = 0;
    try {
        for (size_t axis = 0; axis < mNumAxes; ++axis) {
            if (mActuatorState.MotorOff()[axis]) {
                continue;
            }

            if (!VCS_SetPositionProfile(mHandles[axis], mAxisToNodeIDMap[axis],profileVelocity[axis],profileAcceleration[axis],profileDeceleration[axis],&mErrorCode)) {
                throw std::runtime_error(
                    "Axis " + std::to_string(axis) +
                    " SetPositionProfile failed (err=" +
                    std::to_string(mErrorCode) + ")"
                );
            }
        }
    }
    catch (const std::runtime_error & e) {
        mInterface->SendError(name + ": SetPositionProfile (" + e.what() + ")");
    }
}

bool mtsMaxonEPOS::RobotData::CheckStateEnabled(const char *cmdName) const
{
    if (m_op_state.State() != prmOperatingState::ENABLED) {
        try {
            mInterface->SendWarning(name + ": " + cmdName + ": robot not enabled, current state is "
                                    + prmOperatingState::StateTypeToString(m_op_state.State()));
        }
        catch (const std::runtime_error &e) {
            mInterface->SendError(name + ": " + cmdName + ": robot not enabled, " + e.what());
        }
        return false;
    }
    return true;
}