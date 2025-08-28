//
// Created by Dongho Kang on 22.01.22.
//

#include "crl_humanoid_commons/RobotData.h"

namespace crl::unitree::commons {

    UnitreeLeggedRobotData::UnitreeLeggedRobotData() {}

    UnitreeLeggedRobotData::UnitreeLeggedRobotData(const std::vector<std::string>& jointNames,
                                                   const std::vector<double>& defaultJointConf) {
        PropagateJointInformation(jointNames, defaultJointConf);
    }

    UnitreeLeggedRobotData::~UnitreeLeggedRobotData() {}

    void UnitreeLeggedRobotData::PropagateJointInformation(
            std::vector<std::string> jointNames,
            std::vector<double> defaultJointConf) {
        // Clear existing data first to prevent duplication
        state.jointStates.clear();
        sensor.jointSensors.clear();
        control.jointControl.clear();

        for (size_t i = 0; i < jointNames.size(); ++i) {
            state.jointStates.push_back({jointNames[i], defaultJointConf[i], 0.0});
            sensor.jointSensors.push_back({jointNames[i], defaultJointConf[i], 0.0, 0.0});
            control.jointControl.push_back({jointNames[i], RBJointControlMode::PASSIVE_MODE, defaultJointConf[i], 0, 0, 0, 0});
        }

    }

    ProfilingInfo UnitreeLeggedRobotData::getProfilingInfo() {
        std::lock_guard<std::mutex> guard(profilingMutex);
        ProfilingInfo copy = profilingInfo;
        return copy;
    }

    void UnitreeLeggedRobotData::setProfilingInfo(const ProfilingInfo& profilingInfo) {
        std::lock_guard<std::mutex> guard(profilingMutex);
        this->profilingInfo = profilingInfo;
    }

    LeggedRobotCommand UnitreeLeggedRobotData::getCommand() {
        std::lock_guard<std::mutex> guard(commandMutex);
        LeggedRobotCommand copy = command;
        return copy;
    }

    void UnitreeLeggedRobotData::setCommand(const LeggedRobotCommand& command) {
        std::lock_guard<std::mutex> guard(commandMutex);
        this->command = command;
    }

    LeggedRobotSensor UnitreeLeggedRobotData::getSensor() {
        std::lock_guard<std::mutex> guard(sensorMutex);
        LeggedRobotSensor copy = sensor;
        return copy;
    }

    void UnitreeLeggedRobotData::setSensor(const LeggedRobotSensor& sensor) {
        std::lock_guard<std::mutex> guard(sensorMutex);
        this->sensor = sensor;
    }

    LeggedRobotState UnitreeLeggedRobotData::getLeggedRobotState() {
        std::lock_guard<std::mutex> guard(stateMutex);
        LeggedRobotState copy = state;
        return copy;
    }

    void UnitreeLeggedRobotData::setLeggedRobotState(const LeggedRobotState& state) {
        std::lock_guard<std::mutex> guard(stateMutex);
        this->state = state;
    }


    LeggedRobotControlSignal UnitreeLeggedRobotData::getControlSignal() {
        std::lock_guard<std::mutex> guard(controlMutex);
        LeggedRobotControlSignal copy = control;
        return copy;
    }

    void UnitreeLeggedRobotData::setControlSignal(const LeggedRobotControlSignal& control) {
        std::lock_guard<std::mutex> guard(controlMutex);
        this->control = control;
    }

    double UnitreeLeggedRobotData::getTimeStamp() const {
        return timeStamp;
    }

    void UnitreeLeggedRobotData::advanceInTime(double dt) {
        timeStamp = timeStamp + dt;
    }

}  // namespace crl::unitree::commons
