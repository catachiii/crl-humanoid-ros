//
// Created by Dongho Kang on 22.01.22.
//

#include "crl_humanoid_commons/RobotData.h"

namespace crl::humanoid::commons {

    RobotData::RobotData() {}

    RobotData::RobotData(const std::vector<std::string>& jointNames,
                                                   const std::vector<double>& defaultJointConf) {
        PropagateJointInformation(jointNames, defaultJointConf);
    }

    RobotData::~RobotData() {}

    void RobotData::PropagateJointInformation(
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

    ProfilingInfo RobotData::getProfilingInfo() {
        std::lock_guard<std::mutex> guard(profilingMutex);
        ProfilingInfo copy = profilingInfo;
        return copy;
    }

    void RobotData::setProfilingInfo(const ProfilingInfo& profilingInfo) {
        std::lock_guard<std::mutex> guard(profilingMutex);
        this->profilingInfo = profilingInfo;
    }

    RobotCommand RobotData::getCommand() {
        std::lock_guard<std::mutex> guard(commandMutex);
        RobotCommand copy = command;
        return copy;
    }

    void RobotData::setCommand(const RobotCommand& command) {
        std::lock_guard<std::mutex> guard(commandMutex);
        this->command = command;
    }

    RobotSensor RobotData::getSensor() {
        std::lock_guard<std::mutex> guard(sensorMutex);
        RobotSensor copy = sensor;
        return copy;
    }

    void RobotData::setSensor(const RobotSensor& sensor) {
        std::lock_guard<std::mutex> guard(sensorMutex);
        this->sensor = sensor;
    }

    RobotState RobotData::getRobotState() {
        std::lock_guard<std::mutex> guard(stateMutex);
        RobotState copy = state;
        return copy;
    }

    void RobotData::setRobotState(const RobotState& state) {
        std::lock_guard<std::mutex> guard(stateMutex);
        this->state = state;
    }


    RobotControlSignal RobotData::getControlSignal() {
        std::lock_guard<std::mutex> guard(controlMutex);
        RobotControlSignal copy = control;
        return copy;
    }

    void RobotData::setControlSignal(const RobotControlSignal& control) {
        std::lock_guard<std::mutex> guard(controlMutex);
        this->control = control;
    }

    double RobotData::getTimeStamp() const {
        return timeStamp;
    }

    void RobotData::advanceInTime(double dt) {
        timeStamp = timeStamp + dt;
    }

}  // namespace crl::humanoid::commons
