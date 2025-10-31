#include "crl_g1_mimiccontroller/CRLG1DeepMimicController.h"
#include <filesystem>
#include <fstream>
#include <string>
#include <cmath>
#include <crl-basic/utils/mathDefs.h>

namespace crl::g1::rlcontroller {

CRLG1DeepMimicController::CRLG1DeepMimicController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                         const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                         const rclcpp::Logger &logger)
    : LocomotionController(model, data, logger) {
    // Initialize action vectors with robot joint count
    int jointCount = static_cast<int>(model->jointNames.size());
    crl::utils::resize(action_, jointCount);
}

bool CRLG1DeepMimicController::loadModelFromFile(const std::string &fileName) {
    std::ifstream file(fileName);
    if (file.fail()) {
        RCLCPP_ERROR(logger_, "Failed to load RL policy configuration file: %s", fileName.c_str());
        file.close();
        return false;
    }
    const auto &conf = nlohmann::json::parse(file);

    numObs_ = conf["num_obs"].get<int>();
    numActions_ = conf["num_actions"].get<int>();
    numHistory_ = conf["num_history"].get<int>();
    phase_ = 0.0;
    phaseIncrement_ = conf["phase_increment"].get<double>();
    auto defaultAngles = conf["default_angles"];
    auto actionScales = conf["action_scale"];
    auto jointStiffness = conf["joint_stiffness"];
    auto jointDamping = conf["joint_damping"];

    crl::utils::resize(action_, numActions_);
    crl::utils::resize(defaultJointPos_, defaultAngles.size());
    crl::utils::resize(actionScale_, actionScales.size());
    crl::utils::resize(jointStiffness_, jointStiffness.size());
    crl::utils::resize(jointDamping_, jointDamping.size());
    for (size_t i = 0; i < defaultAngles.size(); i++) {
        defaultJointPos_[i] = defaultAngles[i].get<double>();
        actionScale_[i] = actionScales[i].get<double>();
        jointStiffness_[i] = jointStiffness[i].get<double>();
        jointDamping_[i] = jointDamping[i].get<double>();
    }
    int totalInputDim = numObs_ * numHistory_;
    inputData_.resize(totalInputDim);
    outputData_.resize(numActions_);
    inputShape_ = {1, totalInputDim};
    outputShape_ = {1, numActions_};
    crl::utils::resize(currentObs_, numObs_);
    obsHistory_.resize(numHistory_);
    for (int i = 0; i < numHistory_; i++) {
        crl::utils::resize(obsHistory_[i], numObs_);
        for (int j = 0; j < numObs_; j++) {
            obsHistory_[i][j] = 0.0;
        }
    }
    auto modelName = conf["model_name"].get<std::string>();
    // Always resolve modelName relative to the config file's directory
    std::filesystem::path configPath(fileName);
    std::filesystem::path configDir = configPath.parent_path();
    std::filesystem::path modelPath = configDir / modelName;
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    session_ = Ort::Session(env_, modelPath.c_str(), Ort::SessionOptions{nullptr});
    inputTensor_ = Ort::Value::CreateTensor<float>(memory_info, inputData_.data(), inputData_.size(), inputShape_.data(), inputShape_.size());
    outputTensor_ = Ort::Value::CreateTensor<float>(memory_info, outputData_.data(), outputData_.size(), outputShape_.data(), outputShape_.size());
    RCLCPP_INFO(logger_, "Successfully loaded RL policy from: %s", fileName.c_str());
    return true;
}

void CRLG1DeepMimicController::computeAndApplyControlSignals(double dt) {
    // Update timer
    timer += dt;
    // Get robot state
    const auto &state = data_->getRobotState();
    const auto &sensor = data_->getSensor();
    int obsIndex = 0;
    crl::V3D angVelBody = sensor.gyroscope;
    currentObs_[obsIndex++] = angVelBody[0];
    currentObs_[obsIndex++] = angVelBody[1];
    currentObs_[obsIndex++] = angVelBody[2];
    crl::V3D gravityBody = sensor.imuOrientation.inverse() * crl::V3D(0, 0, -1);
    currentObs_[obsIndex++] = gravityBody[0];
    currentObs_[obsIndex++] = gravityBody[1];
    currentObs_[obsIndex++] = gravityBody[2];
    for (int i = 0; i < static_cast<int>(sensor.jointSensors.size()); i++) {
        currentObs_[obsIndex++] = sensor.jointSensors[i].jointPos - defaultJointPos_[i];
    }
    for (int i = 0; i < static_cast<int>(sensor.jointSensors.size()); i++) {
        currentObs_[obsIndex++] = sensor.jointSensors[i].jointVel;
    }
    for (int i = 0; i < static_cast<int>(action_.size()); i++) {
        currentObs_[obsIndex++] = action_[i];
    }
    currentObs_[obsIndex++] = phase_;
    phase_ += phaseIncrement_;

    if (phase_ >= 1.0) {
        if (!sequenceCompleted_) {
            sequenceCompleted_ = true;
            RCLCPP_INFO(logger_, "Deep mimic sequence completed (phase reached 1.0). Ready to transition to WALK state.");
        }

        if (loopSequence_) {
            // Reset phase to loop the sequence
            phase_ = 0.0;
            sequenceCompleted_ = false;  // Reset completion flag for next loop
            RCLCPP_INFO(logger_, "Looping deep mimic sequence...");
        } else {
            // Keep at 1.0 (stop at completion)
            phase_ = 1.0;
        }
    }

    for (int i = 0; i < numHistory_ - 1; i++) {
        obsHistory_[i] = obsHistory_[i + 1];
    }
    obsHistory_[numHistory_ - 1] = currentObs_;
    int inputIndex = 0;
    for (int hist = 0; hist < numHistory_; hist++) {
        for (int obs = 0; obs < numObs_; obs++) {
            inputData_[inputIndex++] = static_cast<float>(obsHistory_[hist][obs]);
        }
    }
    const char *inputNames[] = {"obs"};
    const char *outputNames[] = {"actions"};
    Ort::RunOptions runOptions;
    session_.Run(runOptions, inputNames, &inputTensor_, 1, outputNames, &outputTensor_, 1);
    crl::utils::resize(action_, numActions_);
    for (int i = 0; i < numActions_; i++) {
        action_[i] = static_cast<double>(outputData_[i]);
    }
    crl::utils::dVector targetJointPos;
    crl::utils::resize(targetJointPos, numActions_);
    for (int i = 0; i < numActions_; i++) {
        targetJointPos[i] = action_[i] * actionScale_[i] + defaultJointPos_[i];
    }
    crl::humanoid::commons::RobotControlSignal control;
    control.jointControl.resize(numActions_);
    for (int i = 0; i < numActions_; i++) {
        control.jointControl[i].name = model_->jointNames[i];
        control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
        control.jointControl[i].desiredPos = targetJointPos[i];
        control.jointControl[i].desiredSpeed = 0;
        control.jointControl[i].desiredTorque = 0;
        control.jointControl[i].stiffness = jointStiffness_[i];
        control.jointControl[i].damping = jointDamping_[i];
    }
    data_->setControlSignal(control);
}

bool CRLG1DeepMimicController::isSequenceComplete() const {
    return sequenceCompleted_;
}

void CRLG1DeepMimicController::setLoopSequence(bool loop) {
    loopSequence_ = loop;
}

} // namespace crl::g1::rlcontroller
