#include "crl_tron1a_rlcontroller/CRLTron1AWalkController.h"
#include <onnxruntime_cxx_api.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <crl-basic/utils/mathDefs.h>
#include <rclcpp/rclcpp.hpp>

namespace crl::tron1a::rlcontroller {

CRLTron1AWalkController::CRLTron1AWalkController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                         const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                         const rclcpp::Logger &logger)
    : LocomotionController(model, data, logger) {
    // Initialize action vectors with robot joint count
    int jointCount = static_cast<int>(model->jointNames.size());
    crl::utils::resize(action_, jointCount);
    crl::utils::resize(lastActions_, jointCount);
    commands_.setZero();
    scaled_commands_.setZero();
    loopCount_ = 0;
    isfirstRecObs_ = true;
}

bool CRLTron1AWalkController::loadModelFromFile(const std::string &fileName) {
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

bool CRLTron1AWalkController::loadModelFromParams(rclcpp::Node& node) {
    // Load configuration first
    if (!loadRLCfg(node)) {
        RCLCPP_ERROR(logger_, "Failed to load RL configuration");
        return false;
    }

    std::string policyModelPath;
    std::string encoderModelPath;

    if (!node.get_parameter("robot_controllers_policy_file", policyModelPath)) {
        RCLCPP_ERROR(logger_, "Failed to retrieve policy path from the parameter server!");
        return false;
    }
    // encoder is optional; try to get it but do not fail if missing
    node.get_parameter("robot_controllers_encoder_file", encoderModelPath);

    // Create session options similar to WheelfootController
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetInterOpNumThreads(1);

    Ort::AllocatorWithDefaultOptions allocator;

    // Load policy session
    try {
        session_ = Ort::Session(env_, policyModelPath.c_str(), sessionOptions);
        
        // Get policy input/output names and shapes
        policyInputNames_.clear();
        policyOutputNames_.clear();
        for (size_t i = 0; i < session_.GetInputCount(); i++) {
            policyInputNames_.push_back(session_.GetInputName(i, allocator));
        }
        for (size_t i = 0; i < session_.GetOutputCount(); i++) {
            policyOutputNames_.push_back(session_.GetOutputName(i, allocator));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to load policy model: %s", e.what());
        return false;
    }

    // Load encoder session if provided
    if (!encoderModelPath.empty()) {
        try {
            encoderSessionPtr_ = std::make_unique<Ort::Session>(env_, encoderModelPath.c_str(), sessionOptions);
            
            // Get encoder input/output names and shapes
            encoderInputNames_.clear();
            encoderOutputNames_.clear();
            encoderInputShapes_.clear();
            encoderOutputShapes_.clear();
            
            for (size_t i = 0; i < encoderSessionPtr_->GetInputCount(); i++) {
                encoderInputNames_.push_back(encoderSessionPtr_->GetInputName(i, allocator));
                encoderInputShapes_.push_back(encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            }
            for (size_t i = 0; i < encoderSessionPtr_->GetOutputCount(); i++) {
                encoderOutputNames_.push_back(encoderSessionPtr_->GetOutputName(i, allocator));
                encoderOutputShapes_.push_back(encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            }
            
            // Compute encoder output size
            if (!encoderOutputShapes_.empty()) {
                int64_t encoderOutSize = 1;
                for (auto d : encoderOutputShapes_[0]) encoderOutSize *= (d > 0 ? d : 1);
                encoderOutputSize_ = static_cast<int>(encoderOutSize);
                encoderOut_.resize(encoderOutputSize_);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Failed to load encoder model (optional): %s", e.what());
            encoderSessionPtr_ = nullptr;
        }
    }

    // Initialize observation and action buffers
    observations_.resize(observationSize_);
    proprioHistoryVector_.resize(obsHistoryLength_ * observationSize_);
    
    // Initialize action vectors
    crl::utils::resize(action_, numActions_);
    crl::utils::resize(lastActions_, numActions_);
    lastActions_.setZero();
    
    // Initialize observation history
    crl::utils::resize(currentObs_, numObs_);
    obsHistory_.resize(numHistory_);
    for (int i = 0; i < numHistory_; i++) {
        crl::utils::resize(obsHistory_[i], numObs_);
        obsHistory_[i].setZero();
    }

    RCLCPP_INFO(logger_, "Successfully loaded ONNX models from params");
    return true;
}

bool CRLTron1AWalkController::loadRLCfg(rclcpp::Node& node) {
    try {
        int error = 0;
        
        // Load init joint angles
        std::map<std::string, double> initState;
        for (size_t i = 0; i < model_->jointNames.size(); i++) {
            std::string paramName = "ControllerCfg.init_state.default_joint_angle." + model_->jointNames[i];
            double value = 0.0;
            if (!node.get_parameter(paramName, value)) {
                node.declare_parameter<double>(paramName, value);
                node.get_parameter(paramName, value);
            }
            initState[model_->jointNames[i]] = value;
        }
        
        // Initialize initJointAngles
        crl::utils::resize(initJointAngles_, model_->jointNames.size());
        for (size_t i = 0; i < model_->jointNames.size(); i++) {
            initJointAngles_[i] = initState[model_->jointNames[i]];
        }
        
        // Load control config
        error += !node.get_parameter("ControllerCfg.control.stiffness", controlCfg_.stiffness);
        error += !node.get_parameter("ControllerCfg.control.damping", controlCfg_.damping);
        error += !node.get_parameter("ControllerCfg.control.action_scale_pos", controlCfg_.action_scale_pos);
        error += !node.get_parameter("ControllerCfg.control.decimation", controlCfg_.decimation);
        error += !node.get_parameter("ControllerCfg.control.user_torque_limit", controlCfg_.user_torque_limit);
        
        // Load observation scales
        error += !node.get_parameter("ControllerCfg.normalization.obs_scales.ang_vel", obsScales_.angVel);
        error += !node.get_parameter("ControllerCfg.normalization.obs_scales.dof_pos", obsScales_.dofPos);
        error += !node.get_parameter("ControllerCfg.normalization.obs_scales.dof_vel", obsScales_.dofVel);
        
        // Load clipping
        error += !node.get_parameter("ControllerCfg.normalization.clip_scales.clip_actions", clipActions_);
        error += !node.get_parameter("ControllerCfg.normalization.clip_scales.clip_observations", clipObs_);
        
        // Load sizes
        error += !node.get_parameter("ControllerCfg.size.actions_size", numActions_);
        error += !node.get_parameter("ControllerCfg.size.observations_size", observationSize_);
        error += !node.get_parameter("ControllerCfg.size.obs_history_length", obsHistoryLength_);
        error += !node.get_parameter("ControllerCfg.size.encoder_output_size", encoderOutputSize_);
        
        // Load IMU offset
        error += !node.get_parameter("ControllerCfg.imu_orientation_offset.yaw", imuOrientationOffset_[0]);
        error += !node.get_parameter("ControllerCfg.imu_orientation_offset.pitch", imuOrientationOffset_[1]);
        error += !node.get_parameter("ControllerCfg.imu_orientation_offset.roll", imuOrientationOffset_[2]);
        
        // Load user command scales
        error += !node.get_parameter("ControllerCfg.user_cmd_scales.lin_vel_x", userCmdCfg_.linVel_x);
        error += !node.get_parameter("ControllerCfg.user_cmd_scales.lin_vel_y", userCmdCfg_.linVel_y);
        error += !node.get_parameter("ControllerCfg.user_cmd_scales.ang_vel_yaw", userCmdCfg_.angVel_yaw);
        
        // Load wheel joint config
        error += !node.get_parameter("ControllerCfg.control.wheel_joint_damping", wheelJointDamping_);
        error += !node.get_parameter("ControllerCfg.control.wheel_joint_torque_limit", wheelJointTorqueLimit_);
        
        // Determine joint position indices (for WF_TRON1A, typically all joints except wheels)
        jointPosIdxs_.clear();
        for (size_t i = 0; i < model_->jointNames.size(); i++) {
            if (i != static_cast<size_t>(wheel_L_idx_) && i != static_cast<size_t>(wheel_R_idx_)) {
                jointPosIdxs_.push_back(static_cast<int>(i));
            }
        }
        
        encoderInputSize_ = obsHistoryLength_ * observationSize_;
        numObs_ = observationSize_;  // Can be adjusted based on actual observation composition
        
        if (error > 0) {
            RCLCPP_WARN(logger_, "Some configuration parameters could not be loaded (%d errors)", error);
        }
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error loading RL configuration: %s", e.what());
        return false;
    }
}

void CRLTron1AWalkController::computeObservation() {
    const auto &sensor = data_->getSensor();
    const auto &command = data_->getCommand();
    
    // Get IMU orientation as quaternion
    Eigen::Quaterniond q_wi = sensor.imuOrientation;
    
    // Convert quaternion to ZYX Euler angles and calculate inverse rotation matrix
    using namespace robot_controllers;
    vector3_t zyx = quatToZyx(q_wi);
    matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
    
    // Define gravity vector and project it to the body frame
    vector3_t gravityVector(0, 0, -1);
    vector3_t projectedGravity(inverseRot * gravityVector);
    
    // Get base angular velocity
    vector3_t baseAngVel(sensor.gyroscope[0], sensor.gyroscope[1], sensor.gyroscope[2]);
    
    // Apply orientation offset
    vector3_t _zyx(imuOrientationOffset_[0], imuOrientationOffset_[1], imuOrientationOffset_[2]);
    matrix_t rot = getRotationMatrixFromZyxEulerAngles(_zyx);
    baseAngVel = rot * baseAngVel;
    projectedGravity = rot * projectedGravity;
    
    // Get joint positions and velocities
    vector_t jointPos(model_->jointNames.size());
    vector_t jointVel(model_->jointNames.size());
    for (size_t i = 0; i < sensor.jointSensors.size(); ++i) {
        jointPos(i) = sensor.jointSensors[i].jointPos;
        jointVel(i) = sensor.jointSensors[i].jointVel;
    }
    
    // Get last actions
    vector_t actions(numActions_);
    for (int i = 0; i < numActions_; i++) {
        actions(i) = lastActions_[i];
    }
    
    // Scale commands
    matrix_t commandScaler = Eigen::DiagonalMatrix<double, 3>(userCmdCfg_.linVel_x,
                                                               userCmdCfg_.linVel_y,
                                                               userCmdCfg_.angVel_yaw);
    commands_(0) = command.targetForwardSpeed;
    commands_(1) = command.targetSidewaysSpeed;
    commands_(2) = command.targetTurningSpeed;
    vector3_t scaled_commands = commandScaler * commands_;
    
    // Build observation vector
    vector_t jointPos_value = (jointPos - initJointAngles_) * obsScales_.dofPos;
    vector_t jointPos_input(jointPosIdxs_.size());
    for (size_t i = 0; i < jointPosIdxs_.size(); i++) {
        jointPos_input(i) = jointPos_value(jointPosIdxs_[i]);
    }
    
    vector_t obs(observationSize_);
    int obsIdx = 0;
    
    // Angular velocity
    obs.segment(obsIdx, 3) = baseAngVel * obsScales_.angVel;
    obsIdx += 3;
    
    // Projected gravity
    obs.segment(obsIdx, 3) = projectedGravity;
    obsIdx += 3;
    
    // Joint positions (filtered)
    obs.segment(obsIdx, jointPos_input.size()) = jointPos_input;
    obsIdx += static_cast<int>(jointPos_input.size());
    
    // Joint velocities
    obs.segment(obsIdx, jointVel.size()) = jointVel * obsScales_.dofVel;
    obsIdx += static_cast<int>(jointVel.size());
    
    // Last actions
    obs.segment(obsIdx, actions.size()) = actions;
    
    // Update observation history buffer
    if (isfirstRecObs_) {
        int64_t inputSize = obsHistoryLength_ * observationSize_;
        proprioHistoryBuffer_.resize(inputSize);
        for (int i = 0; i < obsHistoryLength_; i++) {
            proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = obs.cast<float>();
        }
        isfirstRecObs_ = false;
    }
    
    // Shift history
    proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) = 
        proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
    proprioHistoryBuffer_.tail(observationSize_) = obs.cast<float>();
    
    // Update observation and scaled commands vectors
    for (size_t i = 0; i < obs.size(); i++) {
        observations_[i] = static_cast<float>(obs(i));
    }
    for (size_t i = 0; i < scaled_commands.size(); i++) {
        scaled_commands_[i] = static_cast<float>(scaled_commands(i));
    }
    for (size_t i = 0; i < proprioHistoryBuffer_.size(); i++) {
        proprioHistoryVector_[i] = proprioHistoryBuffer_(i);
    }
    
    // Clip observations
    double obsMin = -clipObs_;
    double obsMax = clipObs_;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                   [obsMin, obsMax](float x) { return std::max(static_cast<float>(obsMin), std::min(static_cast<float>(obsMax), x)); });
    
    // Update currentObs_ for history
    for (int i = 0; i < numObs_; i++) {
        currentObs_[i] = static_cast<double>(obs(i));
    }
}

void CRLTron1AWalkController::computeEncoder() {
    if (!encoderSessionPtr_) {
        // No encoder, skip
        return;
    }
    
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                            OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    inputValues.push_back(Ort::Value::CreateTensor<float>(memoryInfo, proprioHistoryBuffer_.data(),
                                                          proprioHistoryBuffer_.size(),
                                                          encoderInputShapes_[0].data(),
                                                          encoderInputShapes_[0].size()));
    
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues =
        encoderSessionPtr_->Run(runOptions, encoderInputNames_.data(), inputValues.data(), 1,
                                encoderOutputNames_.data(), 1);
    for (int i = 0; i < encoderOutputSize_; i++) {
        encoderOut_[i] = *(outputValues[0].GetTensorMutableData<float>() + i);
    }
}

void CRLTron1AWalkController::computeActions() {
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                            OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    
    // Combine encoder output, current observations, and scaled commands
    std::vector<float> combined_obs;
    if (encoderSessionPtr_) {
        for (const auto &item : encoderOut_) {
            combined_obs.push_back(item);
        }
    }
    for (const auto &item : observations_) {
        combined_obs.push_back(item);
    }
    for (int i = 0; i < scaled_commands_.size(); i++) {
        combined_obs.push_back(scaled_commands_[i]);
    }
    
    // Get policy input shape
    auto policyInputShape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    std::vector<int64_t> inputShapeVec(policyInputShape.begin(), policyInputShape.end());
    
    inputValues.push_back(
        Ort::Value::CreateTensor<float>(memoryInfo, combined_obs.data(), combined_obs.size(),
                                       inputShapeVec.data(), inputShapeVec.size()));
    
    // Run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = session_.Run(runOptions, policyInputNames_.data(),
                                                        inputValues.data(), 1, policyOutputNames_.data(),
                                                        policyOutputNames_.size());
    
    for (size_t i = 0; i < static_cast<size_t>(numActions_); i++) {
        actions_[i] = *(outputValues[0].GetTensorMutableData<float>() + i);
    }
    
    // Clip actions
    double actionMin = -clipActions_;
    double actionMax = clipActions_;
    std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                   [actionMin, actionMax](double x) { return std::max(actionMin, std::min(actionMax, x)); });
}

void CRLTron1AWalkController::computeAndApplyControlSignals(double dt) {
    // Update timer and loop count
    timer += dt;
    loopCount_++;
    
    // Compute observation & actions with decimation
    if (controlCfg_.decimation == 0) {
        RCLCPP_ERROR(logger_, "Error: controlCfg_.decimation is 0");
        return;
    }
    
    if (loopCount_ % controlCfg_.decimation == 0) {
        computeObservation();
        computeEncoder();
        computeActions();
        
        // Limit action range (already done in computeActions, but double-check)
        double actionMin = -clipActions_;
        double actionMax = clipActions_;
        std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                       [actionMin, actionMax](double x) { return std::max(actionMin, std::min(actionMax, x)); });
    }
    
    // Get current joint positions and velocities
    const auto &sensor = data_->getSensor();
    crl::utils::dVector jointPos(model_->jointNames.size());
    crl::utils::dVector jointVel(model_->jointNames.size());
    for (size_t i = 0; i < sensor.jointSensors.size(); i++) {
        jointPos[i] = sensor.jointSensors[i].jointPos;
        jointVel[i] = sensor.jointSensors[i].jointVel;
    }
    
    // Apply actions to joints (similar to WheelfootController::handleWalkMode)
    crl::humanoid::commons::RobotControlSignal control;
    control.jointControl.resize(model_->jointNames.size());
    
    for (size_t i = 0; i < model_->jointNames.size(); i++) {
        control.jointControl[i].name = model_->jointNames[i];
        
        if (i != static_cast<size_t>(wheel_L_idx_) && i != static_cast<size_t>(wheel_R_idx_)) {
            // Regular joint (not wheel): position control
            double actionMin = jointPos[i] - initJointAngles_[i] +
                (controlCfg_.damping * jointVel[i] - controlCfg_.user_torque_limit) / controlCfg_.stiffness;
            double actionMax = jointPos[i] - initJointAngles_[i] +
                (controlCfg_.damping * jointVel[i] + controlCfg_.user_torque_limit) / controlCfg_.stiffness;
            actions_[i] = std::max(actionMin / controlCfg_.action_scale_pos,
                                  std::min(actionMax / controlCfg_.action_scale_pos, actions_[i]));
            double pos_des = actions_[i] * controlCfg_.action_scale_pos + initJointAngles_[i];
            
            control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
            control.jointControl[i].desiredPos = pos_des;
            control.jointControl[i].desiredSpeed = 0;
            control.jointControl[i].stiffness = controlCfg_.stiffness;
            control.jointControl[i].damping = controlCfg_.damping;
            
            lastActions_[i] = actions_[i];
        } else {
            // Wheel joint: velocity control
            double actionMin = (jointVel[i] - wheelJointTorqueLimit_ / wheelJointDamping_);
            double actionMax = (jointVel[i] + wheelJointTorqueLimit_ / wheelJointDamping_);
            lastActions_[i] = actions_[i];
            actions_[i] = std::max(actionMin / wheelJointDamping_,
                                  std::min(actionMax / wheelJointDamping_, actions_[i]));
            double velocity_des = actions_[i] * wheelJointDamping_;
            
            control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::VELOCITY_MODE;
            control.jointControl[i].desiredPos = 0;
            control.jointControl[i].desiredSpeed = velocity_des;
            control.jointControl[i].stiffness = 0;
            control.jointControl[i].damping = wheelJointDamping_;
        }
        
        control.jointControl[i].desiredTorque = 0;
    }
    
    data_->setControlSignal(control);
}

} // namespace crl::tron1a::rlcontroller
