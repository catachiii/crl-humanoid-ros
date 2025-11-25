#include "crl_tron1a_rlcontroller/CRLTron1AWalkController.h"

#include <crl-basic/utils/mathDefs.h>
#include <onnxruntime/onnxruntime_cxx_api.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unordered_set>

namespace crl::tron1a::rlcontroller {

    CRLTron1AWalkController::CRLTron1AWalkController(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                                                     const std::shared_ptr<crl::humanoid::commons::RobotData>& data, const rclcpp::Logger& logger)
        : LocomotionController(model, data, logger),
          memoryInfo_(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault)) {
        // Initialize counters/flags only - vectors will be initialized in loadModelFromParams
        isfirstRecObs_ = true;
    }

    bool CRLTron1AWalkController::loadModelFromParams(rclcpp::Node& node) {
        // Load configuration first
        if (!loadRLCfg(node)) {
            RCLCPP_ERROR(logger_, "Failed to load RL configuration");
            return false;
        }

        std::string policyModelPathParam;
        std::string encoderModelPathParam;

        if (!node.get_parameter("robot_controllers_policy_file", policyModelPathParam)) {
            RCLCPP_ERROR(logger_, "Failed to retrieve policy path from the parameter server!");
            return false;
        }
        // encoder is mandatory
        if (!node.get_parameter("robot_controllers_encoder_file", encoderModelPathParam) || encoderModelPathParam.empty()) {
            RCLCPP_ERROR(logger_, "Failed to retrieve encoder path from the parameter server! Encoder is required.");
            return false;
        }

        std::string packageShareDir;
        auto resolveModelPath = [&](const std::string& paramValue, std::filesystem::path& outPath) -> bool {
            std::filesystem::path candidate(paramValue);
            if (candidate.is_absolute()) {
                outPath = candidate;
                return true;
            }

            if (packageShareDir.empty()) {
                try {
                    packageShareDir = ament_index_cpp::get_package_share_directory("crl_tron1a_rlcontroller");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Failed to locate package share directory: %s", e.what());
                    return false;
                }
            }
            outPath = std::filesystem::path(packageShareDir) / candidate;
            return true;
        };

        std::filesystem::path policyModelPath;
        std::filesystem::path encoderModelPath;
        if (!resolveModelPath(policyModelPathParam, policyModelPath) || !resolveModelPath(encoderModelPathParam, encoderModelPath)) {
            return false;
        }
        const std::string policyModelPathStr = policyModelPath.string();
        const std::string encoderModelPathStr = encoderModelPath.string();

        // Create session options similar to WheelfootController
        Ort::SessionOptions sessionOptions;
        sessionOptions.SetIntraOpNumThreads(1);
        sessionOptions.SetInterOpNumThreads(1);

        Ort::AllocatorWithDefaultOptions allocator;

        // Load policy session
        try {
            session_ = Ort::Session(env_, policyModelPathStr.c_str(), sessionOptions);

            // Get policy input/output names and shapes
            policyInputNames_.clear();
            policyOutputNames_.clear();
            for (size_t i = 0; i < session_.GetInputCount(); i++) {
                auto name = session_.GetInputNameAllocated(i, allocator);
                policyInputNames_.push_back(name.get());
            }
            for (size_t i = 0; i < session_.GetOutputCount(); i++) {
                auto name = session_.GetOutputNameAllocated(i, allocator);
                policyOutputNames_.push_back(name.get());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to load policy model: %s", e.what());
            return false;
        }

        // Load encoder session (required)
        try {
            encoderSessionPtr_ = std::make_unique<Ort::Session>(env_, encoderModelPathStr.c_str(), sessionOptions);

            // Get encoder input/output names and shapes
            encoderInputNames_.clear();
            encoderOutputNames_.clear();
            encoderInputShapes_.clear();
            encoderOutputShapes_.clear();

            for (size_t i = 0; i < encoderSessionPtr_->GetInputCount(); i++) {
                auto name = encoderSessionPtr_->GetInputNameAllocated(i, allocator);
                encoderInputNames_.push_back(name.get());
                encoderInputShapes_.push_back(encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            }
            for (size_t i = 0; i < encoderSessionPtr_->GetOutputCount(); i++) {
                auto name = encoderSessionPtr_->GetOutputNameAllocated(i, allocator);
                encoderOutputNames_.push_back(name.get());
                encoderOutputShapes_.push_back(encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            }

            // Compute encoder output size
            if (!encoderOutputShapes_.empty()) {
                int64_t encoderOutSize = 1;
                for (auto d : encoderOutputShapes_[0])
                    encoderOutSize *= (d > 0 ? d : 1);
                encoderOutputSize_ = static_cast<int>(encoderOutSize);
                encoderOut_.resize(encoderOutputSize_);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to load encoder model: %s", e.what());
            encoderSessionPtr_ = nullptr;
            return false;
        }

        // Initialize all vectors and buffers (similar to WheelfootController::loadRLCfg)
        // Note: encoderOut_ is already resized above when encoder loads successfully

        // Resize std::vector types
        observations_.resize(observationSize_);
        proprioHistoryVector_.resize(obsHistoryLength_ * observationSize_);
        obsHistory_.resize(numHistory_);

        // Resize crl::dVector types (crl::utils::resize also zeros them)
        crl::utils::resize(action_, numActions_);
        crl::utils::resize(lastActions_, numActions_);
        crl::utils::resize(currentObs_, numObs_);
        for (int i = 0; i < numHistory_; i++) {
            crl::utils::resize(obsHistory_[i], numObs_);
        }

        // Initialize Eigen vectors to zero
        commands_.setZero();
        scaled_commands_.setZero();

        // Resize buffers
        int64_t inputSize = obsHistoryLength_ * observationSize_;
        proprioHistoryBuffer_.resize(inputSize);
        proprioHistoryBuffer_.setZero();

        combinedObs_.resize(encoderOutputSize_ + observationSize_ + 3); // 3 for scaled_commands
        outputData_.resize(numActions_);

        // Setup C-string arrays
        encoderInputNamesCStr_.clear();
        for (const auto& s : encoderInputNames_) encoderInputNamesCStr_.push_back(s.c_str());
        encoderOutputNamesCStr_.clear();
        for (const auto& s : encoderOutputNames_) encoderOutputNamesCStr_.push_back(s.c_str());

        policyInputNamesCStr_.clear();
        for (const auto& s : policyInputNames_) policyInputNamesCStr_.push_back(s.c_str());
        policyOutputNamesCStr_.clear();
        for (const auto& s : policyOutputNames_) policyOutputNamesCStr_.push_back(s.c_str());

        // Create Tensors
        // Encoder Input
        if (!encoderInputShapes_.empty()) {
            std::vector<int64_t> encInputShape = encoderInputShapes_[0];
            if (encInputShape[0] == -1) encInputShape[0] = 1;

            encoderInputTensors_.clear();
            encoderInputTensors_.push_back(Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(
                memoryInfo_, proprioHistoryBuffer_.data(), proprioHistoryBuffer_.size(), encInputShape.data(), encInputShape.size()));
        }

        // Encoder Output
        if (!encoderOutputShapes_.empty()) {
            std::vector<int64_t> encOutputShape = encoderOutputShapes_[0];
            if (encOutputShape[0] == -1) encOutputShape[0] = 1;

            encoderOutputTensors_.clear();
            encoderOutputTensors_.push_back(Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(
                memoryInfo_, encoderOut_.data(), encoderOut_.size(), encOutputShape.data(), encOutputShape.size()));
        }

        // Policy Input
        auto policyInputShapeInfo = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::vector<int64_t> polInputShape = policyInputShapeInfo;
        if (polInputShape[0] == -1) polInputShape[0] = 1;

        policyInputTensors_.clear();
        policyInputTensors_.push_back(Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(
            memoryInfo_, combinedObs_.data(), combinedObs_.size(), polInputShape.data(), polInputShape.size()));

        // Policy Output
        auto policyOutputShapeInfo = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::vector<int64_t> polOutputShape = policyOutputShapeInfo;
        if (polOutputShape[0] == -1) polOutputShape[0] = 1;

        policyOutputTensors_.clear();
        policyOutputTensors_.push_back(Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(
            memoryInfo_, outputData_.data(), outputData_.size(), polOutputShape.data(), polOutputShape.size()));

        return true;
    }

    bool CRLTron1AWalkController::loadRLCfg(rclcpp::Node& node) {
        try {
            // Set default joint angles from RobotModel
            crl::utils::resize(initJointAngles_, model_->jointNames.size());
            for (size_t i = 0; i < model_->jointNames.size(); i++) {
                initJointAngles_[i] = model_->defaultJointConf[i];
            }

            // Find wheel joint indices by name
            wheel_L_idx_ = -1;
            wheel_R_idx_ = -1;
            for (size_t i = 0; i < model_->jointNames.size(); i++) {
                if (model_->jointNames[i] == "wheel_L_Joint") {
                    wheel_L_idx_ = static_cast<int>(i);
                } else if (model_->jointNames[i] == "wheel_R_Joint") {
                    wheel_R_idx_ = static_cast<int>(i);
                }
            }
            if (wheel_L_idx_ < 0 || wheel_R_idx_ < 0) {
                RCLCPP_ERROR(logger_, "Failed to find wheel joint indices! wheel_L_idx_=%d, wheel_R_idx_=%d", wheel_L_idx_, wheel_R_idx_);
                return false;
            }

            // Verify wheel indices match expected values based on RL_TYPE (deployment compatibility)
            std::string rl_type;
            if (!node.has_parameter("rl_type")) {
                node.declare_parameter<std::string>("rl_type", "isaaclab");
            }
            if (!node.get_parameter("rl_type", rl_type)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'rl_type'");
                return false;
            }

            int firstNonWheelIdx = (wheel_L_idx_ == 0) ? 1 : 0;
            controlCfg_.stiffness = model_->jointStiffnessDefault[firstNonWheelIdx];
            controlCfg_.damping = model_->jointDampingDefault[firstNonWheelIdx];
            wheelJointDamping_ = model_->jointDampingDefault[wheel_L_idx_];
            wheelJointTorqueLimit_ = model_->jointTorqueMax[wheel_L_idx_];

            // RL-specific control parameters
            if (!node.has_parameter("action_scale_pos")) {
                node.declare_parameter<double>("action_scale_pos", 0.25);
            }
            if (!node.has_parameter("user_torque_limit")) {
                node.declare_parameter<double>("user_torque_limit", 80.0);
            }

            if (!node.get_parameter("action_scale_pos", controlCfg_.action_scale_pos)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'action_scale_pos'");
                return false;
            }
            if (!node.get_parameter("user_torque_limit", controlCfg_.user_torque_limit)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'user_torque_limit'");
                return false;
            }

            // Normalization parameters (RL-specific)
            if (!node.has_parameter("obs_scale_lin_vel")) {
                node.declare_parameter<double>("obs_scale_lin_vel", 2.0);
            }
            if (!node.has_parameter("obs_scale_ang_vel")) {
                node.declare_parameter<double>("obs_scale_ang_vel", 0.25);
            }
            if (!node.has_parameter("obs_scale_dof_pos")) {
                node.declare_parameter<double>("obs_scale_dof_pos", 1.0);
            }
            if (!node.has_parameter("obs_scale_dof_vel")) {
                node.declare_parameter<double>("obs_scale_dof_vel", 0.05);
            }
            if (!node.has_parameter("clip_actions")) {
                node.declare_parameter<double>("clip_actions", 100.0);
            }
            if (!node.has_parameter("clip_observations")) {
                node.declare_parameter<double>("clip_observations", 100.0);
            }

            if (!node.get_parameter("obs_scale_lin_vel", obsScales_.linVel)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'obs_scale_lin_vel'");
                return false;
            }
            if (!node.get_parameter("obs_scale_ang_vel", obsScales_.angVel)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'obs_scale_ang_vel'");
                return false;
            }
            if (!node.get_parameter("obs_scale_dof_pos", obsScales_.dofPos)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'obs_scale_dof_pos'");
                return false;
            }
            if (!node.get_parameter("obs_scale_dof_vel", obsScales_.dofVel)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'obs_scale_dof_vel'");
                return false;
            }
            if (!node.get_parameter("clip_actions", clipActions_)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'clip_actions'");
                return false;
            }
            if (!node.get_parameter("clip_observations", clipObs_)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'clip_observations'");
                return false;
            }

            // Model size parameters (RL-specific)
            if (!node.has_parameter("actions_size")) {
                node.declare_parameter<int>("actions_size", 8);
            }
            if (!node.has_parameter("observations_size")) {
                node.declare_parameter<int>("observations_size", 28);
            }
            if (!node.has_parameter("obs_history_length")) {
                node.declare_parameter<int>("obs_history_length", 10);
            }
            if (!node.has_parameter("encoder_output_size")) {
                node.declare_parameter<int>("encoder_output_size", 3);
            }

            if (!node.get_parameter("actions_size", numActions_)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'actions_size'");
                return false;
            }
            if (!node.get_parameter("observations_size", observationSize_)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'observations_size'");
                return false;
            }
            if (!node.get_parameter("obs_history_length", obsHistoryLength_)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'obs_history_length'");
                return false;
            }
            if (!node.get_parameter("encoder_output_size", encoderOutputSize_)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'encoder_output_size'");
                return false;
            }

            // IMU orientation offset (calibration parameter)
            if (!node.has_parameter("imu_offset_yaw")) {
                node.declare_parameter<double>("imu_offset_yaw", 0.0);
            }
            if (!node.has_parameter("imu_offset_pitch")) {
                node.declare_parameter<double>("imu_offset_pitch", 0.0);
            }
            if (!node.has_parameter("imu_offset_roll")) {
                node.declare_parameter<double>("imu_offset_roll", 0.0);
            }

            if (!node.get_parameter("imu_offset_yaw", imuOrientationOffset_[0])) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'imu_offset_yaw'");
                return false;
            }
            if (!node.get_parameter("imu_offset_pitch", imuOrientationOffset_[1])) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'imu_offset_pitch'");
                return false;
            }
            if (!node.get_parameter("imu_offset_roll", imuOrientationOffset_[2])) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'imu_offset_roll'");
                return false;
            }

            // User command scales (RL-specific)
            if (!node.has_parameter("cmd_scale_lin_vel_x")) {
                node.declare_parameter<double>("cmd_scale_lin_vel_x", 1.5);
            }
            if (!node.has_parameter("cmd_scale_lin_vel_y")) {
                node.declare_parameter<double>("cmd_scale_lin_vel_y", 1.0);
            }
            if (!node.has_parameter("cmd_scale_ang_vel_yaw")) {
                node.declare_parameter<double>("cmd_scale_ang_vel_yaw", 0.5);
            }

            if (!node.get_parameter("cmd_scale_lin_vel_x", userCmdCfg_.linVel_x)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'cmd_scale_lin_vel_x'");
                return false;
            }
            if (!node.get_parameter("cmd_scale_lin_vel_y", userCmdCfg_.linVel_y)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'cmd_scale_lin_vel_y'");
                return false;
            }
            if (!node.get_parameter("cmd_scale_ang_vel_yaw", userCmdCfg_.angVel_yaw)) {
                RCLCPP_ERROR(logger_, "Failed to get parameter 'cmd_scale_ang_vel_yaw'");
                return false;
            }

            jointPosIdxs_.clear();
            std::vector<std::string> targetJointNames;

            if (rl_type == "isaacgym") {
                // isaacgym order: [abad_L, hip_L, knee_L, abad_R, hip_R, knee_R]
                targetJointNames = {"abad_L_Joint", "hip_L_Joint", "knee_L_Joint", "abad_R_Joint", "hip_R_Joint", "knee_R_Joint"};
                RCLCPP_INFO(logger_, "Using isaacgym joint order target");
            } else {
                // isaaclab order: [abad_L, abad_R, hip_L, hip_R, knee_L, knee_R]
                targetJointNames = {"abad_L_Joint", "abad_R_Joint", "hip_L_Joint", "hip_R_Joint", "knee_L_Joint", "knee_R_Joint"};
                RCLCPP_INFO(logger_, "Using isaaclab joint order target");
            }

            for (const auto& name : targetJointNames) {
                bool found = false;
                for (size_t i = 0; i < model_->jointNames.size(); i++) {
                    if (model_->jointNames[i] == name) {
                        jointPosIdxs_.push_back(static_cast<int>(i));
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    RCLCPP_ERROR(logger_, "Failed to find joint %s in robot model", name.c_str());
                    return false;
                }
            }

            encoderInputSize_ = obsHistoryLength_ * observationSize_;
            numObs_ = observationSize_;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error loading RL configuration: %s", e.what());
            return false;
        }
    }

    void CRLTron1AWalkController::computeObservation() {
        const auto& sensor = data_->getSensor();
        const auto& command = data_->getCommand();

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
        matrix_t commandScaler = Eigen::DiagonalMatrix<double, 3>(userCmdCfg_.linVel_x, userCmdCfg_.linVel_y, userCmdCfg_.angVel_yaw);
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
            for (int i = 0; i < obsHistoryLength_; i++) {
                proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = obs.cast<robot_controllers::tensor_element_t>();
            }
            isfirstRecObs_ = false;
        }

        // Shift history
        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
        proprioHistoryBuffer_.tail(observationSize_) = obs.cast<robot_controllers::tensor_element_t>();

        // Update observation and scaled commands vectors (for policy input)
        for (size_t i = 0; i < obs.size(); i++) {
            observations_[i] = static_cast<robot_controllers::tensor_element_t>(obs(i));
        }
        for (size_t i = 0; i < scaled_commands.size(); i++) {
            scaled_commands_[i] = static_cast<robot_controllers::tensor_element_t>(scaled_commands(i));
        }

        // Clip observations after history update
        double obsMin = -clipObs_;
        double obsMax = clipObs_;
        for (auto& value : observations_) {
            double clamped = std::max(obsMin, std::min(obsMax, static_cast<double>(value)));
            value = static_cast<robot_controllers::tensor_element_t>(clamped);
        }
    }

    void CRLTron1AWalkController::computeEncoder() {
        if (!encoderSessionPtr_) return;

        Ort::RunOptions runOptions;
        encoderSessionPtr_->Run(runOptions, encoderInputNamesCStr_.data(), encoderInputTensors_.data(), encoderInputTensors_.size(),
                               encoderOutputNamesCStr_.data(), encoderOutputTensors_.data(), encoderOutputTensors_.size());
    }

    void CRLTron1AWalkController::computeActions() {
        // Combine encoder output, current observations, and scaled commands
        // Order: [encoder_output, current_obs, scaled_commands]

        size_t idx = 0;
        // Add encoder output
        for (const auto& item : encoderOut_) {
            combinedObs_[idx++] = item;
        }

        // Add current observations
        for (const auto& item : observations_) {
            combinedObs_[idx++] = item;
        }

        // Add scaled commands
        for (int i = 0; i < static_cast<int>(scaled_commands_.size()); i++) {
            combinedObs_[idx++] = scaled_commands_[i];
        }

        // Run inference
        Ort::RunOptions runOptions;
        session_.Run(runOptions, policyInputNamesCStr_.data(), policyInputTensors_.data(), policyInputTensors_.size(),
                     policyOutputNamesCStr_.data(), policyOutputTensors_.data(), policyOutputTensors_.size());

        // Copy output to action_
        for (size_t i = 0; i < static_cast<size_t>(numActions_); i++) {
            action_[i] = static_cast<double>(outputData_[i]);
        }

        // Clip actions
        double actionMin = -clipActions_;
        double actionMax = clipActions_;
        std::transform(action_.begin(), action_.end(), action_.begin(),
                       [actionMin, actionMax](double x) { return std::max(actionMin, std::min(actionMax, x)); });
    }

    void CRLTron1AWalkController::computeAndApplyControlSignals(double dt) {
        // Update timer
        timer += dt;

        // Get current joint positions and velocities
        const auto& sensor = data_->getSensor();
        crl::utils::dVector jointPos(model_->jointNames.size());
        crl::utils::dVector jointVel(model_->jointNames.size());
        for (size_t i = 0; i < sensor.jointSensors.size(); i++) {
            jointPos[i] = sensor.jointSensors[i].jointPos;
            jointVel[i] = sensor.jointSensors[i].jointVel;
        }

        computeObservation();
        computeEncoder();
        computeActions();

        // Apply actions to joints (similar to WheelfootController::handleWalkMode)
        crl::humanoid::commons::RobotControlSignal control;
        control.jointControl.resize(model_->jointNames.size());

        for (size_t i = 0; i < model_->jointNames.size(); i++) {
            control.jointControl[i].name = model_->jointNames[i];

            if (i != static_cast<size_t>(wheel_L_idx_) && i != static_cast<size_t>(wheel_R_idx_)) {
                // Regular joint (not wheel): position control
                double actionMin =
                    jointPos[i] - initJointAngles_[i] + (controlCfg_.damping * jointVel[i] - controlCfg_.user_torque_limit) / controlCfg_.stiffness;
                double actionMax =
                    jointPos[i] - initJointAngles_[i] + (controlCfg_.damping * jointVel[i] + controlCfg_.user_torque_limit) / controlCfg_.stiffness;
                // Limit action first, THEN update lastActions_
                action_[i] = std::max(actionMin / controlCfg_.action_scale_pos, std::min(actionMax / controlCfg_.action_scale_pos, action_[i]));
                double pos_des = action_[i] * controlCfg_.action_scale_pos + initJointAngles_[i];

                control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
                control.jointControl[i].desiredPos = pos_des;
                control.jointControl[i].desiredSpeed = 0;
                control.jointControl[i].stiffness = controlCfg_.stiffness;
                control.jointControl[i].damping = controlCfg_.damping;

                // Update lastActions_ AFTER limiting (important for next observation)
                lastActions_[i] = action_[i];
            } else {
                // Wheel joint: velocity control (match deployment WheelfootController logic exactly)
                // Deployment code (lines 150-155):
                //   actionMin = (jointVel(i) - wheelJointTorqueLimit_ / wheelJointDamping_)
                //   actionMax = (jointVel(i) + wheelJointTorqueLimit_ / wheelJointDamping_)
                //   lastActions_(i, 0) = actions_[i]
                //   actions_[i] = std::max(actionMin / wheelJointDamping_, std::min(actionMax / wheelJointDamping_, (double)actions_[i]))
                //   velocity_des = actions_[i] * wheelJointDamping_

                // First, save the raw action to lastActions_ (before limiting)
                lastActions_[i] = action_[i];

                // Compute action limits based on current velocity and torque constraints
                double actionMin = (jointVel[i] - wheelJointTorqueLimit_ / wheelJointDamping_) / wheelJointDamping_;
                double actionMax = (jointVel[i] + wheelJointTorqueLimit_ / wheelJointDamping_) / wheelJointDamping_;

                // Limit the action value (not the velocity)
                action_[i] = std::max(actionMin, std::min(actionMax, action_[i]));

                // Compute desired velocity from limited action
                double velocity_des = action_[i] * wheelJointDamping_;

                // Calculate effective torque: torque = damping * (desiredVel - currentVel)
                double effectiveTorque = wheelJointDamping_ * (velocity_des - jointVel[i]);

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

}  // namespace crl::tron1a::rlcontroller
