#include "crl_tron1a_rlcontroller/CRLTron1AWalkController.h"

#include <crl-basic/utils/mathDefs.h>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>
#include <fstream>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unordered_set>

namespace {
template <typename T>
constexpr T square(T value) {
    return value * value;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(static_cast<SCALAR_T>(-2.) * (q.x() * q.z() - q.w() * q.y()), static_cast<SCALAR_T>(.99999));
    zyx(0) = std::atan2(static_cast<SCALAR_T>(2) * (q.x() * q.y() + q.w() * q.z()),
                        square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(static_cast<SCALAR_T>(2) * (q.y() * q.z() + q.w() * q.x()),
                        square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(
    const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) {
    const SCALAR_T z = eulerAngles(0);
    const SCALAR_T y = eulerAngles(1);
    const SCALAR_T x = eulerAngles(2);

    const SCALAR_T c1 = std::cos(z);
    const SCALAR_T c2 = std::cos(y);
    const SCALAR_T c3 = std::cos(x);
    const SCALAR_T s1 = std::sin(z);
    const SCALAR_T s2 = std::sin(y);
    const SCALAR_T s3 = std::sin(x);

    const SCALAR_T s2s3 = s2 * s3;
    const SCALAR_T s2c3 = s2 * c3;

    Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
    rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                      s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                      -s2,          c2 * s3,                   c2 * c3;
    return rotationMatrix;
}
}

namespace crl::tron1a::rlcontroller {

    CRLTron1AWalkController::CRLTron1AWalkController(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                                                     const std::shared_ptr<crl::humanoid::commons::RobotData>& data, const rclcpp::Logger& logger)
        : LocomotionController(model, data, logger),
          memoryInfo_(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault)) {
        // Initialize counters/flags only - vectors will be initialized in loadModelFromParams
        isfirstRecObs_ = true;
    }

    bool CRLTron1AWalkController::loadModelFromParams(const std::string &fileName) {
        // Set default joint angles from RobotModel
        crl::resize(initJointAngles_, model_->jointNames.size());
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

        std::filesystem::path configPath(fileName);
        std::filesystem::path policyModelPath;
        std::filesystem::path encoderModelPath;

        try {
                std::ifstream file(configPath);
                if (file.fail()) {
                    RCLCPP_ERROR(logger_, "Failed to load RL policy configuration file: %s", configPath.c_str());
                    return false;
                }
                const auto &conf = nlohmann::json::parse(file);

                std::string rl_type = conf.value("rl_type", "isaacgym");

                int firstNonWheelIdx = (wheel_L_idx_ == 0) ? 1 : 0;
                controlCfg_.stiffness = model_->jointStiffnessDefault[firstNonWheelIdx];
                controlCfg_.damping = model_->jointDampingDefault[firstNonWheelIdx];
                wheelJointDamping_ = model_->jointDampingDefault[wheel_L_idx_];
                wheelJointTorqueLimit_ = model_->jointTorqueMax[wheel_L_idx_];

                // RL-specific control parameters
                controlCfg_.action_scale_pos = conf.value("action_scale_pos", 0.25);
                controlCfg_.user_torque_limit = conf.value("user_torque_limit", 80.0);

                // Normalization parameters (RL-specific)
                auto obsScales = conf["obs_scales"];
                obsScales_.linVel = obsScales.value("lin_vel", 2.0);
                obsScales_.angVel = obsScales.value("ang_vel", 0.25);
                obsScales_.dofPos = obsScales.value("dof_pos", 1.0);
                obsScales_.dofVel = obsScales.value("dof_vel", 0.05);

                clipActions_ = conf.value("clip_actions", 100.0);
                clipObs_ = conf.value("clip_observations", 100.0);

                // Model size parameters (RL-specific)
                numActions_ = conf.value("actions_size", 8);
                observationSize_ = conf.value("observations_size", 28);
                obsHistoryLength_ = conf.value("obs_history_length", 10);
                encoderOutputSize_ = conf.value("encoder_output_size", 3);

                // IMU orientation offset (calibration parameter)
                auto imuOffset = conf["imu_offset"];
                imuOrientationOffset_[0] = imuOffset.value("yaw", 0.0);
                imuOrientationOffset_[1] = imuOffset.value("pitch", 0.0);
                imuOrientationOffset_[2] = imuOffset.value("roll", 0.0);

                // User command scales (RL-specific)
                auto cmdScales = conf["cmd_scales"];
                userCmdCfg_.linVel_x = cmdScales.value("lin_vel_x", 1.5);
                userCmdCfg_.linVel_y = cmdScales.value("lin_vel_y", 1.0);
                userCmdCfg_.angVel_yaw = cmdScales.value("ang_vel_yaw", 0.5);

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

                std::string policyModelName = conf.value("policy_model", "policy.onnx");
                std::string encoderModelName = conf.value("encoder_model", "encoder.onnx");

                std::filesystem::path configDir = configPath.parent_path();
                policyModelPath = configDir / policyModelName;
                encoderModelPath = configDir / encoderModelName;

            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "Error loading RL configuration: %s", e.what());
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

        // Resize crl::dVector types (crl::resize also zeros them)
        crl::resize(action_, numActions_);
        crl::resize(lastActions_, numActions_);
        crl::resize(currentObs_, numObs_);
        for (int i = 0; i < numHistory_; i++) {
            crl::resize(obsHistory_[i], numObs_);
        }

        crl::resize(commands_, 3);
        crl::resize(scaled_commands_, 3);
        commands_.setZero();
        scaled_commands_.setZero();

        // Resize buffers
        int64_t inputSize = obsHistoryLength_ * observationSize_;
        crl::resize(proprioHistoryBuffer_, inputSize);
        proprioHistoryBuffer_.setZero();
        encoderInputData_.resize(inputSize);

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
            encoderInputTensors_.push_back(Ort::Value::CreateTensor<float>(
                memoryInfo_, encoderInputData_.data(), encoderInputData_.size(), encInputShape.data(), encInputShape.size()));
        }

        // Encoder Output
        if (!encoderOutputShapes_.empty()) {
            std::vector<int64_t> encOutputShape = encoderOutputShapes_[0];
            if (encOutputShape[0] == -1) encOutputShape[0] = 1;

            encoderOutputTensors_.clear();
            encoderOutputTensors_.push_back(Ort::Value::CreateTensor<float>(
                memoryInfo_, encoderOut_.data(), encoderOut_.size(), encOutputShape.data(), encOutputShape.size()));
        }

        // Policy Input
        auto policyInputShapeInfo = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::vector<int64_t> polInputShape = policyInputShapeInfo;
        if (polInputShape[0] == -1) polInputShape[0] = 1;

        policyInputTensors_.clear();
        policyInputTensors_.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo_, combinedObs_.data(), combinedObs_.size(), polInputShape.data(), polInputShape.size()));

        // Policy Output
        auto policyOutputShapeInfo = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::vector<int64_t> polOutputShape = policyOutputShapeInfo;
        if (polOutputShape[0] == -1) polOutputShape[0] = 1;

        policyOutputTensors_.clear();
        policyOutputTensors_.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo_, outputData_.data(), outputData_.size(), polOutputShape.data(), polOutputShape.size()));

        return true;
    }



    void CRLTron1AWalkController::computeObservation() {
        const auto& sensor = data_->getSensor();
        const auto& command = data_->getCommand();

        // Get IMU orientation as quaternion
        crl::Quaternion q_wi = sensor.imuOrientation;

        // Convert quaternion to ZYX Euler angles and calculate inverse rotation matrix
        crl::Vector3d zyx = quatToZyx(q_wi);
        crl::Matrix inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();

        // Define gravity vector and project it to the body frame
        crl::Vector3d gravityVector(0, 0, -1);
        crl::Vector3d projectedGravity(inverseRot * gravityVector);

        // Get base angular velocity
        crl::Vector3d baseAngVel(sensor.gyroscope[0], sensor.gyroscope[1], sensor.gyroscope[2]);

        // Apply orientation offset
        crl::Vector3d _zyx(imuOrientationOffset_[0], imuOrientationOffset_[1], imuOrientationOffset_[2]);
        crl::Matrix rot = getRotationMatrixFromZyxEulerAngles(_zyx);
        baseAngVel = rot * baseAngVel;
        projectedGravity = rot * projectedGravity;

        // Get joint positions and velocities
        crl::dVector jointPos(model_->jointNames.size());
        crl::dVector jointVel(model_->jointNames.size());
        for (size_t i = 0; i < sensor.jointSensors.size(); ++i) {
            jointPos(i) = sensor.jointSensors[i].jointPos;
            jointVel(i) = sensor.jointSensors[i].jointVel;
        }

        // Get last actions
        crl::dVector actions(numActions_);
        for (int i = 0; i < numActions_; i++) {
            actions(i) = lastActions_[i];
        }

        // Scale commands
        crl::Matrix commandScaler = Eigen::DiagonalMatrix<double, 3>(userCmdCfg_.linVel_x, userCmdCfg_.linVel_y, userCmdCfg_.angVel_yaw);
        commands_(0) = command.targetForwardSpeed;
        commands_(1) = command.targetSidewaysSpeed;
        commands_(2) = command.targetTurningSpeed;
        crl::dVector scaled_commands = commandScaler * commands_;

        // Build observation vector
        crl::dVector jointPos_value = (jointPos - initJointAngles_) * obsScales_.dofPos;
        crl::dVector jointPos_input(jointPosIdxs_.size());
        for (size_t i = 0; i < jointPosIdxs_.size(); i++) {
            jointPos_input(i) = jointPos_value(jointPosIdxs_[i]);
        }

        crl::dVector obs(observationSize_);
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
                proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = obs;
            }
            isfirstRecObs_ = false;
        }

        // Shift history
        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
        proprioHistoryBuffer_.tail(observationSize_) = obs;

        // Update observation and scaled commands vectors (for policy input)
        for (size_t i = 0; i < obs.size(); i++) {
            observations_[i] = static_cast<float>(obs(i));
        }
        for (size_t i = 0; i < scaled_commands.size(); i++) {
            scaled_commands_[i] = static_cast<float>(scaled_commands(i));
        }

        // Clip observations after history update
        double obsMin = -clipObs_;
        double obsMax = clipObs_;
        for (auto& value : observations_) {
            double clamped = std::max(obsMin, std::min(obsMax, static_cast<double>(value)));
            value = static_cast<float>(clamped);
        }
    }

    void CRLTron1AWalkController::computeEncoder() {
        if (!encoderSessionPtr_) return;

        // Copy double buffer to float buffer for ONNX
        for (size_t i = 0; i < proprioHistoryBuffer_.size(); i++) {
            encoderInputData_[i] = static_cast<float>(proprioHistoryBuffer_[i]);
        }

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
        crl::dVector jointPos(model_->jointNames.size());
        crl::dVector jointVel(model_->jointNames.size());
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
