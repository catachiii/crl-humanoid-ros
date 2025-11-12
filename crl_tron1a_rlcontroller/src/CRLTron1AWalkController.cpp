#include "crl_tron1a_rlcontroller/CRLTron1AWalkController.h"
#include <onnxruntime_cxx_api.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <sstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <crl-basic/utils/mathDefs.h>
#include <rclcpp/rclcpp.hpp>

namespace crl::tron1a::rlcontroller {

CRLTron1AWalkController::CRLTron1AWalkController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                         const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                         const rclcpp::Logger &logger)
    : LocomotionController(model, data, logger) {
    // Initialize counters/flags only - vectors will be initialized in loadModelFromParams
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
    inputTensor_ = Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(memory_info, inputData_.data(), inputData_.size(), inputShape_.data(), inputShape_.size());
    outputTensor_ = Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(memory_info, outputData_.data(), outputData_.size(), outputShape_.data(), outputShape_.size());
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
    // encoder is mandatory
    if (!node.get_parameter("robot_controllers_encoder_file", encoderModelPath) || encoderModelPath.empty()) {
        RCLCPP_ERROR(logger_, "Failed to retrieve encoder path from the parameter server! Encoder is required.");
        return false;
    }

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

    // Load encoder session (required)
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

    RCLCPP_INFO(logger_, "Successfully loaded ONNX models from params");
    RCLCPP_INFO(logger_, "Initialized vectors: actions=%d, observations=%d, history_length=%d, encoder_output=%d",
               numActions_, observationSize_, obsHistoryLength_, encoderOutputSize_);
    
    return true;
}

bool CRLTron1AWalkController::loadRLCfg(rclcpp::Node& node) {
    try {
        RCLCPP_INFO(logger_, "Loading RL configuration parameters...");
        
        // ============================================================================
        // Step 1: Set values directly from RobotParameters.h (no override)
        // ============================================================================
        
        // Set default joint angles from RobotModel
        crl::utils::resize(initJointAngles_, model_->jointNames.size());
        for (size_t i = 0; i < model_->jointNames.size(); i++) {
            initJointAngles_[i] = model_->defaultJointConf[i];
        }
        RCLCPP_INFO(logger_, "Set default joint angles from RobotParameters.h");
        
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
            RCLCPP_ERROR(logger_, "Failed to find wheel joint indices! wheel_L_idx_=%d, wheel_R_idx_=%d", 
                        wheel_L_idx_, wheel_R_idx_);
            return false;
        }
        
        // Verify wheel indices match expected values based on RL_TYPE (deployment compatibility)
        const char* rl_value_check = ::getenv("RL_TYPE");
        std::string rl_type_check = (rl_value_check && strlen(rl_value_check) > 0) ? std::string(rl_value_check) : "isaaclab";
        int expected_wheel_L_idx = (rl_type_check == "isaacgym") ? 3 : 6;
        int expected_wheel_R_idx = 7;
        
        if (wheel_L_idx_ != expected_wheel_L_idx || wheel_R_idx_ != expected_wheel_R_idx) {
            RCLCPP_WARN(logger_, "Wheel joint indices don't match expected values for RL_TYPE=%s. "
                        "Found: wheel_L_idx_=%d (expected %d), wheel_R_idx_=%d (expected %d). "
                        "This may indicate a joint ordering mismatch.",
                        rl_type_check.c_str(), wheel_L_idx_, expected_wheel_L_idx, 
                        wheel_R_idx_, expected_wheel_R_idx);
        } else {
            RCLCPP_INFO(logger_, "Wheel joint indices verified: wheel_L_idx_=%d, wheel_R_idx_=%d (RL_TYPE=%s)",
                       wheel_L_idx_, wheel_R_idx_, rl_type_check.c_str());
        }
        
        // Set stiffness/damping directly from RobotModel (use first non-wheel joint as reference)
        // For WF_TRON1A, all non-wheel joints have the same stiffness/damping
        int firstNonWheelIdx = (wheel_L_idx_ == 0) ? 1 : 0;
        controlCfg_.stiffness = model_->jointStiffnessDefault[firstNonWheelIdx];
        controlCfg_.damping = model_->jointDampingDefault[firstNonWheelIdx];
        wheelJointDamping_ = model_->jointDampingDefault[wheel_L_idx_];
        wheelJointTorqueLimit_ = model_->jointTorqueMax[wheel_L_idx_];
        
        RCLCPP_INFO(logger_, "Set from RobotParameters.h: stiffness=%.1f, damping=%.1f, wheel_damping=%.2f, wheel_torque_limit=%.1f",
                   controlCfg_.stiffness, controlCfg_.damping, wheelJointDamping_, wheelJointTorqueLimit_);
        
        // ============================================================================
        // Step 2: Load RL-specific parameters from YAML only
        // ============================================================================
        
        // RL-specific control parameters
        if (!node.has_parameter("action_scale_pos")) {
            node.declare_parameter<double>("action_scale_pos", 0.25);
        }
        if (!node.has_parameter("decimation")) {
            node.declare_parameter<int>("decimation", 4);
        }
        if (!node.has_parameter("user_torque_limit")) {
            node.declare_parameter<double>("user_torque_limit", 80.0);
        }
        
        if (!node.get_parameter("action_scale_pos", controlCfg_.action_scale_pos)) {
            RCLCPP_ERROR(logger_, "Failed to get parameter 'action_scale_pos'");
            return false;
        }
        if (!node.get_parameter("decimation", controlCfg_.decimation)) {
            RCLCPP_ERROR(logger_, "Failed to get parameter 'decimation'");
            return false;
        }
        if (!node.get_parameter("user_torque_limit", controlCfg_.user_torque_limit)) {
            RCLCPP_ERROR(logger_, "Failed to get parameter 'user_torque_limit'");
            return false;
        }
        
        if (controlCfg_.decimation <= 0) {
            RCLCPP_ERROR(logger_, "Invalid decimation: %d (must be > 0)", controlCfg_.decimation);
            return false;
        }
        
        RCLCPP_INFO(logger_, "RL control params: action_scale_pos=%.2f, decimation=%d, user_torque_limit=%.1f", 
                   controlCfg_.action_scale_pos, controlCfg_.decimation, controlCfg_.user_torque_limit);
        RCLCPP_INFO(logger_, "Wheel joint RL config: damping=%.2f, torque_limit=%.1f", 
                   wheelJointDamping_, wheelJointTorqueLimit_);
        
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
        
        // Determine joint position indices based on RL_TYPE (matching deployment logic)
        // Deployment uses different joint orders for isaacgym vs isaaclab
        const char* rl_value = ::getenv("RL_TYPE");
        std::string rl_type = (rl_value && strlen(rl_value) > 0) ? std::string(rl_value) : "isaaclab";
        
        jointPosIdxs_.clear();
        if (rl_type == "isaacgym") {
            // isaacgym order: [abad_L, hip_L, knee_L, wheel_L, abad_R, hip_R, knee_R, wheel_R]
            // jointPosIdxs = [0, 1, 2, 4, 5, 6] (skip wheel_L at index 3)
            jointPosIdxs_ = {0, 1, 2, 4, 5, 6};
            RCLCPP_INFO(logger_, "Using isaacgym joint order: jointPosIdxs_ = [0, 1, 2, 4, 5, 6]");
        } else {
            // isaaclab order: [abad_L, abad_R, hip_L, hip_R, knee_L, knee_R, wheel_L, wheel_R]
            // jointPosIdxs = [0, 1, 2, 3, 4, 5] (skip wheel_L at index 6)
            jointPosIdxs_ = {0, 1, 2, 3, 4, 5};
            RCLCPP_INFO(logger_, "Using isaaclab joint order: jointPosIdxs_ = [0, 1, 2, 3, 4, 5]");
        }
        
        // Verify that the indices match the actual joint names (sanity check)
        if (jointPosIdxs_.size() != 6) {
            RCLCPP_ERROR(logger_, "jointPosIdxs_ must have 6 elements, got %zu", jointPosIdxs_.size());
            return false;
        }

        // Sanity check: ensure jointPosIdxs_ has unique indices and log their names for traceability.
        {
            std::unordered_set<int> uniqueIdxs;
            bool duplicateFound = false;
            for (int idx : jointPosIdxs_) {
                if (!uniqueIdxs.insert(idx).second) {
                    duplicateFound = true;
                    RCLCPP_ERROR(logger_, "Duplicate joint index %d detected in jointPosIdxs_.", idx);
                }
            }
            if (!duplicateFound) {
                std::ostringstream oss;
                for (size_t i = 0; i < jointPosIdxs_.size(); ++i) {
                    if (i > 0) oss << ", ";
                    oss << model_->jointNames[jointPosIdxs_[i]];
                }
                RCLCPP_INFO(logger_, "Observation joint order (%zu DOFs): [%s]",
                            jointPosIdxs_.size(), oss.str().c_str());
            }
        }
        
        encoderInputSize_ = obsHistoryLength_ * observationSize_;
        numObs_ = observationSize_;
        
        RCLCPP_INFO(logger_, "All RL configuration parameters loaded successfully");
        RCLCPP_INFO(logger_, "  Model sizes: actions=%d, observations=%d, history_length=%d, encoder_output=%d",
                   numActions_, observationSize_, obsHistoryLength_, encoderOutputSize_);
        
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
    for (size_t i = 0; i < proprioHistoryBuffer_.size(); i++) {
        proprioHistoryVector_[i] = proprioHistoryBuffer_(i);
    }
    
    // Clip observations after history update (matches WheelfootController behaviour)
    double obsMin = -clipObs_;
    double obsMax = clipObs_;
    for (auto &value : observations_) {
        double clamped = std::max(obsMin, std::min(obsMax, static_cast<double>(value)));
        value = static_cast<robot_controllers::tensor_element_t>(clamped);
    }
    
    // Update currentObs_ for history
    for (int i = 0; i < numObs_; i++) {
        currentObs_[i] = static_cast<double>(obs(i));
    }
}

void CRLTron1AWalkController::computeEncoder() {
    RCLCPP_INFO(logger_, "=== computeEncoder() called ===");
    
    if (!encoderSessionPtr_) {
        RCLCPP_ERROR(logger_, "Encoder session not available but required!");
        return;
    }

    // Verify encoder input size matches history buffer
    int64_t expectedEncoderInputSize = obsHistoryLength_ * observationSize_;
    if (proprioHistoryBuffer_.size() != static_cast<size_t>(expectedEncoderInputSize)) {
        RCLCPP_ERROR(logger_, "Encoder input size mismatch! Buffer size: %zu, Expected: %ld", 
                    proprioHistoryBuffer_.size(), expectedEncoderInputSize);
        return;
    }
    
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                            OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    inputValues.push_back(Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(memoryInfo, proprioHistoryBuffer_.data(),
                                                          proprioHistoryBuffer_.size(),
                                                          encoderInputShapes_[0].data(),
                                                          encoderInputShapes_[0].size()));
    
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues =
        encoderSessionPtr_->Run(runOptions, encoderInputNames_.data(), inputValues.data(), 1,
                                encoderOutputNames_.data(), 1);
    
    if (outputValues.empty() || outputValues[0].GetTensorMutableData<robot_controllers::tensor_element_t>() == nullptr) {
        RCLCPP_ERROR(logger_, "Encoder returned empty or invalid output!");
        return;
    }
    
    for (int i = 0; i < encoderOutputSize_; i++) {
        encoderOut_[i] = *(outputValues[0].GetTensorMutableData<robot_controllers::tensor_element_t>() + i);
    }
    
    RCLCPP_INFO(logger_, "Encoder output computed successfully (size: %d)", 
                encoderOutputSize_);
}

void CRLTron1AWalkController::computeActions() {
    RCLCPP_INFO(logger_, "=== computeActions() called ===");
    
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                            OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    
    // Combine encoder output, current observations, and scaled commands
    // Order: [encoder_output, current_obs, scaled_commands]
    std::vector<robot_controllers::tensor_element_t> combined_obs;
    
    // Add encoder output (required)
    if (!encoderSessionPtr_ || encoderOut_.empty()) {
        RCLCPP_ERROR(logger_, "Encoder output not available. Ensure computeEncoder() was called and succeeded.");
        return;
    }
    for (const auto &item : encoderOut_) {
        combined_obs.push_back(item);
    }
    RCLCPP_INFO(logger_, "Added encoder output to policy input (size: %zu)", encoderOut_.size());
    
    // Add current observations
    for (const auto &item : observations_) {
        combined_obs.push_back(item);
    }
    RCLCPP_INFO(logger_, "Added current observations to policy input (size: %zu)", observations_.size());
    
    // Add scaled commands
    for (int i = 0; i < static_cast<int>(scaled_commands_.size()); i++) {
        combined_obs.push_back(scaled_commands_[i]);
    }
    RCLCPP_INFO(logger_, "Added scaled commands to policy input (size: %zu, values: [%.3f, %.3f, %.3f])", 
                scaled_commands_.size(), scaled_commands_[0], scaled_commands_[1], scaled_commands_[2]);
    
    // Get policy input shape
    auto policyInputShape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    std::vector<int64_t> inputShapeVec(policyInputShape.begin(), policyInputShape.end());
    
    // Log the actual shape from ONNX model
    std::string shapeStr = "[";
    for (size_t i = 0; i < inputShapeVec.size(); i++) {
        if (i > 0) shapeStr += ", ";
        shapeStr += std::to_string(inputShapeVec[i]);
    }
    shapeStr += "]";
    RCLCPP_INFO(logger_, "Policy input shape from ONNX: %s, combined_obs size: %zu", 
                shapeStr.c_str(), combined_obs.size());
    
    // Create tensor shape - handle dynamic dimensions (-1)
    // For shapes like [-1, 34], replace -1 with 1 (batch size)
    // For shapes like [34], use as-is
    std::vector<int64_t> tensorShape;
    if (inputShapeVec.size() == 1 && inputShapeVec[0] == -1) {
        // Dynamic 1D: [features] -> use actual size
        tensorShape.push_back(static_cast<int64_t>(combined_obs.size()));
    } else if (inputShapeVec.size() == 1) {
        // Fixed 1D: [features]
        tensorShape = inputShapeVec;
    } else {
        // 2D+: [batch, features, ...]
        // Replace -1 in batch dimension with 1
        tensorShape.push_back(inputShapeVec[0] == -1 ? 1 : inputShapeVec[0]);
        for (size_t i = 1; i < inputShapeVec.size(); i++) {
            tensorShape.push_back(inputShapeVec[i]);
        }
    }
    
    // Calculate expected total elements (excluding dynamic dimensions)
    int64_t expectedElements = 1;
    for (size_t i = 0; i < tensorShape.size(); i++) {
        expectedElements *= tensorShape[i];
    }
    
    if (static_cast<int64_t>(combined_obs.size()) != expectedElements) {
        RCLCPP_ERROR(logger_, "Policy input size mismatch! Combined obs size: %zu, Expected elements: %ld", 
                    combined_obs.size(), expectedElements);
        RCLCPP_ERROR(logger_, "ONNX shape: %s, Tensor shape: [", shapeStr.c_str());
        for (size_t i = 0; i < tensorShape.size(); i++) {
            RCLCPP_ERROR(logger_, "%s%ld", (i > 0 ? ", " : ""), tensorShape[i]);
        }
        RCLCPP_ERROR(logger_, "]");
        RCLCPP_ERROR(logger_, "Encoder output: %zu, Observations: %zu, Commands: %zu", 
                    encoderSessionPtr_ ? encoderOut_.size() : 0, observations_.size(), scaled_commands_.size());
        return;
    }
    
    inputValues.push_back(
        Ort::Value::CreateTensor<robot_controllers::tensor_element_t>(memoryInfo, combined_obs.data(), combined_obs.size(),
                                       tensorShape.data(), tensorShape.size()));
    
    // Run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = session_.Run(runOptions, policyInputNames_.data(),
                                                        inputValues.data(), 1, policyOutputNames_.data(),
                                                        policyOutputNames_.size());
    
    if (outputValues.empty() || outputValues[0].GetTensorMutableData<robot_controllers::tensor_element_t>() == nullptr) {
        RCLCPP_ERROR(logger_, "Policy returned empty or invalid output!");
        return;
    }
    
    for (size_t i = 0; i < static_cast<size_t>(numActions_); i++) {
        action_[i] = *(outputValues[0].GetTensorMutableData<robot_controllers::tensor_element_t>() + i);
    }
    
    // Clip actions
    double actionMin = -clipActions_;
    double actionMax = clipActions_;
    std::transform(action_.begin(), action_.end(), action_.begin(),
                   [actionMin, actionMax](double x) { return std::max(actionMin, std::min(actionMax, x)); });
    
    if (!model_->jointNames.empty() && action_.size() == model_->jointNames.size()) {
        std::ostringstream oss;
        oss << "Policy inference completed. Actions:";
        oss << std::fixed << std::setprecision(3);
        for (size_t i = 0; i < action_.size(); ++i) {
            oss << " " << model_->jointNames[i] << "=" << action_[i];
            if (i + 1 < action_.size()) {
                oss << ",";
            }
        }
        RCLCPP_INFO(logger_, "%s", oss.str().c_str());
    } else {
        RCLCPP_INFO(logger_, "Policy inference completed. Total actions: %zu", action_.size());
    }
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
    
    // Get current joint positions and velocities
    const auto &sensor = data_->getSensor();
    crl::utils::dVector jointPos(model_->jointNames.size());
    crl::utils::dVector jointVel(model_->jointNames.size());
    for (size_t i = 0; i < sensor.jointSensors.size(); i++) {
        jointPos[i] = sensor.jointSensors[i].jointPos;
        jointVel[i] = sensor.jointSensors[i].jointVel;
    }
    
    // Compute observation & actions with decimation (matches original deploy logic)
    if (loopCount_ % controlCfg_.decimation == 0) {
        RCLCPP_INFO(logger_, "Decimation check passed (loopCount=%ld), calling computeObservation/Encoder/Actions", loopCount_);
        computeObservation();
        computeEncoder();
        computeActions();
        
    }
    
    // Apply actions to joints (similar to WheelfootController::handleWalkMode)
    crl::humanoid::commons::RobotControlSignal control;
    control.jointControl.resize(model_->jointNames.size());
    
    // Log control signals being applied (only on decimation steps to avoid spam)
    static int controlLogCounter = 0;
    bool shouldLogControl = (loopCount_ % controlCfg_.decimation == 0);
    
    for (size_t i = 0; i < model_->jointNames.size(); i++) {
        control.jointControl[i].name = model_->jointNames[i];
        
        if (i != static_cast<size_t>(wheel_L_idx_) && i != static_cast<size_t>(wheel_R_idx_)) {
            // Regular joint (not wheel): position control
            double actionMin = jointPos[i] - initJointAngles_[i] +
                (controlCfg_.damping * jointVel[i] - controlCfg_.user_torque_limit) / controlCfg_.stiffness;
            double actionMax = jointPos[i] - initJointAngles_[i] +
                (controlCfg_.damping * jointVel[i] + controlCfg_.user_torque_limit) / controlCfg_.stiffness;
            // Limit action first, THEN update lastActions_
            action_[i] = std::max(actionMin / controlCfg_.action_scale_pos,
                                  std::min(actionMax / controlCfg_.action_scale_pos, action_[i]));
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
            
            // Log wheel control signals
            if (shouldLogControl) {
                RCLCPP_INFO(logger_, "Wheel %s: action=%.3f, desiredVel=%.3f rad/s, currentVel=%.3f rad/s, effectiveTorque=%.3f Nm (limit=%.1f Nm)", 
                           model_->jointNames[i].c_str(), action_[i], velocity_des, jointVel[i], 
                           effectiveTorque, wheelJointTorqueLimit_);
            }
        }
        
        control.jointControl[i].desiredTorque = 0;
    }
    
    data_->setControlSignal(control);
}

} // namespace crl::tron1a::rlcontroller
