#include "crl_g1_mimiccontroller/CRLG1DeepTrackController.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <crl-basic/utils/mathDefs.h>

namespace crl::g1::mimiccontroller {

CRLG1DeepTrackController::CRLG1DeepTrackController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                         const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                         const rclcpp::Logger &logger)
    : LocomotionController(model, data, logger) {
    // Initialize action vectors with robot joint count
    int jointCount = static_cast<int>(model->jointNames.size());
    crl::utils::resize(action_, jointCount);
}

bool CRLG1DeepTrackController::loadModelFromFile(const std::string &fileName) {
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
    featureBodyIds_ = conf["feature_body_ids"].get<std::vector<int>>();
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

    auto motionName = conf["motion_name"].get<std::string>();

    // Load motion CSV file
    std::filesystem::path motionPath = std::filesystem::path(CRL_G1_MIMICCONTROLLER_DATA_FOLDER) / "motions" / motionName;
    std::ifstream motionFile(motionPath);
    if (motionFile.fail()) {
        RCLCPP_ERROR(logger_, "Failed to load motion file: %s", motionPath.c_str());
        motionFile.close();
        return false;
    }

    // Read CSV data into matrix
    motionData_.clear();
    std::string line;
    while (std::getline(motionFile, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }

        motionData_.push_back(row);
    }
    motionFile.close();

    RCLCPP_INFO(logger_, "Successfully loaded motion data: %lu rows x %lu columns",
                motionData_.size(), motionData_.empty() ? 0 : motionData_[0].size());

    motionDataLength_ = static_cast<int>(motionData_.size());
    currentFrameIndex_ = 0;

    // preprocess motion data
    preprocessMotionData();

    return true;
}

void CRLG1DeepTrackController::preprocessMotionData() {
    // Motion file joint order
    static const std::vector<std::string> motionJointNames = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    };

    // Create joint index mapping from motion order to policy order
    std::vector<int> jointIndexMapping;
    jointIndexMapping.reserve(model_->jointNames.size());

    for (const auto& policyJointName : model_->jointNames) {
        auto it = std::find(motionJointNames.begin(), motionJointNames.end(), policyJointName);
        if (it != motionJointNames.end()) {
            jointIndexMapping.push_back(static_cast<int>(std::distance(motionJointNames.begin(), it)));
        } else {
            RCLCPP_ERROR(logger_, "Joint %s not found in motion data", policyJointName.c_str());
            jointIndexMapping.push_back(0); // Default to first joint if not found
        }
    }

    // Process each frame
    processedMotionData_.resize(motionData_.size());

    for (size_t frameIdx = 0; frameIdx < motionData_.size(); frameIdx++) {
        const auto& frame = motionData_[frameIdx];

        // Expected frame structure from CSV:
        // [0:3]   - root_pos (3)
        // [3:7]   - root_quat xyzw (4)
        // [7:36]  - joint_pos (29)
        // [36:39] - root_lin_vel (3)
        // [39:42] - root_ang_vel (3)
        // [42:71] - joint_vel (29)
        // [71:]   - body_state (remaining)

        std::vector<double> processedFrame;

        // 1. Root position [0:3]
        processedFrame.insert(processedFrame.end(), frame.begin(), frame.begin() + 3);

        // 2. Root quaternion - convert from xyzw to wxyz format [3:7] -> [6,3,4,5]
        processedFrame.push_back(frame[6]); // w
        processedFrame.push_back(frame[3]); // x
        processedFrame.push_back(frame[4]); // y
        processedFrame.push_back(frame[5]); // z

        // 3. Joint positions - reordered [7:36]
        for (int idx : jointIndexMapping) {
            processedFrame.push_back(frame[7 + idx]);
        }

        // 4. Root linear velocity [36:39]
        processedFrame.insert(processedFrame.end(), frame.begin() + 36, frame.begin() + 39);

        // 5. Root angular velocity [39:42]
        processedFrame.insert(processedFrame.end(), frame.begin() + 39, frame.begin() + 42);

        // 6. Joint velocities - reordered [42:71]
        for (int idx : jointIndexMapping) {
            processedFrame.push_back(frame[42 + idx]);
        }

        // 7. Body state [71:]
        if (frame.size() > 71) {
            processedFrame.insert(processedFrame.end(), frame.begin() + 71, frame.end());
        }

        processedMotionData_[frameIdx] = processedFrame;
    }

    RCLCPP_INFO(logger_, "Preprocessed %lu motion frames", processedMotionData_.size());
}

// Helper function implementations
crl::V3D CRLG1DeepTrackController::quatApplyInverse(const crl::Quaternion& quat, const crl::V3D& vec) const {
    // Apply inverse quaternion rotation to vector
    crl::V3D xyz(quat.x(), quat.y(), quat.z());
    crl::V3D t = xyz.cross(vec) * 2.0;
    return vec - quat.w() * t + xyz.cross(t);
}

crl::Quaternion CRLG1DeepTrackController::quatConjugate(const crl::Quaternion& q) const {
    return crl::Quaternion(q.w(), -q.x(), -q.y(), -q.z());
}

crl::Quaternion CRLG1DeepTrackController::quatMul(const crl::Quaternion& q1, const crl::Quaternion& q2) const {
    double w1 = q1.w(), x1 = q1.x(), y1 = q1.y(), z1 = q1.z();
    double w2 = q2.w(), x2 = q2.x(), y2 = q2.y(), z2 = q2.z();

    double ww = (z1 + x1) * (x2 + y2);
    double yy = (w1 - y1) * (w2 + z2);
    double zz = (w1 + y1) * (w2 - z2);
    double xx = ww + yy + zz;
    double qq = 0.5 * (xx + (z1 - x1) * (x2 - y2));

    double w = qq - ww + (z1 - y1) * (y2 - z2);
    double x = qq - xx + (x1 + w1) * (x2 + w2);
    double y = qq - yy + (w1 - x1) * (y2 + z2);
    double z = qq - zz + (z1 + y1) * (w2 - x2);

    return crl::Quaternion(w, x, y, z);
}

crl::Matrix3x3 CRLG1DeepTrackController::matrixFromQuat(const crl::Quaternion& quat) const {
    double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
    double two_s = 2.0 / (w*w + x*x + y*y + z*z);

    crl::Matrix3x3 mat;
    mat(0, 0) = 1.0 - two_s * (y*y + z*z);
    mat(0, 1) = two_s * (x*y - z*w);
    mat(0, 2) = two_s * (x*z + y*w);
    mat(1, 0) = two_s * (x*y + z*w);
    mat(1, 1) = 1.0 - two_s * (x*x + z*z);
    mat(1, 2) = two_s * (y*z - x*w);
    mat(2, 0) = two_s * (x*z - y*w);
    mat(2, 1) = two_s * (y*z + x*w);
    mat(2, 2) = 1.0 - two_s * (x*x + y*y);

    return mat;
}

void CRLG1DeepTrackController::computeAndApplyControlSignals(double dt) {
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

    // Get motion frame data
    int queryFrame = std::min(currentFrameIndex_, motionDataLength_ - 1);
    const auto& motionFrame = processedMotionData_[queryFrame];

    // Extract motion data from processed frame
    // Frame structure: [root_pos(3), root_quat(4), joint_pos(29), root_lin_vel(3), root_ang_vel(3), joint_vel(29), body_state(...)]
    crl::V3D motionRootPosW(motionFrame[0], motionFrame[1], motionFrame[2]);
    crl::Quaternion motionRootQuatW(motionFrame[3], motionFrame[4], motionFrame[5], motionFrame[6]); // w,x,y,z
    crl::Matrix3x3 motionRootMatW = matrixFromQuat(motionRootQuatW);

    // Motion joint positions and velocities
    int motionJointStart = 7;
    int motionJointVelStart = 7 + numActions_ + 3 + 3; // after root_pos, root_quat, joint_pos, root_lin_vel, root_ang_vel

    // Add motion root position to observation
    currentObs_[obsIndex++] = motionRootPosW[0];
    currentObs_[obsIndex++] = motionRootPosW[1];
    currentObs_[obsIndex++] = motionRootPosW[2];

    // Add motion root rotation matrix (first 2 rows) to observation
    // Python: motion_root_mat_w[:, :2, :].flatten() takes shape [1, 3, 3] -> [1, 2, 3] (first 2 rows)
    // Flattens in row-major order: [row0col0, row0col1, row0col2, row1col0, row1col1, row1col2]
    for (int row = 0; row < 2; row++) {
        for (int col = 0; col < 3; col++) {
            currentObs_[obsIndex++] = motionRootMatW(row, col);
        }
    }

    // Add motion joint positions (relative to default)
    for (int i = 0; i < numActions_; i++) {
        currentObs_[obsIndex++] = motionFrame[motionJointStart + i] - defaultJointPos_[i];
    }

    // Add motion joint velocities
    for (int i = 0; i < numActions_; i++) {
        currentObs_[obsIndex++] = motionFrame[motionJointVelStart + i];
    }

    // Extract and transform body states to body frame
    // Body state starts at index 71 in processed frame
    // Total bodies in dataset: 30 bodies × 7 values (pos_xyz + quat_wxyz) = 210 values
    int bodyStateStart = 71;

    // First collect all body positions and rotations
    std::vector<crl::V3D> bodyPositions;
    std::vector<crl::Matrix3x3> bodyRotations;

    for (int bodyIdx : featureBodyIds_) {
        // Each body has 7 values: position (3) + quaternion (4)
        int bodyDataStart = bodyStateStart + bodyIdx * 7;

        // Extract body position and quaternion in world frame
        crl::V3D bodyPosW(motionFrame[bodyDataStart + 0],
                          motionFrame[bodyDataStart + 1],
                          motionFrame[bodyDataStart + 2]);
        crl::Quaternion bodyQuatW(motionFrame[bodyDataStart + 3], // w
                                   motionFrame[bodyDataStart + 4], // x
                                   motionFrame[bodyDataStart + 5], // y
                                   motionFrame[bodyDataStart + 6]); // z

        // Transform to body frame
        crl::V3D bodyPosB = quatApplyInverse(motionRootQuatW, bodyPosW - motionRootPosW);
        crl::Quaternion bodyQuatB = quatMul(quatConjugate(motionRootQuatW), bodyQuatW);
        crl::Matrix3x3 bodyMatB = matrixFromQuat(bodyQuatB);

        bodyPositions.push_back(bodyPosB);
        bodyRotations.push_back(bodyMatB);
    }

    // Add all body positions first
    for (const auto& bodyPosB : bodyPositions) {
        currentObs_[obsIndex++] = bodyPosB[0];
        currentObs_[obsIndex++] = bodyPosB[1];
        currentObs_[obsIndex++] = bodyPosB[2];
    }

    // Then add all body rotation matrices (first 2 rows per body)
    for (const auto& bodyMatB : bodyRotations) {
        // Python: motion_body_mat_b[:, :, :2, :].flatten() takes first 2 rows per body
        // Flattens in row-major order: [row0col0, row0col1, row0col2, row1col0, row1col1, row1col2]
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 3; col++) {
                currentObs_[obsIndex++] = bodyMatB(row, col);
            }
        }
    }

    // Check if sequence is complete
    if (currentFrameIndex_ >= motionDataLength_ - 1) {
        if (!sequenceCompleted_) {
            sequenceCompleted_ = true;
            RCLCPP_INFO(logger_, "Deep mimic sequence completed. Ready to transition to WALK state.");
        }

        if (loopSequence_) {
            currentFrameIndex_ = 0;
            sequenceCompleted_ = false;
            RCLCPP_INFO(logger_, "Looping deep mimic sequence...");
        }
    } else {
        currentFrameIndex_++;
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

bool CRLG1DeepTrackController::isSequenceComplete() const {
    return sequenceCompleted_;
}

void CRLG1DeepTrackController::setLoopSequence(bool loop) {
    loopSequence_ = loop;
}

} // namespace crl::g1::mimiccontroller
