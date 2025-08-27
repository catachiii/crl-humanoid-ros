//
// Created by Dongho Kang on 22.01.22.
// Adapted to use MuJoCo instead of ODE
//

#ifndef CRL_HUMANOID_SIM_NODE
#define CRL_HUMANOID_SIM_NODE

// crl_humanoid_commons
#include "crl_humanoid_commons/nodes/RobotNode.h"

// MuJoCo includes
#include <mujoco/mujoco.h>

// Standard libraries
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <mutex>
#include <atomic>
#include <thread>

namespace crl::unitree::simulator {

    template <typename States, typename Machines, std::size_t N>
    class SimNode : public crl::unitree::commons::RobotNode<States, Machines, N> {
        using BaseRobotNode = crl::unitree::commons::RobotNode<States, Machines, N>;

    public:
        SimNode(const crl::unitree::commons::UnitreeRobotModel& model,
                const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData>& data,
                const std::array<Machines, N>& monitoring,
                const std::atomic<bool>& is_transitioning)
            : BaseRobotNode(model, data, monitoring, is_transitioning) {

            // Parameters
            auto simulationParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
            simulationParamDesc.description = "Simulation parameters";
            simulationParamDesc.read_only = true;
            this->template declare_parameter<std::string>("robot_xml_file", "scene_crl.xml", simulationParamDesc);

            // Initialize MuJoCo
            initializeMuJoCo();

            // Setup joint mappings
            setupJointMappings();

            // Set default pose after mappings are established
            setDefaultPose();
        }

        ~SimNode() {
            if (mujocoModel_) {
                mj_deleteModel(mujocoModel_);
            }
            if (mujocoData_) {
                mj_deleteData(mujocoData_);
            }
        }

    protected:
        void resetRobot(const crl::unitree::commons::UnitreeRobotModel& model) override {
            std::lock_guard<std::mutex> lock(mujocoMutex_);
            if (mujocoData_ && mujocoModel_) {
                mj_resetData(mujocoModel_, mujocoData_);
                setDefaultPose();
                RCLCPP_WARN(this->get_logger(), "Robot reset to default configuration.");
            }
        }

        void overwriteStateEstimator() {
            if (!mujocoData_ || !mujocoModel_) return;

            // Create ground truth state from MuJoCo
            crl::unitree::commons::LeggedRobotState groundTruthState;

            // Get base position and orientation from MuJoCo
            int baseBodyId = mj_name2id(mujocoModel_, mjOBJ_BODY, "pelvis");
            if (baseBodyId >= 0) {
                // Base position (world frame)
                groundTruthState.basePosition = crl::P3D(mujocoData_->xpos[3*baseBodyId],
                                                       mujocoData_->xpos[3*baseBodyId+1],
                                                       mujocoData_->xpos[3*baseBodyId+2]);

                // Base orientation (world frame)
                groundTruthState.baseOrientation = crl::Quaternion(mujocoData_->xquat[4*baseBodyId],
                                                                 mujocoData_->xquat[4*baseBodyId+1],
                                                                 mujocoData_->xquat[4*baseBodyId+2],
                                                                 mujocoData_->xquat[4*baseBodyId+3]);

                // Base velocity (world frame) - for floating base, first 3 elements are linear velocity
                groundTruthState.baseVelocity = crl::V3D(mujocoData_->qvel[0],
                                                        mujocoData_->qvel[1],
                                                        mujocoData_->qvel[2]);

                // Base angular velocity (world frame) - for floating base, next 3 elements are angular velocity
                groundTruthState.baseAngularVelocity = crl::V3D(mujocoData_->qvel[3],
                                                               mujocoData_->qvel[4],
                                                               mujocoData_->qvel[5]);
            }

            // Get joint states from MuJoCo
            std::vector<double> q, dq, tau;
            getJointStates(q, dq, tau);

            // Populate joint states
            groundTruthState.jointStates.resize(q.size());
            for (size_t i = 0; i < q.size(); ++i) {
                groundTruthState.jointStates[i].jointName = (i < jointNames_.size()) ? jointNames_[i] : "joint_" + std::to_string(i);
                groundTruthState.jointStates[i].jointPos = q[i];
                groundTruthState.jointStates[i].jointVel = dq[i];
            }

            // Set covariance to zero for ground truth (perfect knowledge)
            groundTruthState.basePositionCov = crl::V3D(0, 0, 0);
            groundTruthState.baseVelocityCov = crl::V3D(0, 0, 0);
            groundTruthState.baseOrientationCov = crl::V3D(0, 0, 0);

            // Update the robot data with ground truth state
            this->data_->setLeggedRobotState(groundTruthState);
        }

        void updateDataWithSensorReadings() override {
            std::lock_guard<std::mutex> lock(mujocoMutex_);

            if (!mujocoData_ || !mujocoModel_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MuJoCo data is null");
                return;
            }

            // Step the simulation
            mj_step(mujocoModel_, mujocoData_);

            // Get IMU data from base body (pelvis)
            crl::V3D accelerometer = crl::V3D(0, 0, 0);
            crl::V3D gyro = crl::V3D(0, 0, 0);
            crl::Quaternion imuQuat = crl::Quaternion::Identity();

            // Get IMU orientation from the pelvis IMU site
            int imuSiteId = mj_name2id(mujocoModel_, mjOBJ_SITE, "imu_in_pelvis");
            if (imuSiteId >= 0) {
                // Get IMU site orientation from rotation matrix and convert to quaternion
                // site_xmat is a 3x3 rotation matrix stored in column-major order (9 consecutive values)
                crl::Matrix3x3 rotMat;
                // MuJoCo stores rotation matrix in column-major order: [col0, col1, col2]
                // Eigen expects row-major order, so we need to transpose
                rotMat << mujocoData_->site_xmat[9*imuSiteId], mujocoData_->site_xmat[9*imuSiteId+3], mujocoData_->site_xmat[9*imuSiteId+6],
                         mujocoData_->site_xmat[9*imuSiteId+1], mujocoData_->site_xmat[9*imuSiteId+4], mujocoData_->site_xmat[9*imuSiteId+7],
                         mujocoData_->site_xmat[9*imuSiteId+2], mujocoData_->site_xmat[9*imuSiteId+5], mujocoData_->site_xmat[9*imuSiteId+8];
                imuQuat = crl::Quaternion(rotMat);
            } else {
                // Fallback to pelvis body orientation if IMU site not found
                int baseBodyId = mj_name2id(mujocoModel_, mjOBJ_BODY, "pelvis");
                if (baseBodyId >= 0) {
                    imuQuat = crl::Quaternion(mujocoData_->xquat[4*baseBodyId],
                                            mujocoData_->xquat[4*baseBodyId+1],
                                            mujocoData_->xquat[4*baseBodyId+2],
                                            mujocoData_->xquat[4*baseBodyId+3]);
                }
            }

            // Get sensor data by name using MuJoCo API
            int gyroId = mj_name2id(mujocoModel_, mjOBJ_SENSOR, "imu-pelvis-angular-velocity");
            int accelId = mj_name2id(mujocoModel_, mjOBJ_SENSOR, "imu-pelvis-linear-acceleration");

            if (gyroId >= 0) {
                // Gyroscope: get from MuJoCo pelvis sensor (already in body frame)
                gyro = crl::V3D(mujocoData_->sensordata[3*gyroId],
                               mujocoData_->sensordata[3*gyroId+1],
                               mujocoData_->sensordata[3*gyroId+2]);
            }

            if (accelId >= 0) {
                // Accelerometer: get from MuJoCo pelvis sensor (already in body frame)
                accelerometer = crl::V3D(mujocoData_->sensordata[3*accelId],
                                        mujocoData_->sensordata[3*accelId+1],
                                        mujocoData_->sensordata[3*accelId+2]);
            }

            // Get joint encoder measurements
            std::vector<double> q, dq, tau;
            getJointStates(q, dq, tau);

            // Create sensor data structure
            crl::unitree::commons::LeggedRobotSensor sensorInput;
            sensorInput.accelerometer = accelerometer;
            sensorInput.gyroscope = gyro;
            sensorInput.imuOrientation = imuQuat;

            // Populate joint sensor data
            sensorInput.jointSensors.resize(q.size());
            for (size_t i = 0; i < q.size(); ++i) {
                sensorInput.jointSensors[i].jointName = (i < jointNames_.size()) ? jointNames_[i] : "joint_" + std::to_string(i);
                sensorInput.jointSensors[i].jointPos = q[i];
                sensorInput.jointSensors[i].jointVel = dq[i];
                sensorInput.jointSensors[i].jointTorque = tau[i]; // Get torque from MuJoCo simulation
            }

            // Safety check with sensor data
            bool jointLimit = false;
            {
                // Check joint angle / velocity violation only in WALK mode
                auto state = this->fsm_state_informer.get_first_state();
                if (state == States::WALK) {
                    for (size_t i = 0; i < q.size() && i < this->JOINT_POSITION_MAX.size(); ++i) {
                        // Angle limit check
                        if (q[i] > this->JOINT_POSITION_MAX[i]) {
                            jointLimit = true;
                            RCLCPP_WARN(this->get_logger(), "Joint %zu, with current angle %f, breached max angle %f",
                                       i, q[i], this->JOINT_POSITION_MAX[i]);
                        }
                        if (q[i] < this->JOINT_POSITION_MIN[i]) {
                            jointLimit = true;
                            RCLCPP_WARN(this->get_logger(), "Joint %zu, with current angle %f, breached min angle %f",
                                       i, q[i], this->JOINT_POSITION_MIN[i]);
                        }
                        // Velocity limit check
                        if (std::abs(dq[i]) > this->JOINT_VELOCITY_MAX[i]) {
                            jointLimit = true;
                            RCLCPP_WARN(this->get_logger(), "Joint %zu, with current velocity %f, breached max velocity %f",
                                       i, dq[i], this->JOINT_VELOCITY_MAX[i]);
                        }
                    }
                }
            }

            // Trigger estop if safety check failed
            if (jointLimit && !this->data_->softEStop) {
                RCLCPP_WARN(this->get_logger(), "Joint limit breached, switching to ESTOP");
                this->fsm_broadcaster.broadcast_switch(States::ESTOP);
            }

            // Update sensor data
            this->data_->setSensor(sensorInput);

            // Always use ground truth from MuJoCo simulation until we want to implement state estimation
            overwriteStateEstimator();
        }

        void updateCommandWithData() override {
            std::lock_guard<std::mutex> lock(mujocoMutex_);

            if (!mujocoData_ || !mujocoModel_) {
                return;
            }

            auto control = this->data_->getControlSignal();

            // Safety check
            bool torqueLimit = false;
            std::vector<double> q, dq, tau;
            getJointStates(q, dq, tau);

            for (size_t i = 0; i < control.jointControl.size() && i < this->JOINT_TORQUE_MAX.size(); ++i) {
                double torque = control.jointControl[i].desiredTorque;

                // Torque limit check
                if (std::abs(torque) > this->JOINT_TORQUE_MAX[i]) {
                    torqueLimit = true;
                    RCLCPP_WARN(this->get_logger(), "Joint %zu, with current torque %f, breached max torque %f",
                               i, torque, this->JOINT_TORQUE_MAX[i]);
                }
            }

            // Trigger estop if safety check failed
            if (torqueLimit && !this->data_->softEStop) {
                RCLCPP_WARN(this->get_logger(), "Torque limit breached, switching to ESTOP.");
                this->fsm_broadcaster.broadcast_switch(States::ESTOP);
            }

            // Apply control signals to MuJoCo
            bool eStop = this->data_->softEStop;

            for (size_t dataIdx = 0; dataIdx < control.jointControl.size() && dataIdx < static_cast<size_t>(mujocoModel_->nu); ++dataIdx) {
                double commandTorque = 0.0;
                double currentPos = (dataIdx < q.size()) ? q[dataIdx] : 0.0;
                double currentVel = (dataIdx < dq.size()) ? dq[dataIdx] : 0.0;

                if (eStop) {
                    // Pure damping in simulation for soft-estop
                    commandTorque = -this->JOINT_POSITION_CONTROL_KD[dataIdx] * currentVel;
                } else {
                    // Normal control mode
                    switch (static_cast<int>(control.jointControl[dataIdx].mode)) {
                        case 1: // Position mode
                            commandTorque = this->JOINT_POSITION_CONTROL_KP[dataIdx] * (control.jointControl[dataIdx].desiredPos - currentPos) +
                                           this->JOINT_POSITION_CONTROL_KD[dataIdx] * (0.0 - currentVel);
                            break;
                        case 2: // Velocity mode
                            commandTorque = this->JOINT_POSITION_CONTROL_KD[dataIdx] * (control.jointControl[dataIdx].desiredSpeed - currentVel);
                            break;
                        case 3: // Force/Torque mode
                            commandTorque = control.jointControl[dataIdx].desiredTorque +
                                           this->JOINT_TORQUE_CONTROL_KP[dataIdx] * (control.jointControl[dataIdx].desiredPos - currentPos) +
                                           this->JOINT_TORQUE_CONTROL_KD[dataIdx] * (control.jointControl[dataIdx].desiredSpeed - currentVel);
                            break;
                        default: // Motor brake (same as soft e-stop)
                            commandTorque = -this->JOINT_POSITION_CONTROL_KD[dataIdx] * currentVel;
                            break;
                    }
                }

                // Apply the torque command using actuator address
                // Map from canonical data index to MuJoCo control index
                if (dataIdx < dataToMujocoMapping_.size()) {
                    size_t mujocoIdx = dataToMujocoMapping_[dataIdx];

                    // Check for valid mapping (not SIZE_MAX) and bounds check for MuJoCo control array
                    if (mujocoIdx != SIZE_MAX && mujocoIdx < static_cast<size_t>(mujocoModel_->nu)) {
                        mujocoData_->ctrl[mujocoIdx] = commandTorque;
                    } else {
                        if (mujocoIdx == SIZE_MAX) {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                 "No mapping found for data index %zu", dataIdx);
                        } else {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                 "Invalid MuJoCo control index %zu for data index %zu", mujocoIdx, dataIdx);
                        }
                    }
                } else {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Data index %zu out of bounds for mapping array size %zu", dataIdx, dataToMujocoMapping_.size());
                }
            }
        }

        void applyRobotParameters() override {
            BaseRobotNode::applyRobotParameters();
            // MuJoCo handles all physics parameters internally
        }

    private:
        void initializeMuJoCo() {
            // Load MuJoCo model
            std::string xmlPath = this->get_parameter("robot_xml_file").as_string();
            std::string fullPath = std::string(CRL_HUMANOID_COMMONS_DATA_FOLDER) + "/robots/g1_description/" + xmlPath;

            char error[1000];
            mujocoModel_ = mj_loadXML(fullPath.c_str(), nullptr, error, sizeof(error));

            if (!mujocoModel_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model: %s", error);
                return;
            }

            // Create MuJoCo data
            mujocoData_ = mj_makeData(mujocoModel_);
            if (!mujocoData_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create MuJoCo data");
                return;
            }

            // Set simulation parameters
            mujocoModel_->opt.timestep = this->timeStepSize_;
        }

        void setDefaultPose() {
            if (!mujocoData_ || !mujocoModel_) return;

            // Set base position and orientation
            mujocoData_->qpos[0] = 0.0; // x position
            mujocoData_->qpos[1] = 0.0; // y position
            mujocoData_->qpos[2] = 0.79; // z position (height)
            mujocoData_->qpos[3] = 1.0; // w (quaternion)
            mujocoData_->qpos[4] = 0.0; // x (quaternion)
            mujocoData_->qpos[5] = 0.0; // y (quaternion)
            mujocoData_->qpos[6] = 0.0; // z (quaternion)

            // Set ALL joint positions to zero (default standing pose)
            // This should override any default configuration that might not be zeros
            for (size_t canonicalIdx = 0; canonicalIdx < jointNames_.size(); ++canonicalIdx) {
                if (canonicalIdx < dataToMujocoMapping_.size()) {
                    size_t mujocoIdx = dataToMujocoMapping_[canonicalIdx];
                    if (mujocoIdx != SIZE_MAX) {
                        // Find the joint ID and set its position to zero
                        std::vector<std::string> mujocoJointOrder = {
                            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
                            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
                            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
                            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
                        };

                        if (mujocoIdx < mujocoJointOrder.size()) {
                            int jointId = getJointId(mujocoJointOrder[mujocoIdx]);
                            if (jointId >= 0) {
                                int qposIndex = mujocoModel_->jnt_qposadr[jointId];
                                if (qposIndex >= 0 && qposIndex < mujocoModel_->nq) {
                                    // Force all joints to zero position for symmetric standing
                                    mujocoData_->qpos[qposIndex] = 0.0;
                                }
                            }
                        }
                    }
                }
            }

            // Also zero out all velocities
            for (int i = 0; i < mujocoModel_->nv; ++i) {
                mujocoData_->qvel[i] = 0.0;
            }

            mj_forward(mujocoModel_, mujocoData_);
        }

        void setupJointMappings() {
            jointNames_.clear();
            mujocoToDataMapping_.clear();
            dataToMujocoMapping_.clear();

            // Get the canonical joint order from data control signal
            auto control = this->data_->getControlSignal();
            std::vector<std::string> canonicalJointNames;
            canonicalJointNames.reserve(control.jointControl.size());
            for (const auto& jointControl : control.jointControl) {
                canonicalJointNames.push_back(jointControl.name);
            }

            // MuJoCo joint order (as defined in the XML)
            std::vector<std::string> mujocoJointOrder = {
                // Left leg (6 DOF)
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                // Right leg (6 DOF)
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                // Waist (3 DOF)
                "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
                // Left arm (7 DOF)
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
                "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
                // Right arm (7 DOF)
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
            };

            // Store canonical order for internal use (this is what the controller expects)
            jointNames_ = canonicalJointNames;

            // Create mapping between canonical data order and MuJoCo XML order
            mujocoToDataMapping_.resize(mujocoJointOrder.size());
            dataToMujocoMapping_.resize(canonicalJointNames.size());

            // Initialize mappings with invalid values to detect unset mappings
            std::fill(mujocoToDataMapping_.begin(), mujocoToDataMapping_.end(), SIZE_MAX);
            std::fill(dataToMujocoMapping_.begin(), dataToMujocoMapping_.end(), SIZE_MAX);

            // Create mapping: canonical (data) order -> MuJoCo XML order
            for (size_t dataIdx = 0; dataIdx < canonicalJointNames.size(); ++dataIdx) {
                const std::string& canonicalJointName = canonicalJointNames[dataIdx];

                // Find this joint in the MuJoCo XML order
                auto it = std::find(mujocoJointOrder.begin(), mujocoJointOrder.end(), canonicalJointName);
                if (it != mujocoJointOrder.end()) {
                    size_t mujocoIdx = std::distance(mujocoJointOrder.begin(), it);

                    // Bounds check before setting mapping
                    if (mujocoIdx < mujocoToDataMapping_.size()) {
                        mujocoToDataMapping_[mujocoIdx] = dataIdx;
                        dataToMujocoMapping_[dataIdx] = mujocoIdx;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "MuJoCo index %zu out of bounds for mapping array size %zu",
                                   mujocoIdx, mujocoToDataMapping_.size());
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Joint '%s' found in canonical order but not in MuJoCo XML order!",
                               canonicalJointName.c_str());
                }
            }

            // Validation: Check that all canonical joints were mapped
            for (size_t dataIdx = 0; dataIdx < dataToMujocoMapping_.size(); ++dataIdx) {
                if (dataToMujocoMapping_[dataIdx] == SIZE_MAX) {
                    RCLCPP_ERROR(this->get_logger(), "Canonical index %zu was not mapped to any MuJoCo joint", dataIdx);
                } else if (dataToMujocoMapping_[dataIdx] >= mujocoJointOrder.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mapping: Canonical index %zu maps to invalid MuJoCo index %zu",
                               dataIdx, dataToMujocoMapping_[dataIdx]);
                }
            }

            // Validation: Check that all MuJoCo joints were mapped
            for (size_t mujocoIdx = 0; mujocoIdx < mujocoToDataMapping_.size(); ++mujocoIdx) {
                if (mujocoToDataMapping_[mujocoIdx] == SIZE_MAX) {
                    RCLCPP_WARN(this->get_logger(), "MuJoCo index %zu was not mapped to any canonical joint", mujocoIdx);
                } else if (mujocoToDataMapping_[mujocoIdx] >= canonicalJointNames.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mapping: MuJoCo index %zu maps to invalid canonical index %zu",
                               mujocoIdx, mujocoToDataMapping_[mujocoIdx]);
                }
            }
        }

        int getJointId(const std::string& jointName) {
            if (!mujocoModel_) return -1;
            return mj_name2id(mujocoModel_, mjOBJ_JOINT, jointName.c_str());
        }

        void getJointStates(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& torques) {
            positions.clear();
            velocities.clear();
            torques.clear();

            if (!mujocoData_ || !mujocoModel_) return;

            // Use the canonical joint count for output arrays
            size_t numCanonicalJoints = jointNames_.size(); // jointNames_ now stores canonical order

            positions.resize(numCanonicalJoints);
            velocities.resize(numCanonicalJoints);
            torques.resize(numCanonicalJoints);

            // MuJoCo joint order (XML order)
            std::vector<std::string> mujocoJointOrder = {
                // Left leg (6 DOF)
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                // Right leg (6 DOF)
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                // Waist (3 DOF)
                "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
                // Left arm (7 DOF)
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
                "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
                // Right arm (7 DOF)
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
            };

            // Get data in MuJoCo order first
            std::vector<double> mujocoPositions(mujocoJointOrder.size());
            std::vector<double> mujocoVelocities(mujocoJointOrder.size());
            std::vector<double> mujocoTorques(mujocoJointOrder.size());

            // Extract joint states from MuJoCo in MuJoCo XML order
            for (size_t mujocoIdx = 0; mujocoIdx < mujocoJointOrder.size(); ++mujocoIdx) {
                int jointId = getJointId(mujocoJointOrder[mujocoIdx]);
                if (jointId >= 0) {
                    // Get the index in the state vector for this joint
                    int qposIndex = mujocoModel_->jnt_qposadr[jointId];
                    int qvelIndex = mujocoModel_->jnt_dofadr[jointId];

                    if (qposIndex >= 0 && qposIndex < mujocoModel_->nq) {
                        mujocoPositions[mujocoIdx] = mujocoData_->qpos[qposIndex];
                    } else {
                        mujocoPositions[mujocoIdx] = 0.0;
                    }

                    if (qvelIndex >= 0 && qvelIndex < mujocoModel_->nv) {
                        mujocoVelocities[mujocoIdx] = mujocoData_->qvel[qvelIndex];
                    } else {
                        mujocoVelocities[mujocoIdx] = 0.0;
                    }

                    // Get joint torque from MuJoCo
                    // qfrc_actuator contains actuator forces/torques
                    if (qvelIndex >= 0 && qvelIndex < mujocoModel_->nv) {
                        mujocoTorques[mujocoIdx] = mujocoData_->qfrc_actuator[qvelIndex];
                    } else {
                        mujocoTorques[mujocoIdx] = 0.0;
                    }
                } else {
                    mujocoPositions[mujocoIdx] = 0.0;
                    mujocoVelocities[mujocoIdx] = 0.0;
                    mujocoTorques[mujocoIdx] = 0.0;
                    RCLCPP_WARN(this->get_logger(), "Joint not found: %s", mujocoJointOrder[mujocoIdx].c_str());
                }
            }

            // Now map from MuJoCo order to canonical order
            for (size_t canonicalIdx = 0; canonicalIdx < numCanonicalJoints; ++canonicalIdx) {
                // Find mapping from canonical to MuJoCo
                if (canonicalIdx < dataToMujocoMapping_.size()) {
                    size_t mujocoIdx = dataToMujocoMapping_[canonicalIdx];

                    // Check for valid mapping (not SIZE_MAX) and bounds check for MuJoCo arrays
                    if (mujocoIdx != SIZE_MAX && mujocoIdx < mujocoPositions.size()) {
                        positions[canonicalIdx] = mujocoPositions[mujocoIdx];
                        velocities[canonicalIdx] = mujocoVelocities[mujocoIdx];
                        torques[canonicalIdx] = mujocoTorques[mujocoIdx];
                    } else {
                        if (mujocoIdx == SIZE_MAX) {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                 "No mapping found for canonical index %zu", canonicalIdx);
                        } else {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                 "Invalid MuJoCo index %zu for canonical index %zu", mujocoIdx, canonicalIdx);
                        }
                        positions[canonicalIdx] = 0.0;
                        velocities[canonicalIdx] = 0.0;
                        torques[canonicalIdx] = 0.0;
                    }
                } else {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Canonical index %zu out of bounds for mapping array size %zu", canonicalIdx, dataToMujocoMapping_.size());
                    positions[canonicalIdx] = 0.0;
                    velocities[canonicalIdx] = 0.0;
                    torques[canonicalIdx] = 0.0;
                }
            }
        }

    private:
        // MuJoCo objects
        mjModel* mujocoModel_ = nullptr;
        mjData* mujocoData_ = nullptr;

        // Synchronization
        std::mutex mujocoMutex_;

        // Joint mappings
        std::vector<std::string> jointNames_; // Canonical order (policy order)
        std::vector<size_t> mujocoToDataMapping_; // Maps MuJoCo XML index to canonical index
        std::vector<size_t> dataToMujocoMapping_; // Maps canonical index to MuJoCo XML index
    };

}  // namespace crl::unitree::simulator

#endif  //CRL_HUMANOID_SIM_NODE
