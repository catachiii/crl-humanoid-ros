//
// Created by Dongho Kang on 22.01.22.
// Adapted to use MuJoCo instead of ODE
//

#ifndef CRL_HUMANOID_SIMULATOR_G1_NODE
#define CRL_HUMANOID_SIMULATOR_G1_NODE

// crl_humanoid_commons
#include "crl_humanoid_commons/nodes/RobotNode.h"

// ROS2 service messages
#include "crl_humanoid_msgs/srv/elastic_band.hpp"

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
#include <algorithm>

namespace crl::humanoid::simulator {

    template <typename States, typename Machines, std::size_t N>
    class G1SimNode : public crl::humanoid::commons::RobotNode<States, Machines, N> {
        using BaseRobotNode = crl::humanoid::commons::RobotNode<States, Machines, N>;

    public:
        G1SimNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                const std::array<Machines, N>& monitoring,
                const std::atomic<bool>& is_transitioning)
            : BaseRobotNode(model, data, monitoring, is_transitioning) {

            // Parameters
            auto simulationParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
            simulationParamDesc.description = "Simulation parameters";
            simulationParamDesc.read_only = true;
            this->template declare_parameter<std::string>("robot_xml_file", "g1_description/scene_crl.xml", simulationParamDesc);

            // Declare joystick velocity parameters
            this->declare_parameter("joystick_max_forward_velocity", 1.0);
            this->declare_parameter("joystick_max_backward_velocity", 1.0);
            this->declare_parameter("joystick_max_sideways_velocity", 1.0);
            this->declare_parameter("joystick_max_turning_velocity", 1.0);

            joystickMaxForwardVel_ = this->get_parameter("joystick_max_forward_velocity").as_double();
            joystickMaxBackwardVel_ = this->get_parameter("joystick_max_backward_velocity").as_double();
            joystickMaxSidewaysVel_ = this->get_parameter("joystick_max_sideways_velocity").as_double();
            joystickMaxTurningVel_ = this->get_parameter("joystick_max_turning_velocity").as_double();

            // Setup elastic band service
            elasticBandService_ = this->template create_service<crl_humanoid_msgs::srv::ElasticBand>(
                "elastic_band",
                std::bind(&G1SimNode::elasticBandServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
            );

            // Initialize MuJoCo
            initializeMuJoCo();

            // Setup joint mappings
            setupJointMappings();

            // Set default pose after mappings are established
            setDefaultPose();
        }

        /**
         * @brief Enable or disable the elastic rubber band support for the robot.
         * This creates a virtual elastic force that supports the robot's weight,
         * useful for testing walking gaits without the robot falling.
         *
         * @param enable True to enable the elastic band, false to disable
         * @param stiffness Stiffness of the elastic band (default: 500.0 N/m)
         * @param damping Damping coefficient for the elastic band (default: 50.0 Ns/m)
         * @param targetHeight Target height for the robot's pelvis (default: current height)
         */
        void setElasticBandSupport(bool enable, double stiffness = 500.0, double damping = 100.0, double targetHeight = 1.45) {
            std::lock_guard<std::mutex> lock(mujocoMutex_);

            elasticBandEnabled_ = enable;
            elasticBandStiffness_ = stiffness;
            elasticBandDamping_ = damping;
            elasticBandTargetHeight_ = targetHeight;
        }

        /**
         * @brief Get the current state of the elastic band support
         * @return True if elastic band is enabled, false otherwise
         */
        bool isElasticBandEnabled() const {
            return elasticBandEnabled_;
        }

        ~G1SimNode() {
            if (mujocoModel_) {
                mj_deleteModel(mujocoModel_);
            }
            if (mujocoData_) {
                mj_deleteData(mujocoData_);
            }
        }

    protected:
        void resetRobot(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model) override {
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
            crl::humanoid::commons::RobotState groundTruthState;

            // Get base position and orientation from MuJoCo
            // Base position (world frame)
            groundTruthState.basePosition = crl::P3D(mujocoData_->qpos[0],
                                                    mujocoData_->qpos[1],
                                                    mujocoData_->qpos[2]);

            // Base orientation (world frame)
            groundTruthState.baseOrientation = crl::Quaternion(mujocoData_->qpos[3],
                                                                mujocoData_->qpos[4],
                                                                mujocoData_->qpos[5],
                                                                mujocoData_->qpos[6]);

            // Base velocity (world frame) - for floating base, first 3 elements are linear velocity
            groundTruthState.baseVelocity = crl::V3D(mujocoData_->qvel[0],
                                                    mujocoData_->qvel[1],
                                                    mujocoData_->qvel[2]);

            // Base angular velocity (world frame) - for floating base, next 3 elements are angular velocity
            groundTruthState.baseAngularVelocity = crl::V3D(mujocoData_->qvel[3],
                                                            mujocoData_->qvel[4],
                                                            mujocoData_->qvel[5]);

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
            groundTruthState.baseAngularVelocityCov = crl::V3D(0, 0, 0);

            // Update the robot data with ground truth state
            this->data_->setRobotState(groundTruthState);
        }

        void updateDataWithSensorReadings() override {
            std::lock_guard<std::mutex> lock(mujocoMutex_);

            if (!mujocoData_ || !mujocoModel_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MuJoCo data is null");
                return;
            }

            // Apply elastic band support force if enabled
            applyElasticBandForce();

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
                rotMat << mujocoData_->site_xmat[9*imuSiteId], mujocoData_->site_xmat[9*imuSiteId+1], mujocoData_->site_xmat[9*imuSiteId+2],
                         mujocoData_->site_xmat[9*imuSiteId+3], mujocoData_->site_xmat[9*imuSiteId+4], mujocoData_->site_xmat[9*imuSiteId+5],
                         mujocoData_->site_xmat[9*imuSiteId+6], mujocoData_->site_xmat[9*imuSiteId+7], mujocoData_->site_xmat[9*imuSiteId+8];
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
            crl::humanoid::commons::RobotSensor sensorInput;
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

            // Safety check with sensor data, don't trigger emergency stop
            {
                auto state = this->fsm_state_informer.get_first_state();
                for (size_t i = 0; i < q.size() && i < this->jointCount_; ++i) {
                    // Angle limit check
                    if (q[i] > this->jointPosMax_[i]) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint %zu (%s), with current angle %f, breached max angle %f",
                                    i, sensorInput.jointSensors[i].jointName.c_str(), q[i], this->jointPosMax_[i]);
                    }
                    if (q[i] < this->jointPosMin_[i]) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint %zu (%s), with current angle %f, breached min angle %f",
                                    i, sensorInput.jointSensors[i].jointName.c_str(), q[i], this->jointPosMin_[i]);
                    }
                    // Velocity limit check
                    if (std::abs(dq[i]) > this->jointVelMax_[i]) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint %zu (%s), with current velocity %f, breached max velocity %f",
                                    i, sensorInput.jointSensors[i].jointName.c_str(), dq[i], this->jointVelMax_[i]);
                    }
                    // Torque limit check
                    if (std::abs(tau[i]) > this->jointTorqueMax_[i]) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint %zu (%s), with current torque %f, breached max torque %f",
                                    i, sensorInput.jointSensors[i].jointName.c_str(), tau[i], this->jointTorqueMax_[i]);
                    }
                }
            }

            // Update sensor data
            this->data_->setSensor(sensorInput);

            auto command = this->data_->getCommand();
            if (command.targetForwardSpeed > joystickMaxForwardVel_) {
                command.targetForwardSpeed = joystickMaxForwardVel_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Forward speed command limited to max %f m/s", joystickMaxForwardVel_);
            } else if (command.targetForwardSpeed < -joystickMaxBackwardVel_) {
                command.targetForwardSpeed = -joystickMaxBackwardVel_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Backward speed command limited to max %f m/s", joystickMaxBackwardVel_);
            }

            if (command.targetSidewaysSpeed > joystickMaxSidewaysVel_) {
                command.targetSidewaysSpeed = joystickMaxSidewaysVel_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Sideways speed command limited to max %f m/s", joystickMaxSidewaysVel_);
            } else if (command.targetSidewaysSpeed < -joystickMaxSidewaysVel_) {
                command.targetSidewaysSpeed = -joystickMaxSidewaysVel_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Sideways speed command limited to max %f m/s", -joystickMaxSidewaysVel_);
            }

            if (command.targetTurningSpeed > joystickMaxTurningVel_) {
                command.targetTurningSpeed = joystickMaxTurningVel_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Turning speed command limited to max %f rad/s", joystickMaxTurningVel_);
            } else if (command.targetTurningSpeed < -joystickMaxTurningVel_) {
                command.targetTurningSpeed = -joystickMaxTurningVel_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Turning speed command limited to max %f rad/s", -joystickMaxTurningVel_);
            }

            this->data_->setCommand(command);

            // Always use ground truth from MuJoCo simulation until we want to implement state estimation
            overwriteStateEstimator();
        }

        void updateCommandWithData() override {
            std::lock_guard<std::mutex> lock(mujocoMutex_);

            if (!mujocoData_ || !mujocoModel_) {
                return;
            }

            auto control = this->data_->getControlSignal();

            std::vector<double> q, dq, tau;
            getJointStates(q, dq, tau);

            // Apply control signals to MuJoCo
            bool eStop = this->data_->softEStop;


            for (size_t dataIdx = 0; dataIdx < control.jointControl.size() && dataIdx < static_cast<size_t>(mujocoModel_->nu); ++dataIdx) {
                double commandTorque = 0.0;
                double currentPos = (dataIdx < q.size()) ? q[dataIdx] : 0.0;
                double currentVel = (dataIdx < dq.size()) ? dq[dataIdx] : 0.0;

                if (eStop) {
                    // Pure damping in simulation for soft-estop
                    commandTorque = -this->jointDampingDefault_[dataIdx] * currentVel;
                } else {
                    // Use per-joint stiffness and damping if provided, otherwise fall back to global values
                    double jointKp = (control.jointControl[dataIdx].stiffness > 0) ?
                                    control.jointControl[dataIdx].stiffness :
                                    this->jointStiffnessDefault_[dataIdx];
                    double jointKd = (control.jointControl[dataIdx].damping > 0) ?
                                    control.jointControl[dataIdx].damping :
                                    this->jointDampingDefault_[dataIdx];

                    // Normal control mode
                    switch (static_cast<int>(control.jointControl[dataIdx].mode)) {
                        case 1: // Position mode
                            commandTorque = jointKp * (control.jointControl[dataIdx].desiredPos - currentPos) +
                                           jointKd * (0.0 - currentVel);
                            break;
                        case 2: // Velocity mode
                            commandTorque = jointKd * (control.jointControl[dataIdx].desiredSpeed - currentVel);
                            break;
                        case 3: // Force/Torque mode
                            commandTorque = control.jointControl[dataIdx].desiredTorque +
                                           jointKp * (control.jointControl[dataIdx].desiredPos - currentPos) +
                                           jointKd * (control.jointControl[dataIdx].desiredSpeed - currentVel);
                            break;
                        default: // Motor brake (same as soft e-stop)
                            commandTorque = -jointKd * currentVel;
                            break;
                    }

                    // safety check the command Torque, gives a warning, but not switch to estop
                    if (std::abs(commandTorque) > this->jointTorqueMax_[dataIdx]) {
                        RCLCPP_WARN(this->get_logger(), "Joint %zu (%s) torque command exceeds max limit: %f > %f",
                                    dataIdx, control.jointControl[dataIdx].name.c_str(), commandTorque, this->jointTorqueMax_[dataIdx]);
                        commandTorque = std::copysign(this->jointTorqueMax_[dataIdx], commandTorque);
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

    private:
        void initializeMuJoCo() {
            // Load MuJoCo model
            std::string xmlPath = this->get_parameter("robot_xml_file").as_string();
            std::string fullPath = std::string(CRL_HUMANOID_COMMONS_DATA_FOLDER) + "/robots/" + xmlPath;

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

        void applyElasticBandForce() {
            // Always clear all external forces first to prevent accumulation
            if (!mujocoData_ || !mujocoModel_) return;

            // Clear all external forces
            for (int i = 0; i < mujocoModel_->nbody * 6; ++i) {
                mujocoData_->xfrc_applied[i] = 0.0;
            }

            // If elastic band is disabled, just return (forces are already cleared)
            if (!elasticBandEnabled_) return;

            // Find the torso_link body
            int baseBodyId = mj_name2id(mujocoModel_, mjOBJ_BODY, "torso_link");
            if (baseBodyId < 0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Could not find torso_link body for elastic band support");
                return;
            }

            // Get current torso_link position and velocity
            double currentHeight = mujocoData_->xpos[3*baseBodyId + 2]; // Z position

            // Get global linear velocity of torso_link body
            // Use MuJoCo's mj_objectVelocity to get global frame velocity
            mjtNum vel[6]; // [linear_vel_x, linear_vel_y, linear_vel_z, angular_vel_x, angular_vel_y, angular_vel_z]
            mj_objectVelocity(mujocoModel_, mujocoData_, mjOBJ_BODY, baseBodyId, vel, 0);

            // Extract velocity components (global frame)
            double currentVelocityX = vel[0]; // X component of linear velocity
            double currentVelocityY = vel[1]; // Y component of linear velocity
            double currentVelocityZ = vel[2]; // Z component of linear velocity
            double currentAngularVelX = vel[3]; // X component of angular velocity
            double currentAngularVelY = vel[4]; // Y component of angular velocity
            double currentAngularVelZ = vel[5]; // Z component of angular velocity

            // Calculate elastic force: F = -k * (x - x0) - c * v
            // Spring force: pulls robot towards target height
            // Damping force: opposes velocity (if falling, creates upward force)
            double elasticForceZ = -elasticBandStiffness_ * (currentHeight - elasticBandTargetHeight_) - elasticBandDamping_ * currentVelocityZ;

            // Add damping forces for X and Y linear velocities to stabilize horizontal motion
            double dampingForceX = -elasticBandDamping_ * 0.01 * currentVelocityX; // Reduced damping for horizontal
            double dampingForceY = -elasticBandDamping_ * 0.01 * currentVelocityY; // Reduced damping for horizontal

            // Add damping torques for angular velocities to stabilize orientation
            double dampingTorqueX = -elasticBandDamping_ * 0.01 * currentAngularVelX; // Reduced damping for rotation
            double dampingTorqueY = -elasticBandDamping_ * 0.01 * currentAngularVelY; // Reduced damping for rotation
            double dampingTorqueZ = -elasticBandDamping_ * 0.01 * currentAngularVelZ; // Reduced damping for rotation

            // Apply forces and torques to the torso_link body
            // xfrc_applied stores external forces and torques applied to bodies
            // Format: [fx, fy, fz, tx, ty, tz] for each body
            mujocoData_->xfrc_applied[6*baseBodyId+0] = dampingForceX;   // Apply force in X direction
            mujocoData_->xfrc_applied[6*baseBodyId+1] = dampingForceY;   // Apply force in Y direction
            mujocoData_->xfrc_applied[6*baseBodyId+2] = elasticForceZ;   // Apply force in Z direction
            mujocoData_->xfrc_applied[6*baseBodyId+3] = dampingTorqueX;  // Apply torque around X axis
            mujocoData_->xfrc_applied[6*baseBodyId+4] = dampingTorqueY;  // Apply torque around Y axis
            mujocoData_->xfrc_applied[6*baseBodyId+5] = dampingTorqueZ;  // Apply torque around Z axis
        }

        void elasticBandServiceCallback(
            const std::shared_ptr<crl_humanoid_msgs::srv::ElasticBand::Request> request,
            std::shared_ptr<crl_humanoid_msgs::srv::ElasticBand::Response> response) {

            try {
                setElasticBandSupport(request->enable, request->stiffness, request->damping, request->target_height);

                response->success = true;
                if (request->enable) {
                    response->message = "Elastic band support enabled with stiffness=" +
                                      std::to_string(request->stiffness) + " N/m, damping=" +
                                      std::to_string(request->damping) + " Ns/m, target_height=" +
                                      std::to_string(elasticBandTargetHeight_) + " m";
                } else {
                    response->message = "Elastic band support disabled";
                }

                RCLCPP_INFO(this->get_logger(), "Elastic band service call: %s", response->message.c_str());

            } catch (const std::exception& e) {
                response->success = false;
                response->message = "Failed to set elastic band: " + std::string(e.what());
                RCLCPP_ERROR(this->get_logger(), "Elastic band service error: %s", e.what());
            }
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

        // Joystick velocity parameters
        double joystickMaxForwardVel_;
        double joystickMaxBackwardVel_;
        double joystickMaxSidewaysVel_;
        double joystickMaxTurningVel_;

        // Elastic band support for hanging the robot
        bool elasticBandEnabled_ = true;
        double elasticBandStiffness_ = 500.0;  // N/m
        double elasticBandDamping_ = 100.0;     // Ns/m
        double elasticBandTargetHeight_ = 1.45; // m

        // Elastic band service
        rclcpp::Service<crl_humanoid_msgs::srv::ElasticBand>::SharedPtr elasticBandService_;
    };

}  // namespace crl::humanoid::simulator

#endif  //CRL_HUMANOID_SIMULATOR_G1_NODE
