#ifndef CRL_HUMANOID_HARDWARE_WF_TRON1A_NODE
#define CRL_HUMANOID_HARDWARE_WF_TRON1A_NODE
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <map>
#include <cstdint>

// crl_humanoid_commons
#include "crl_humanoid_commons/nodes/RobotNode.h"

// limx_sdk - use PointFoot for WheelFoot (TRON1A) robots
#include <limxsdk/pointfoot.h>
#include <limxsdk/datatypes.h>

namespace crl::unitree::hardware::wf_tron1a {

    // Number of motors in WF_TRON1A
    const int WF_TRON1A_NUM_MOTOR = 8;

    template <typename States, typename Machines, std::size_t N>
    class Tron1aNode : public crl::humanoid::commons::RobotNode<States, Machines, N> {
        using BaseRobotNode = crl::humanoid::commons::RobotNode<States, Machines, N>;
        using StateNameToEnumMap = std::unordered_map<std::string, States>;

    public:
        // Structure to hold state-key mappings
        struct StateKeyBinding {
            std::string stateName;
            std::string keyName;
            uint32_t keyConstant;
            States stateEnum;
        };

        // Helper function to create state name mappings
        static StateNameToEnumMap createStateMapping(
            const std::vector<std::pair<std::string, States>>& mappings) {
            StateNameToEnumMap result;
            for (const auto& mapping : mappings) {
                result[mapping.first] = mapping.second;
            }
            return result;
        }

        Tron1aNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
               const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
               const std::array<Machines, N>& monitoring,
               const std::atomic<bool>& is_transitioning,
               const StateNameToEnumMap& stateNameMap = {})
            : BaseRobotNode(model, data, monitoring, is_transitioning),
              isSDKInitialized_(false),
              robot_(nullptr),
              stateNameToEnumMap_(stateNameMap) {

            // Initialize default state mappings if none provided
            if (stateNameToEnumMap_.empty()) {
                initializeDefaultStateMappings();
            }

            // Declare and get robot IP address parameter
            this->declare_parameter("robot_ip_address", "10.192.1.2");  // Default: localhost for simulation
            robotIpAddress_ = this->get_parameter("robot_ip_address").as_string();

            // Declare joystick velocity parameters
            this->declare_parameter("joystick_max_forward_velocity", 1.0);
            this->declare_parameter("joystick_max_backward_velocity", 1.0);
            this->declare_parameter("joystick_max_sideways_velocity", 1.0);
            this->declare_parameter("joystick_max_turning_velocity", 1.0);

            joystickMaxForwardVel_ = this->get_parameter("joystick_max_forward_velocity").as_double();
            joystickMaxBackwardVel_ = this->get_parameter("joystick_max_backward_velocity").as_double();
            joystickMaxSidewaysVel_ = this->get_parameter("joystick_max_sideways_velocity").as_double();
            joystickMaxTurningVel_ = this->get_parameter("joystick_max_turning_velocity").as_double();

            // Declare keybinding parameters
            this->declare_parameter("keybinding_modifier", "L1");  // L1, L2, R1, R2

            // Declare state keybindings as lists
            this->declare_parameter("state_keybindings.states", std::vector<std::string>{"ESTOP", "STAND", "WALK"});
            this->declare_parameter("state_keybindings.keys", std::vector<std::string>{"Circle", "Square", "Triangle"});

            // Get keybinding parameters
            std::string modifier = this->get_parameter("keybinding_modifier").as_string();
            modifierButton_ = stringToButtonConstant(modifier);

            // Setup state keybindings
            setupStateKeybindings();

            // Setup joint mappings
            setupJointMappings();

            // Initialize SDK (this starts the communication)
            robot_ = limxsdk::PointFoot::getInstance();  // Use PointFoot for WheelFoot
            initializeSDK();

            // initialize buffers
            initializeBuffers();

            // Setup subscriptions (after SDK init, like in the examples)
            setupSubscriptions();
        }

    private:
        // Convert string to button constant
        uint32_t stringToButtonConstant(const std::string& buttonName) {
            if (buttonName == "Cross") return 1;          // Index 0
            else if (buttonName == "Circle") return 2;    // Index 1
            else if (buttonName == "Square") return 4;    // Index 2
            else if (buttonName == "Triangle") return 8;  // Index 3
            else if (buttonName == "L1") return 16;       // Index 4
            else if (buttonName == "R2") return 32;       // Index 5
            else if (buttonName == "L2") return 64;       // Index 6
            else if (buttonName == "R1") return 128;      // Index 7
            else if (buttonName == "Select") return 256;  // Index 8
            else if (buttonName == "Start") return 512;   // Index 9
            else if (buttonName == "Up") return 4096;     // Index 12
            else if (buttonName == "Down") return 8192;   // Index 13
            else if (buttonName == "Left") return 16384;  // Index 14
            else if (buttonName == "Right") return 32768; // Index 15
            else if (buttonName == "Menu") return 65536;  // Index 16
            else if (buttonName == "Back") return 131072; // Index 17
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown button name: %s, defaulting to Cross", buttonName.c_str());
                return 1; // Default to Cross
            }
        }

        // Convert string to state enum using configurable mapping
        States stringToStateEnum(const std::string& stateName) {
            auto it = stateNameToEnumMap_.find(stateName);
            if (it != stateNameToEnumMap_.end()) {
                return it->second;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown state name: %s. Available states are not properly configured!", stateName.c_str());
                // Log available states for debugging
                std::string availableStates;
                for (const auto& pair : stateNameToEnumMap_) {
                    if (!availableStates.empty()) availableStates += ", ";
                    availableStates += pair.first;
                }
                RCLCPP_ERROR(this->get_logger(), "Available states: %s", availableStates.c_str());
                throw std::runtime_error("Invalid state name: " + stateName);
            }
        }

        // Initialize default state mappings
        void initializeDefaultStateMappings() {
            // This will be populated with the basic states that are always available
            // Users can override this by providing a custom map in the constructor
            RCLCPP_INFO(this->get_logger(), "No explicit state mappings provided. State transitions will need to be configured via the state mapping.");
        }

        void setupStateKeybindings() {
            try {
                auto stateNames = this->get_parameter("state_keybindings.states").as_string_array();
                auto keyNames = this->get_parameter("state_keybindings.keys").as_string_array();

                if (stateNames.size() != keyNames.size()) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Mismatch between state_keybindings.states (%zu) and state_keybindings.keys (%zu) array sizes",
                        stateNames.size(), keyNames.size());
                    // Fall back to default bindings
                    setupDefaultKeybindings();
                    return;
                }

                stateKeybindings_.clear();
                stateKeybindings_.reserve(stateNames.size());

                for (size_t i = 0; i < stateNames.size(); ++i) {
                    StateKeyBinding binding;
                    binding.stateName = stateNames[i];
                    binding.keyName = keyNames[i];
                    binding.keyConstant = stringToButtonConstant(keyNames[i]);
                    binding.stateEnum = stringToStateEnum(stateNames[i]);

                    stateKeybindings_.push_back(binding);

                    RCLCPP_INFO(this->get_logger(), "Keybinding: %s -> %s (modifier + %s)",
                        binding.stateName.c_str(), binding.keyName.c_str(), binding.keyName.c_str());
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to setup state keybindings: %s. Using defaults.", e.what());
                setupDefaultKeybindings();
            }
        }

        void setupDefaultKeybindings() {
            stateKeybindings_.clear();

            // Only set up default keybindings if we have state mappings available
            if (!stateNameToEnumMap_.empty()) {
                auto estopIt = stateNameToEnumMap_.find("ESTOP");
                auto standIt = stateNameToEnumMap_.find("STAND");
                auto walkIt = stateNameToEnumMap_.find("WALK");

                if (estopIt != stateNameToEnumMap_.end()) {
                    stateKeybindings_.push_back({"ESTOP", "Circle", stringToButtonConstant("Circle"), estopIt->second});
                }
                if (standIt != stateNameToEnumMap_.end()) {
                    stateKeybindings_.push_back({"STAND", "Square", stringToButtonConstant("Square"), standIt->second});
                }
                if (walkIt != stateNameToEnumMap_.end()) {
                    stateKeybindings_.push_back({"WALK", "Triangle", stringToButtonConstant("Triangle"), walkIt->second});
                }
            }

            RCLCPP_INFO(this->get_logger(), "Using default keybindings: ESTOP=Circle, STAND=Square, WALK=Triangle");
        }

        void initializeSDK() {
            try {
                if (!robot_) {
                    RCLCPP_ERROR(this->get_logger(), "Robot instance is null");
                    throw std::runtime_error("Robot instance is null");
                }

                isSDKInitialized_ = robot_->init(robotIpAddress_);
                if (isSDKInitialized_) {
                    RCLCPP_INFO(this->get_logger(), "LIMX SDK initialized successfully with IP: %s", robotIpAddress_.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "LIMX SDK initialization failed with IP: %s", robotIpAddress_.c_str());
                    throw std::runtime_error("Failed to initialize LIMX SDK");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize LIMX SDK: %s", e.what());
                throw;
            }
        }

        void setupSubscriptions() {
            robot_->subscribeRobotState([this](const limxsdk::RobotStateConstPtr& msg) {
                {
                    std::unique_lock<std::shared_mutex> lock(sdk_data_mutex_);
                    robot_state_ = *msg;
                }
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                //                     "Received robot state update with timestamp: %lu, motor count: %zu, q[0]=%.3f",
                //                     msg->stamp, msg->q.size(), msg->q.size() > 0 ? msg->q[0] : 0.0);
            });

            // Subscribe to IMU data
            robot_->subscribeImuData([this](const limxsdk::ImuDataConstPtr& msg) {
                {
                    std::unique_lock<std::shared_mutex> lock(sdk_data_mutex_);
                    imu_data_ = *msg;
                }
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                //                     "Received IMU data - acc: [%.2f, %.2f, %.2f]",
                //                     msg->acc[0], msg->acc[1], msg->acc[2]);
            });

            // Subscribe to joystick
            robot_->subscribeSensorJoy([this](const limxsdk::SensorJoyConstPtr& msg) {
                {
                    std::unique_lock<std::shared_mutex> lock(sdk_data_mutex_);
                    sensor_joy_ = *msg;
                }
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                //                     "Received SensorJoy data - axes: %zu, buttons: %zu",
                //                     msg->axes.size(), msg->buttons.size());
            });
        }

        void initializeBuffers() {
            robot_state_ = limxsdk::RobotState(robot_->getMotorNumber());
            robot_cmd_ = limxsdk::RobotCmd(robot_->getMotorNumber());
            RCLCPP_INFO(this->get_logger(), "Buffers initialized with %d motors", robot_->getMotorNumber());
        }

        void setupJointMappings() {
            CanonicalJointNames_.clear();
            hardwareToDataMapping_.clear();
            dataToHardwareMapping_.clear();

            // Get the canonical joint order from data control signal
            auto control = this->data_->getControlSignal();
            CanonicalJointNames_.reserve(control.jointControl.size());
            for (const auto& jointControl : control.jointControl) {
                CanonicalJointNames_.push_back(jointControl.name);
            }

            // LimX SDK motor order mapping for WF_TRON1A:
            // 0: abad_L_Joint, 1: hip_L_Joint, 2: knee_L_Joint, 3: wheel_L_Joint
            // 4: abad_R_Joint, 5: hip_R_Joint, 6: knee_R_Joint, 7: wheel_R_Joint
            std::map<std::string, size_t> canonicalToLimxMotorIndex = {
                {"abad_L_Joint", 0},
                {"hip_L_Joint", 1},
                {"knee_L_Joint", 2},
                {"wheel_L_Joint", 3},
                {"abad_R_Joint", 4},
                {"hip_R_Joint", 5},
                {"knee_R_Joint", 6},
                {"wheel_R_Joint", 7}
            };

            // Initialize mappings with invalid values
            dataToHardwareMapping_.resize(CanonicalJointNames_.size(), SIZE_MAX);
            // LimX SDK has 16 motors total
            hardwareToDataMapping_.resize(16, SIZE_MAX);

            // Create mapping: canonical (data) order -> LimX SDK motor index
            for (size_t dataIdx = 0; dataIdx < CanonicalJointNames_.size(); ++dataIdx) {
                const std::string& canonicalJointName = CanonicalJointNames_[dataIdx];
                auto it = canonicalToLimxMotorIndex.find(canonicalJointName);
                if (it != canonicalToLimxMotorIndex.end()) {
                    size_t limxMotorIdx = it->second;
                    dataToHardwareMapping_[dataIdx] = limxMotorIdx;
                    hardwareToDataMapping_[limxMotorIdx] = dataIdx;
                    RCLCPP_INFO(this->get_logger(), "Mapped canonical joint '%s' (idx %zu) -> LimX motor index %zu",
                               canonicalJointName.c_str(), dataIdx, limxMotorIdx);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Joint '%s' found in canonical order but not in LimX SDK mapping!", canonicalJointName.c_str());
                }
            }

            // Validation: Check that all canonical joints were mapped
            for (size_t dataIdx = 0; dataIdx < dataToHardwareMapping_.size(); ++dataIdx) {
                if (dataToHardwareMapping_[dataIdx] == SIZE_MAX) {
                    RCLCPP_ERROR(this->get_logger(), "Canonical index %zu (%s) was not mapped to any LimX motor index",
                               dataIdx, dataIdx < CanonicalJointNames_.size() ? CanonicalJointNames_[dataIdx].c_str() : "unknown");
                } else if (dataToHardwareMapping_[dataIdx] >= 16) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mapping: Canonical index %zu maps to invalid LimX motor index %zu",
                               dataIdx, dataToHardwareMapping_[dataIdx]);
                }
            }
        }

        void updateDataWithSensorReadings() override {
            // Create sensor data structure for humanoid robot
            auto sensorInput = this->data_->getSensor();
            auto robotState = this->data_->getRobotState();

            // Thread-safe access to LimX SDK data with shared lock for reading
            limxsdk::RobotState currentRobotState;
            limxsdk::ImuData currentImu;
            limxsdk::SensorJoy currentJoy;
            {
                std::shared_lock<std::shared_mutex> lock(sdk_data_mutex_);
                currentRobotState = robot_state_;
                currentImu = imu_data_;
                currentJoy = sensor_joy_;
            }

            // Populate IMU data from LimX SDK
            sensorInput.accelerometer = crl::V3D(
                currentImu.acc[0],
                currentImu.acc[1],
                currentImu.acc[2]
            );
            sensorInput.gyroscope = crl::V3D(
                currentImu.gyro[0],
                currentImu.gyro[1],
                currentImu.gyro[2]
            );
            // LimX SDK quaternion order: [w, x, y, z]
            sensorInput.imuOrientation = crl::Quaternion(
                currentImu.quat[0],  // w
                currentImu.quat[1],  // x
                currentImu.quat[2],  // y
                currentImu.quat[3]   // z
            );

            // robotState.baseOrientation = sensorInput.imuOrientation;

            // Populate joint sensor data using mappings
            sensorInput.jointSensors.resize(CanonicalJointNames_.size());
            robotState.jointStates.resize(CanonicalJointNames_.size());
            for (size_t dataIdx = 0; dataIdx < CanonicalJointNames_.size(); ++dataIdx) {
                if (dataIdx < dataToHardwareMapping_.size()) {
                    size_t hardwareIdx = dataToHardwareMapping_[dataIdx];

                    if (hardwareIdx != SIZE_MAX && hardwareIdx < currentRobotState.q.size()) {
                        sensorInput.jointSensors[dataIdx].jointName = CanonicalJointNames_[dataIdx];
                        sensorInput.jointSensors[dataIdx].jointPos = currentRobotState.q[hardwareIdx];
                        sensorInput.jointSensors[dataIdx].jointVel = currentRobotState.dq[hardwareIdx];
                        sensorInput.jointSensors[dataIdx].jointTorque = currentRobotState.tau[hardwareIdx];
                        robotState.jointStates[dataIdx].jointName = CanonicalJointNames_[dataIdx];
                        robotState.jointStates[dataIdx].jointPos = currentRobotState.q[hardwareIdx];
                        robotState.jointStates[dataIdx].jointVel = currentRobotState.dq[hardwareIdx];
                    } else {
                        // Set defaults for unmapped joints
                        sensorInput.jointSensors[dataIdx].jointName = CanonicalJointNames_[dataIdx];
                        sensorInput.jointSensors[dataIdx].jointPos = 0.0;
                        sensorInput.jointSensors[dataIdx].jointVel = 0.0;
                        sensorInput.jointSensors[dataIdx].jointTorque = 0.0;
                        robotState.jointStates[dataIdx].jointName = CanonicalJointNames_[dataIdx];
                        robotState.jointStates[dataIdx].jointPos = 0.0;
                        robotState.jointStates[dataIdx].jointVel = 0.0;

                        if (hardwareIdx == SIZE_MAX) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                               "No hardware mapping for canonical joint %zu (%s)",
                                               dataIdx, CanonicalJointNames_[dataIdx].c_str());
                        }
                    }
                }
            }

            // update sensor data
            this->data_->setSensor(sensorInput);
            this->data_->setRobotState(robotState);

            // safety check with sensor values
            bool jointLimit = false;
            {
                // angle limits - check using the joint parameter arrays from RobotNode base class
                for (size_t dataIdx = 0; dataIdx < CanonicalJointNames_.size() && dataIdx < this->jointCount_; ++dataIdx) {
                    if (dataIdx < dataToHardwareMapping_.size()) {
                        size_t hardwareIdx = dataToHardwareMapping_[dataIdx];

                        if (hardwareIdx != SIZE_MAX && hardwareIdx < currentRobotState.q.size()) {
                            double angle = currentRobotState.q[hardwareIdx];
                            if (angle > this->jointPosMax_[dataIdx]) {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                   "Joint %s (idx %zu) angle %f exceeds max %f",
                                                   CanonicalJointNames_[dataIdx].c_str(), dataIdx, angle, this->jointPosMax_[dataIdx]);
                                jointLimit = true;
                            }
                            if (angle < this->jointPosMin_[dataIdx]) {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                   "Joint %s (idx %zu) angle %f below min %f",
                                                   CanonicalJointNames_[dataIdx].c_str(), dataIdx, angle, this->jointPosMin_[dataIdx]);
                                jointLimit = true;
                            }
                        }
                    }
                }
            }

            // // trigger estop if safe check failed
            // if (jointLimit && !this->data_->softEStop) {
            //     RCLCPP_WARN(this->get_logger(), "Joint limit breached, switching to ESTOP");
            //     this->fsm_broadcaster.broadcast_switch(States::ESTOP);
            // }

            // populate joystick command to robot data
            {
                auto command = this->data_->getCommand();

                // Check if joystick data is available
                if (currentJoy.axes.size() >= 3) {
                    // forward speed
                    if (currentJoy.axes[1] > 0) {
                        command.targetForwardSpeed = currentJoy.axes[1] * joystickMaxForwardVel_;
                    } else {
                        command.targetForwardSpeed = currentJoy.axes[1] * joystickMaxBackwardVel_;
                    }
                    // sideways speed
                    command.targetSidewaysSpeed = currentJoy.axes[0] * joystickMaxSidewaysVel_;
                    // turning speed
                    command.targetTurningSpeed = currentJoy.axes[2] * joystickMaxTurningVel_;
                }

                this->data_->setCommand(command);

                // Convert button vector to bitmask
                uint32_t keys = 0;
                for (size_t i = 0; i < currentJoy.buttons.size() && i < 32; ++i) {
                    if (currentJoy.buttons[i] != 0) {
                        keys |= (1 << i);
                    }
                }

                // Configurable keybindings for state transitions
                if (keys & modifierButton_) {
                    for (const auto& binding : stateKeybindings_) {
                        if (keys & binding.keyConstant) {
                            RCLCPP_INFO(this->get_logger(), "Trigger state transition to %s.", binding.stateName.c_str());

                            // Broadcast state transition using the stored state enum
                            this->fsm_broadcaster.broadcast_switch(binding.stateEnum);
                            break; // Only handle the first matching key
                        }
                    }
                }
            }
        }

        void updateCommandWithData() override {
            auto control = this->data_->getControlSignal();

            // populate control signal using joint mappings
            bool eStop = this->data_->softEStop;

            // Clear all motor commands first
            for (size_t i = 0; i < robot_cmd_.mode.size(); i++) {
                robot_cmd_.mode[i] = 0;
                robot_cmd_.Kp[i] = 0.0f;
                robot_cmd_.Kd[i] = 0.0f;
                robot_cmd_.q[i] = 0.0f;
                robot_cmd_.dq[i] = 0.0f;
                robot_cmd_.tau[i] = 0.0f;
            }

            // Set timestamp
            robot_cmd_.stamp = this->now().nanoseconds();

            // Apply control commands using mappings
            for (size_t dataIdx = 0; dataIdx < control.jointControl.size() && dataIdx < CanonicalJointNames_.size(); ++dataIdx) {
                if (dataIdx < dataToHardwareMapping_.size()) {
                    size_t hardwareIdx = dataToHardwareMapping_[dataIdx];

                    if (hardwareIdx != SIZE_MAX && hardwareIdx < robot_cmd_.mode.size()) {
                        if (eStop) {
                            robot_cmd_.mode[hardwareIdx] = 0;
                            robot_cmd_.Kp[hardwareIdx] = 0.0f;
                            robot_cmd_.Kd[hardwareIdx] = 0.0f;
                            robot_cmd_.q[hardwareIdx] = 0.0f;
                            robot_cmd_.dq[hardwareIdx] = 0.0f;
                            robot_cmd_.tau[hardwareIdx] = 0.0f;
                        } else {
                            double jointKp = (control.jointControl[dataIdx].stiffness > 0) ?
                                            control.jointControl[dataIdx].stiffness :
                                            this->jointStiffnessDefault_[dataIdx];
                            double jointKd = (control.jointControl[dataIdx].damping > 0) ?
                                            control.jointControl[dataIdx].damping :
                                            this->jointDampingDefault_[dataIdx];
                            switch (static_cast<int>(control.jointControl[dataIdx].mode)) {
                                case 1:  // Position mode
                                    robot_cmd_.mode[hardwareIdx] = 0;
                                    robot_cmd_.Kp[hardwareIdx] = jointKp;
                                    robot_cmd_.Kd[hardwareIdx] = jointKd;
                                    robot_cmd_.q[hardwareIdx] = control.jointControl[dataIdx].desiredPos;
                                    robot_cmd_.dq[hardwareIdx] = 0.0f;
                                    robot_cmd_.tau[hardwareIdx] = 0.0f;
                                    break;

                                case 2:  // Velocity mode
                                    robot_cmd_.mode[hardwareIdx] = 0;
                                    robot_cmd_.Kp[hardwareIdx] = 0.0f;
                                    robot_cmd_.Kd[hardwareIdx] = jointKd;
                                    robot_cmd_.q[hardwareIdx] = 0.0f;
                                    robot_cmd_.dq[hardwareIdx] = control.jointControl[dataIdx].desiredSpeed;
                                    robot_cmd_.tau[hardwareIdx] = 0.0f;
                                    break;

                                case 3:  // Force/Torque mode
                                    robot_cmd_.mode[hardwareIdx] = 0;
                                    robot_cmd_.Kp[hardwareIdx] = jointKp;
                                    robot_cmd_.Kd[hardwareIdx] = jointKd;
                                    robot_cmd_.q[hardwareIdx] = control.jointControl[dataIdx].desiredPos;
                                    robot_cmd_.dq[hardwareIdx] = control.jointControl[dataIdx].desiredSpeed;
                                    robot_cmd_.tau[hardwareIdx] = control.jointControl[dataIdx].desiredTorque;
                                    break;

                                default:  // Motor brake (same as soft e-stop)
                                    robot_cmd_.mode[hardwareIdx] = 0;
                                    robot_cmd_.Kp[hardwareIdx] = 0.0f;
                                    robot_cmd_.Kd[hardwareIdx] = 0.0f;
                                    robot_cmd_.q[hardwareIdx] = 0.0f;
                                    robot_cmd_.dq[hardwareIdx] = 0.0f;
                                    robot_cmd_.tau[hardwareIdx] = 0.0f;
                                    break;
                            }
                        }
                    } else {
                        if (hardwareIdx == SIZE_MAX) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                               "No hardware mapping for canonical joint %zu (%s)",
                                               dataIdx, CanonicalJointNames_[dataIdx].c_str());
                        } else {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                                "Hardware index %zu out of bounds for motor command array", hardwareIdx);
                        }
                    }
                }
            }

            // Publish command via LimX SDK
            bool publishSuccess = robot_->publishRobotCmd(robot_cmd_);
            if (!publishSuccess) {
                RCLCPP_ERROR(this->get_logger(), "Failed to publish robot command to LimX SDK");
            }
        }


    private:
        // Joint mappings
        std::vector<std::string> CanonicalJointNames_;  // Canonical order (policy order)
        std::vector<size_t> hardwareToDataMapping_;     // Maps LimX SDK motor index to canonical index
        std::vector<size_t> dataToHardwareMapping_;     // Maps canonical index to LimX SDK motor index

        // SDK configuration
        std::string robotIpAddress_;
        bool isSDKInitialized_;
        limxsdk::PointFoot* robot_;  // Use PointFoot for WheelFoot variant

        // Joystick velocity parameters
        double joystickMaxForwardVel_;
        double joystickMaxBackwardVel_;
        double joystickMaxSidewaysVel_;
        double joystickMaxTurningVel_;

        // Keybinding parameters
        uint32_t modifierButton_;
        std::vector<StateKeyBinding> stateKeybindings_;
        StateNameToEnumMap stateNameToEnumMap_;

        // LimX SDK data structures
        limxsdk::RobotState robot_state_;
        limxsdk::ImuData imu_data_;
        limxsdk::SensorJoy sensor_joy_;
        limxsdk::RobotCmd robot_cmd_;

        // Mutex for thread-safe access to SDK data structures
        mutable std::shared_mutex sdk_data_mutex_;
    };

}  // namespace crl::unitree::hardware::wf_tron1a

#endif  // CRL_HUMANOID_HARDWARE_WF_TRON1A_NODE
