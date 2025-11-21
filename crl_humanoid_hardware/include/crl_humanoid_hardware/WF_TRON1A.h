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

// limx_sdk
#include <limxsdk/wheellegged.h>
#include <limxsdk/datatypes.h>

namespace crl::unitree::hardware::wf_tron1a {

    // Number of motors in WF_TRON1A
    const int WF_TRON1A_NUM_MOTOR = 8;

    // Simple joystick data structure to maintain compatibility
    struct JoyData {
        float lx = 0.0f;
        float ly = 0.0f;
        float rx = 0.0f;
        float ry = 0.0f;
        uint16_t keys = 0;
    };

    template <typename States, typename Machines, std::size_t N>
    class Tron1aNode : public crl::humanoid::commons::RobotNode<States, Machines, N> {
        using BaseRobotNode = crl::humanoid::commons::RobotNode<States, Machines, N>;
        using StateNameToEnumMap = std::unordered_map<std::string, States>;

    public:
        // Structure to hold state-key mappings
        struct StateKeyBinding {
            std::string stateName;
            std::string keyName;
            uint16_t keyConstant;
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
              wheellegged_(nullptr),
              stateNameToEnumMap_(stateNameMap) {

            // Initialize default state mappings if none provided
            if (stateNameToEnumMap_.empty()) {
                initializeDefaultStateMappings();
            }

            // Declare and get robot IP address parameter
            this->declare_parameter("robot_ip_address", "127.0.0.1");  // Default: localhost for simulation
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
            this->declare_parameter("state_keybindings.keys", std::vector<std::string>{"B", "A", "X"});

            // Get keybinding parameters
            std::string modifier = this->get_parameter("keybinding_modifier").as_string();
            modifierButton_ = stringToButtonConstant(modifier);

            // Setup state keybindings
            setupStateKeybindings();

            // Setup joint mappings
            setupJointMappings();

            // Initialize SDK
            initializeSDK();

            // Setup publishers and subscribers (initializes command structure)
            setupCommunication();
        }

    private:
        // Convert string to button constant
        uint16_t stringToButtonConstant(const std::string& buttonName) {
            if (buttonName == "R1") return 1;  //0x1
            else if (buttonName == "L1") return 2;  //0x2
            else if (buttonName == "start") return 4;  //0x4
            else if (buttonName == "select") return 8;  //0x8
            else if (buttonName == "R2") return 16;  //0x10
            else if (buttonName == "L2") return 32;  //0x20
            else if (buttonName == "F1") return 64;  //0x40
            else if (buttonName == "F2") return 128;  //0x80
            else if (buttonName == "A") return 256;  //0x100
            else if (buttonName == "B") return 512;  //0x200
            else if (buttonName == "X") return 1024;  //0x400
            else if (buttonName == "Y") return 2048;  //0x800
            else if (buttonName == "up") return 4096; //0x1000
            else if (buttonName == "right") return 8192; //0x2000
            else if (buttonName == "down") return 16384; //0x4000
            else if (buttonName == "left") return 32768; //0x8000
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown button name: %s, defaulting to A", buttonName.c_str());
                return 256; // Default to A
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
                    stateKeybindings_.push_back({"ESTOP", "B", stringToButtonConstant("B"), estopIt->second});
                }
                if (standIt != stateNameToEnumMap_.end()) {
                    stateKeybindings_.push_back({"STAND", "A", stringToButtonConstant("A"), standIt->second});
                }
                if (walkIt != stateNameToEnumMap_.end()) {
                    stateKeybindings_.push_back({"WALK", "X", stringToButtonConstant("X"), walkIt->second});
                }
            }

            RCLCPP_INFO(this->get_logger(), "Using default keybindings: ESTOP=B, STAND=A, WALK=X");
        }
        
        void initializeSDK() {
            try {
                wheellegged_ = limxsdk::Wheellegged::getInstance();
                isSDKInitialized_ = wheellegged_->init(robotIpAddress_);
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

        // void setupCommunication() {
        //     try {
        //         // Create SDK publisher for low commands
        //         lowCommandPublisher_.reset(new ::unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(HG_CMD_TOPIC));
        //         lowCommandPublisher_->InitChannel();

        //         // Create SDK subscriber for low state
        //         lowStateSubscriber_.reset(new ::unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(HG_STATE_TOPIC));
        //         lowStateSubscriber_->InitChannel(
        //             std::bind(&Tron1aNode::LowStateHandler, this, std::placeholders::_1), 1);

        //         RCLCPP_INFO(this->get_logger(), "SDK communication channels established");
        //     } catch (const std::exception& e) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to setup communication: %s", e.what());
        //         throw;
        //     }
        // }

        void setupCommunication() {
            // Subscribe to robot state
            wheellegged_->subscribeRobotState(
                std::bind(&Tron1aNode::RobotStateHandler, this, std::placeholders::_1));
            
            // Subscribe to IMU data
            wheellegged_->subscribeImuData(
                std::bind(&Tron1aNode::ImuDataHandler, this, std::placeholders::_1));
            
            // Subscribe to joystick
            wheellegged_->subscribeSensorJoy(
                std::bind(&Tron1aNode::SensorJoyHandler, this, std::placeholders::_1));
            
            // Initialize command structure
            robot_cmd_ = limxsdk::RobotCmd(wheellegged_->getMotorNumber());
        }

        void RobotStateHandler(const limxsdk::RobotStateConstPtr &msg) {
            std::lock_guard<std::mutex> lock(stateMutex_);
            robot_state_ = *msg;
        }
        
        void ImuDataHandler(const limxsdk::ImuDataConstPtr &msg) {
            std::lock_guard<std::mutex> lock(imuMutex_);
            imu_data_ = *msg;
        }
        
        void SensorJoyHandler(const limxsdk::SensorJoyConstPtr &msg) {
            std::lock_guard<std::mutex> lock(joyMutex_);
            sensor_joy_ = *msg;
            extractWirelessController(); // Extract from sensor_joy_ instead
        }

        void extractWirelessController() {
            // Extract gamepad data from LimX SDK SensorJoy structure
            // LimX SDK joystick axes order: typically [lx, ly, rx, ry, ...]
            // Buttons are in a vector<int32_t>
            if (sensor_joy_.axes.size() >= 4) {
                joy_.lx = sensor_joy_.axes[0];
                joy_.ly = sensor_joy_.axes[1];
                joy_.rx = sensor_joy_.axes[2];
                joy_.ry = sensor_joy_.axes[3];
            }
            
            // Convert button vector to bitmask
            joy_.keys = 0;
            for (size_t i = 0; i < sensor_joy_.buttons.size() && i < 16; ++i) {
                if (sensor_joy_.buttons[i] != 0) {
                    joy_.keys |= (1 << i);
                }
            }
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
            // LimX SDK motor indices: 0: LF_HAA, 1: LF_HFE, 2: LF_KFE, 3: LF_WHL
            //                         4: LH_HAA, 5: LH_HFE, 6: LH_KFE, 7: LH_WHL (not used)
            //                         8: RF_HAA, 9: RF_HFE, 10: RF_KFE, 11: RF_WH
            //                         12: RH_HAA, 13: RH_HFE, 14: RH_KFE, 15: RH_WHL (not used)
            // Canonical joint names: abad_L_Joint, hip_L_Joint, knee_L_Joint, wheel_L_Joint,
            //                        abad_R_Joint, hip_R_Joint, knee_R_Joint, wheel_R_Joint
            // Direct mapping: abad_L->0, hip_L->1, knee_L->2, wheel_L->3,
            //                 abad_R->8, hip_R->9, knee_R->10, wheel_R->11
            std::map<std::string, size_t> canonicalToLimxMotorIndex = {
                {"abad_L_Joint", 0},
                {"hip_L_Joint", 1},
                {"knee_L_Joint", 2},
                {"wheel_L_Joint", 3},
                {"abad_R_Joint", 8},
                {"hip_R_Joint", 9},
                {"knee_R_Joint", 10},
                {"wheel_R_Joint", 11}
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
            crl::humanoid::commons::RobotSensor sensorInput;
            crl::humanoid::commons::RobotState robotState;

            // Thread-safe access to LimX SDK data
            limxsdk::RobotState currentRobotState;
            limxsdk::ImuData currentImu;
            JoyData currentJoy;
            {
                std::lock_guard<std::mutex> stateLock(stateMutex_);
                std::lock_guard<std::mutex> imuLock(imuMutex_);
                std::lock_guard<std::mutex> joyLock(joyMutex_);
                currentRobotState = robot_state_;
                currentImu = imu_data_;
                currentJoy = joy_;
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

                // forward speed
                if (currentJoy.ly > 0) {
                    command.targetForwardSpeed = currentJoy.ly * joystickMaxForwardVel_;
                } else {
                    command.targetForwardSpeed = currentJoy.ly * joystickMaxBackwardVel_;
                }
                // sideways speed
                command.targetSidewaysSpeed = -currentJoy.lx * joystickMaxSidewaysVel_;
                // turning speed
                command.targetTurningSpeed = -currentJoy.rx * joystickMaxTurningVel_;

                this->data_->setCommand(command);

                // Configurable keybindings for state transitions
                if (currentJoy.keys & modifierButton_) {
                    for (const auto& binding : stateKeybindings_) {
                        if (currentJoy.keys & binding.keyConstant) {
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
            robot_cmd_.stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

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
                                    robot_cmd_.mode[hardwareIdx] = 1;
                                    robot_cmd_.Kp[hardwareIdx] = jointKp;
                                    robot_cmd_.Kd[hardwareIdx] = jointKd;
                                    robot_cmd_.q[hardwareIdx] = control.jointControl[dataIdx].desiredPos;
                                    robot_cmd_.dq[hardwareIdx] = 0.0f;
                                    robot_cmd_.tau[hardwareIdx] = 0.0f;
                                    break;

                                case 2:  // Velocity mode
                                    robot_cmd_.mode[hardwareIdx] = 1;
                                    robot_cmd_.Kp[hardwareIdx] = 0.0f;
                                    robot_cmd_.Kd[hardwareIdx] = jointKd;
                                    robot_cmd_.q[hardwareIdx] = 0.0f;
                                    robot_cmd_.dq[hardwareIdx] = control.jointControl[dataIdx].desiredSpeed;
                                    robot_cmd_.tau[hardwareIdx] = 0.0f;
                                    break;

                                case 3:  // Force/Torque mode
                                    robot_cmd_.mode[hardwareIdx] = 1;
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
            wheellegged_->publishRobotCmd(robot_cmd_);
        }


    private:
        // Joint mappings
        std::vector<std::string> CanonicalJointNames_;  // Canonical order (policy order)
        std::vector<size_t> hardwareToDataMapping_;     // Maps LimX SDK motor index to canonical index
        std::vector<size_t> dataToHardwareMapping_;     // Maps canonical index to LimX SDK motor index

        // SDK configuration
        std::string robotIpAddress_;
        bool isSDKInitialized_;
        limxsdk::Wheellegged* wheellegged_;

        // Joystick velocity parameters
        double joystickMaxForwardVel_;
        double joystickMaxBackwardVel_;
        double joystickMaxSidewaysVel_;
        double joystickMaxTurningVel_;

        // Keybinding parameters
        uint16_t modifierButton_;
        std::vector<StateKeyBinding> stateKeybindings_;
        StateNameToEnumMap stateNameToEnumMap_;

        // LimX SDK data structures
        limxsdk::RobotCmd robot_cmd_;
        limxsdk::RobotState robot_state_;
        limxsdk::ImuData imu_data_;
        limxsdk::SensorJoy sensor_joy_;
        JoyData joy_;

        // Thread safety
        std::mutex stateMutex_;
        std::mutex imuMutex_;
        std::mutex joyMutex_;
    };

}  // namespace crl::unitree::hardware::wf_tron1a

#endif  //CRL_HUMANOID_HARDWARE_WF_TRON1A_NODE
