#ifndef CRL_HUMANOID_HARDWARE_G1_NODE
#define CRL_HUMANOID_HARDWARE_G1_NODE
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <chrono>
#include <unordered_map>

// crl_humanoid_commons
#include "crl_humanoid_commons/nodes/RobotNode.h"

// unitree_sdk2
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/common/thread/thread.hpp>

namespace crl::unitree::hardware::g1 {

    // Number of motors in G1
    const int G1_NUM_MOTOR = 29;

    // DDS topic names
    static const std::string HG_CMD_TOPIC = "rt/lowcmd";
    static const std::string HG_STATE_TOPIC = "rt/lowstate";

    // Gamepad data structures (from SDK examples)
    typedef union {
        struct {
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t start : 1;
            uint8_t select : 1;
            uint8_t R2 : 1;
            uint8_t L2 : 1;
            uint8_t F1 : 1;
            uint8_t F2 : 1;
            uint8_t A : 1;
            uint8_t B : 1;
            uint8_t X : 1;
            uint8_t Y : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;
        } components;
        uint16_t value;
    } xKeySwitchUnion;

    typedef struct {
        uint8_t head[2];
        xKeySwitchUnion btn;
        float lx;
        float rx;
        float ry;
        float L2;
        float ly;
        uint8_t idle[16];
    } xRockerBtnDataStruct;

    // CRC calculation function
    inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
        uint32_t xbit = 0;
        uint32_t data = 0;
        uint32_t CRC32 = 0xFFFFFFFF;
        const uint32_t dwPolynomial = 0x04c11db7;
        for (uint32_t i = 0; i < len; i++) {
            xbit = 1 << 31;
            data = ptr[i];
            for (uint32_t bits = 0; bits < 32; bits++) {
                if (CRC32 & 0x80000000) {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                } else
                    CRC32 <<= 1;
                if (data & xbit) CRC32 ^= dwPolynomial;
                xbit >>= 1;
            }
        }
        return CRC32;
    }

    // Simple joystick data structure to maintain compatibility
    struct JoyData {
        float lx = 0.0f;
        float ly = 0.0f;
        float rx = 0.0f;
        float ry = 0.0f;
        uint16_t keys = 0;
    };

    template <typename States, typename Machines, std::size_t N>
    class G1Node : public crl::humanoid::commons::RobotNode<States, Machines, N> {
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

        G1Node(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
               const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
               const std::array<Machines, N>& monitoring,
               const std::atomic<bool>& is_transitioning,
               const StateNameToEnumMap& stateNameMap = {})
            : BaseRobotNode(model, data, monitoring, is_transitioning),
              isSDKInitialized_(false),
              mode_machine_(0),
              stateNameToEnumMap_(stateNameMap) {

            // Initialize default state mappings if none provided
            if (stateNameToEnumMap_.empty()) {
                initializeDefaultStateMappings();
            }

            // Declare and get network interface parameter
            this->declare_parameter("network_interface", "eth0");
            networkInterface_ = this->get_parameter("network_interface").as_string();

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

            // Initialize low command
            InitLowCmd();

            // Setup publishers and subscribers
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
                ::unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface_);
                isSDKInitialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Unitree SDK initialized successfully on interface: %s", networkInterface_.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize Unitree SDK: %s", e.what());
                throw;
            }
        }

        void setupCommunication() {
            try {
                // Create SDK publisher for low commands
                lowCommandPublisher_.reset(new ::unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(HG_CMD_TOPIC));
                lowCommandPublisher_->InitChannel();

                // Create SDK subscriber for low state
                lowStateSubscriber_.reset(new ::unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(HG_STATE_TOPIC));
                lowStateSubscriber_->InitChannel(
                    std::bind(&G1Node::LowStateHandler, this, std::placeholders::_1), 1);

                RCLCPP_INFO(this->get_logger(), "SDK communication channels established");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to setup communication: %s", e.what());
                throw;
            }
        }

        void LowStateHandler(const void* message) {
            unitree_hg::msg::dds_::LowState_ low_state = *(const unitree_hg::msg::dds_::LowState_*)message;

            // Verify CRC
            if (low_state.crc() != Crc32Core((uint32_t*)&low_state,
                                           (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Low state CRC error");
                return;
            }

            // Update mode machine and log if it changes (similar to G1 example)
            if (mode_machine_ != low_state.mode_machine()) {
                if (mode_machine_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "G1 type: %u", static_cast<unsigned>(low_state.mode_machine()));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Mode machine changed from %u to %u",
                               static_cast<unsigned>(mode_machine_), static_cast<unsigned>(low_state.mode_machine()));
                }
                mode_machine_ = low_state.mode_machine();
            }

            // Update state
            {
                std::lock_guard<std::mutex> lock(stateMutex_);
                state_ = low_state;
            }

            // Extract wireless controller data
            extractWirelessController();
        }

        void extractWirelessController() {
            std::lock_guard<std::mutex> lock(joyMutex_);

            // Extract gamepad data from state_.wireless_remote_ array
            if (state_.wireless_remote().size() >= sizeof(xRockerBtnDataStruct)) {
                const xRockerBtnDataStruct* gamepad =
                    reinterpret_cast<const xRockerBtnDataStruct*>(state_.wireless_remote().data());

                // Convert to our joy_ format
                joy_.lx = gamepad->lx;
                joy_.ly = gamepad->ly;
                joy_.rx = gamepad->rx;
                joy_.ry = gamepad->ry;
                joy_.keys = gamepad->btn.value;
            }
        }
        void setupJointMappings() {
            CanonicalJointNames_.clear();
            HardwareJointNames_.clear();
            hardwareToDataMapping_.clear();
            dataToHardwareMapping_.clear();

            // Get the canonical joint order from data control signal
            auto control = this->data_->getControlSignal();
            CanonicalJointNames_.reserve(control.jointControl.size());
            for (const auto& jointControl : control.jointControl) {
                CanonicalJointNames_.push_back(jointControl.name);
            }

            // Hardware joint order (as defined by the G1 robot hardware)
            HardwareJointNames_ = {"left_hip_pitch_joint",     "left_hip_roll_joint",     "left_hip_yaw_joint",         "left_knee_joint",
                                   "left_ankle_pitch_joint",   "left_ankle_roll_joint",   "right_hip_pitch_joint",      "right_hip_roll_joint",
                                   "right_hip_yaw_joint",      "right_knee_joint",        "right_ankle_pitch_joint",    "right_ankle_roll_joint",
                                   "waist_yaw_joint",          "waist_roll_joint",        "waist_pitch_joint",          "left_shoulder_pitch_joint",
                                   "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",           "left_wrist_roll_joint",
                                   "left_wrist_pitch_joint",   "left_wrist_yaw_joint",    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                                   "right_shoulder_yaw_joint", "right_elbow_joint",       "right_wrist_roll_joint",     "right_wrist_pitch_joint",
                                   "right_wrist_yaw_joint"};

            // Create mapping between canonical data order and Hardware order
            hardwareToDataMapping_.resize(HardwareJointNames_.size());
            dataToHardwareMapping_.resize(CanonicalJointNames_.size());

            // Initialize mappings with invalid values to detect unset mappings
            std::fill(hardwareToDataMapping_.begin(), hardwareToDataMapping_.end(), SIZE_MAX);
            std::fill(dataToHardwareMapping_.begin(), dataToHardwareMapping_.end(), SIZE_MAX);

            // Create mapping: canonical (data) order -> Hardware order
            for (size_t dataIdx = 0; dataIdx < CanonicalJointNames_.size(); ++dataIdx) {
                const std::string& canonicalJointName = CanonicalJointNames_[dataIdx];

                // Find this joint in the Hardware order
                auto it = std::find(HardwareJointNames_.begin(), HardwareJointNames_.end(), canonicalJointName);
                if (it != HardwareJointNames_.end()) {
                    size_t hardwareIdx = std::distance(HardwareJointNames_.begin(), it);

                    // Bounds check before setting mapping
                    if (hardwareIdx < hardwareToDataMapping_.size()) {
                        hardwareToDataMapping_[hardwareIdx] = dataIdx;
                        dataToHardwareMapping_[dataIdx] = hardwareIdx;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Hardware index %zu out of bounds for mapping array size %zu", hardwareIdx,
                                     hardwareToDataMapping_.size());
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Joint '%s' found in canonical order but not in Hardware order!", canonicalJointName.c_str());
                }
            }

            // Validation: Check that all canonical joints were mapped
            for (size_t dataIdx = 0; dataIdx < dataToHardwareMapping_.size(); ++dataIdx) {
                if (dataToHardwareMapping_[dataIdx] == SIZE_MAX) {
                    RCLCPP_ERROR(this->get_logger(), "Canonical index %zu was not mapped to any Hardware joint", dataIdx);
                } else if (dataToHardwareMapping_[dataIdx] >= HardwareJointNames_.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mapping: Canonical index %zu maps to invalid Hardware index %zu", dataIdx,
                                 dataToHardwareMapping_[dataIdx]);
                }
            }

            // Validation: Check that all Hardware joints were mapped
            for (size_t hardwareIdx = 0; hardwareIdx < hardwareToDataMapping_.size(); ++hardwareIdx) {
                if (hardwareToDataMapping_[hardwareIdx] == SIZE_MAX) {
                    RCLCPP_WARN(this->get_logger(), "Hardware index %zu was not mapped to any canonical joint", hardwareIdx);
                } else if (hardwareToDataMapping_[hardwareIdx] >= CanonicalJointNames_.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mapping: Hardware index %zu maps to invalid canonical index %zu", hardwareIdx,
                                 hardwareToDataMapping_[hardwareIdx]);
                }
            }
        }

        void updateDataWithSensorReadings() override {
            // Create sensor data structure for humanoid robot
            crl::humanoid::commons::RobotSensor sensorInput;
            crl::humanoid::commons::RobotState robotState;

            // Thread-safe access to state
            unitree_hg::msg::dds_::LowState_ currentState;
            JoyData currentJoy;
            {
                std::lock_guard<std::mutex> stateLock(stateMutex_);
                std::lock_guard<std::mutex> joyLock(joyMutex_);
                currentState = state_;
                currentJoy = joy_;
            }

            // Populate IMU data using DDS message structure
            sensorInput.accelerometer = crl::V3D(
                currentState.imu_state().accelerometer()[0],
                currentState.imu_state().accelerometer()[1],
                currentState.imu_state().accelerometer()[2]
            );
            sensorInput.gyroscope = crl::V3D(
                currentState.imu_state().gyroscope()[0],
                currentState.imu_state().gyroscope()[1],
                currentState.imu_state().gyroscope()[2]
            );
            sensorInput.imuOrientation = crl::Quaternion(
                currentState.imu_state().quaternion()[0],
                currentState.imu_state().quaternion()[1],
                currentState.imu_state().quaternion()[2],
                currentState.imu_state().quaternion()[3]
            );

            // Populate joint sensor data using mappings
            sensorInput.jointSensors.resize(CanonicalJointNames_.size());
            robotState.jointStates.resize(CanonicalJointNames_.size());
            for (size_t dataIdx = 0; dataIdx < CanonicalJointNames_.size(); ++dataIdx) {
                if (dataIdx < dataToHardwareMapping_.size()) {
                    size_t hardwareIdx = dataToHardwareMapping_[dataIdx];

                    if (hardwareIdx != SIZE_MAX && hardwareIdx < currentState.motor_state().size()) {
                        sensorInput.jointSensors[dataIdx].jointName = CanonicalJointNames_[dataIdx];
                        sensorInput.jointSensors[dataIdx].jointPos = currentState.motor_state()[hardwareIdx].q();
                        sensorInput.jointSensors[dataIdx].jointVel = currentState.motor_state()[hardwareIdx].dq();
                        sensorInput.jointSensors[dataIdx].jointTorque = currentState.motor_state()[hardwareIdx].tau_est();
                        robotState.jointStates[dataIdx].jointName = CanonicalJointNames_[dataIdx];
                        robotState.jointStates[dataIdx].jointPos = currentState.motor_state()[hardwareIdx].q();
                        robotState.jointStates[dataIdx].jointVel = currentState.motor_state()[hardwareIdx].dq();
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

                        if (hardwareIdx != SIZE_MAX && hardwareIdx < currentState.motor_state().size()) {
                            double angle = currentState.motor_state()[hardwareIdx].q();
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

            // Clear all motor commands first (G1 has up to 35 motors in DDS structure)
            for (size_t i = 0; i < cmd_.motor_cmd().size(); i++) {
                cmd_.motor_cmd()[i].mode() = 0;
                cmd_.motor_cmd()[i].kp() = 0;
                cmd_.motor_cmd()[i].kd() = 0;
                cmd_.motor_cmd()[i].q() = 0.0f;
                cmd_.motor_cmd()[i].dq() = 0.0f;
                cmd_.motor_cmd()[i].tau() = 0;
            }

            cmd_.mode_pr() = 0; // PR mode 0, AB mode 1
            cmd_.mode_machine() = mode_machine_;


            // Apply control commands using mappings
            for (size_t dataIdx = 0; dataIdx < control.jointControl.size() && dataIdx < CanonicalJointNames_.size(); ++dataIdx) {
                if (dataIdx < dataToHardwareMapping_.size()) {
                    size_t hardwareIdx = dataToHardwareMapping_[dataIdx];

                    if (hardwareIdx != SIZE_MAX && hardwareIdx < cmd_.motor_cmd().size()) {
                        if (eStop) {
                            cmd_.motor_cmd()[hardwareIdx].mode() = 0;
                            cmd_.motor_cmd()[hardwareIdx].kp() = 0;
                            cmd_.motor_cmd()[hardwareIdx].kd() = 0;
                            cmd_.motor_cmd()[hardwareIdx].q() = 0.0f;
                            cmd_.motor_cmd()[hardwareIdx].dq() = 0.0f;
                            cmd_.motor_cmd()[hardwareIdx].tau() = 0;
                        } else {
                            double jointKp = (control.jointControl[dataIdx].stiffness > 0) ?
                                            control.jointControl[dataIdx].stiffness :
                                            this->jointStiffnessDefault_[dataIdx];
                            double jointKd = (control.jointControl[dataIdx].damping > 0) ?
                                            control.jointControl[dataIdx].damping :
                                            this->jointDampingDefault_[dataIdx];
                            switch (static_cast<int>(control.jointControl[dataIdx].mode)) {
                                case 1:  // Position mode
                                    cmd_.motor_cmd()[hardwareIdx].mode() = 1;
                                    cmd_.motor_cmd()[hardwareIdx].kp() = jointKp;
                                    cmd_.motor_cmd()[hardwareIdx].kd() = jointKd;
                                    cmd_.motor_cmd()[hardwareIdx].q() = (float)control.jointControl[dataIdx].desiredPos;
                                    cmd_.motor_cmd()[hardwareIdx].dq() = 0;
                                    cmd_.motor_cmd()[hardwareIdx].tau() = 0;
                                    break;

                                case 2:  // Velocity mode
                                    cmd_.motor_cmd()[hardwareIdx].mode() = 1;
                                    cmd_.motor_cmd()[hardwareIdx].kp() = 0;
                                    cmd_.motor_cmd()[hardwareIdx].kd() = jointKd;
                                    cmd_.motor_cmd()[hardwareIdx].q() = 0.0f;
                                    cmd_.motor_cmd()[hardwareIdx].dq() = (float)control.jointControl[dataIdx].desiredSpeed;
                                    cmd_.motor_cmd()[hardwareIdx].tau() = 0;
                                    break;

                                case 3:  // Force/Torque mode
                                    cmd_.motor_cmd()[hardwareIdx].mode() = 1;
                                    cmd_.motor_cmd()[hardwareIdx].kp() = jointKp;
                                    cmd_.motor_cmd()[hardwareIdx].kd() = jointKd;
                                    cmd_.motor_cmd()[hardwareIdx].q() = (float)control.jointControl[dataIdx].desiredPos;
                                    cmd_.motor_cmd()[hardwareIdx].dq() = (float)control.jointControl[dataIdx].desiredSpeed;
                                    cmd_.motor_cmd()[hardwareIdx].tau() = (float)control.jointControl[dataIdx].desiredTorque;
                                    break;

                                default:  // Motor brake (same as soft e-stop)
                                    cmd_.motor_cmd()[hardwareIdx].mode() = 0;
                                    cmd_.motor_cmd()[hardwareIdx].kp() = 0;
                                    cmd_.motor_cmd()[hardwareIdx].kd() = 0;
                                    cmd_.motor_cmd()[hardwareIdx].q() = 0.0f;
                                    cmd_.motor_cmd()[hardwareIdx].dq() = 0.0f;
                                    cmd_.motor_cmd()[hardwareIdx].tau() = 0;
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

            // Calculate CRC and publish via SDK
            cmd_.crc() = Crc32Core((uint32_t*)&cmd_, (sizeof(cmd_) >> 2) - 1);
            lowCommandPublisher_->Write(cmd_);
        }

        void InitLowCmd() {
            // Initialize DDS command structure
            cmd_.mode_pr() = 0;
            cmd_.mode_machine() = mode_machine_;  // Use stored mode_machine value

            // Initialize all motors (G1 DDS structure has 35 motor slots)
            for (size_t i = 0; i < cmd_.motor_cmd().size(); i++) {
                cmd_.motor_cmd()[i].mode() = 0x01;  // motor switch to servo (PMSM) mode
                cmd_.motor_cmd()[i].q() = 0.0f;
                cmd_.motor_cmd()[i].dq() = 0.0f;
                cmd_.motor_cmd()[i].kp() = 0;
                cmd_.motor_cmd()[i].kd() = 0;
                cmd_.motor_cmd()[i].tau() = 0;
            }

            // Initialize reserve array
            for (size_t i = 0; i < cmd_.reserve().size(); i++) {
                cmd_.reserve()[i] = 0;
            }

            cmd_.crc() = 0;
        }

    private:
        // Joint mappings
        std::vector<std::string> CanonicalJointNames_;  // Canonical order (policy order)
        std::vector<std::string> HardwareJointNames_;   // Hardware order (robot-specific)
        std::vector<size_t> hardwareToDataMapping_;     // Maps Hardware XML index to canonical index
        std::vector<size_t> dataToHardwareMapping_;     // Maps canonical index to Hardware XML index

        // SDK configuration
        std::string networkInterface_;
        bool isSDKInitialized_;
        uint8_t mode_machine_;

        // Joystick velocity parameters
        double joystickMaxForwardVel_;
        double joystickMaxBackwardVel_;
        double joystickMaxSidewaysVel_;
        double joystickMaxTurningVel_;

        // Keybinding parameters
        uint16_t modifierButton_;
        std::vector<StateKeyBinding> stateKeybindings_;
        StateNameToEnumMap stateNameToEnumMap_;

        // SDK publishers and subscribers
        ::unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowCommandPublisher_;
        ::unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowStateSubscriber_;

        // DDS message types
        unitree_hg::msg::dds_::LowCmd_ cmd_;
        unitree_hg::msg::dds_::LowState_ state_;
        JoyData joy_;

        // Thread safety
        std::mutex stateMutex_;
        std::mutex joyMutex_;
    };

}  // namespace crl::unitree::hardware::g1

#endif  //CRL_HUMANOID_HARDWARE_G1_NODE
