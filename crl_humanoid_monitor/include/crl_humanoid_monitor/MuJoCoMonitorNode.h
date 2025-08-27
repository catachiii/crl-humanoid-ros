#pragma once

// ROS2 includes
#include "rclcpp/rclcpp.hpp"

// MuJoCo includes
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Note: Removed crl-basic GUI plots to focus on MuJoCo rendering only

// ROS2 messages
#include "std_srvs/srv/trigger.hpp"
#include "crl_humanoid_msgs/msg/monitor.hpp"
#include "crl_humanoid_msgs/msg/remote.hpp"
#include "crl_humanoid_msgs/srv/ping.hpp"
#include "crl_humanoid_msgs/srv/restart.hpp"
#include "crl_humanoid_msgs/srv/elastic_band.hpp"

// crl_humanoid_commons
#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/RobotParameters.h"
#include "crl_humanoid_commons/helpers/MessageHelper.h"

// crl_fsm
#include "crl_fsm/client.h"

// crl_ros_helper
#include "crl_ros_helper/node.h"

// crl-basic logger
#include "crl-basic/utils/logger.h"

// Standard libraries
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include <cmath>

namespace crl::unitree::monitor {

    // Helper function to convert state enum to string
    template<typename States>
    const char* stateToString(States state) {
        switch (static_cast<int>(state)) {
            case 0: return "STAND";
            case 1: return "WALK";
            case 2: return "RUN";
            default: return "UNKNOWN";
        }
    }

    /**
     * MuJoCo-based monitor node for visualizing robot state using MuJoCo's GUI.
     * This is a pure viewer that subscribes to ROS2 topics and renders the robot state.
     */
    template <typename States, typename Machines, typename TransitionsCont, std::size_t N, typename LeggedRobotDataType>
    class BaseMuJoCoMonitorNode : public crl::ros::Node {
        static_assert(std::is_convertible<LeggedRobotDataType*, crl::unitree::commons::UnitreeLeggedRobotData*>::value,
                      "LeggedRobotDataType must inherit crl::unitree::commons::UnitreeLeggedRobotData as public");

    public:
        BaseMuJoCoMonitorNode(const std::string& to_monitor, const TransitionsCont& trans_cont, const std::array<Machines, N>& monitoring,
                              const std::string& nodeName = "mujoco_monitor")
            : crl::ros::Node(nodeName), monitoring_(monitoring), trans_cont_(trans_cont), fsmClient_(to_monitor, trans_cont, monitoring) {

            // Initialize ROS2 clients
            restartServiceClient_ = this->create_client<crl_humanoid_msgs::srv::Restart>("restart");
            elasticBandServiceClient_ = this->create_client<crl_humanoid_msgs::srv::ElasticBand>("elastic_band");

            // Initialize MuJoCo
            initializeMuJoCo();
        }

        ~BaseMuJoCoMonitorNode() override {
            // Cleanup MuJoCo
            if (mujocoData_) {
                mj_deleteData(mujocoData_);
            }
            if (mujocoModel_) {
                mj_deleteModel(mujocoModel_);
            }
        }

        /**
         * Initialize the node and MuJoCo GUI.
         */
        virtual bool initializeNode() {

            if (!rclcpp::ok()) {
                return false;
            }

            // Hardcode for G1 robot
            modelType_ = crl::unitree::commons::RobotModelType::UNITREE_G1;
            data_ = std::make_shared<LeggedRobotDataType>();

            // Initialize topic subscriptions
            monitorSubscription_ = this->create_safe_subscription<crl_humanoid_msgs::msg::Monitor>(
                "monitor", 10, std::bind(&BaseMuJoCoMonitorNode::monitorTopicSubscriptionCallback, this, std::placeholders::_1));

            // Initialize topic publisher
            remotePublisher_ = this->create_publisher<crl_humanoid_msgs::msg::Remote>("monitor_joystick", 10);

            // Ping service
            pingService_ = this->create_safe_service<crl_humanoid_msgs::srv::Ping>(
                "ping", std::bind(&BaseMuJoCoMonitorNode::pingServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

            return true;
        }

        /**
         * Update the monitor (called by the app).
         */
        void update() {

            // Add safety check for MuJoCo objects
            if (!mujocoModel_ || !mujocoData_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "MuJoCo objects not initialized in update()");
                return;
            }

            // Update FSM states
            try {
                fsmClient_.get_states(curStates_);
                assumeState_ = curStates_[0];
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Exception during FSM state update: %s", e.what());
                return;
            }

            // Update MuJoCo data with latest state (with thread safety)
            try {
                updateMuJoCoData();
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Exception during MuJoCo data update: %s", e.what());
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Unknown exception during MuJoCo data update");
            }
        }

        /**
         * Render the MuJoCo scene to the current OpenGL context.
         */
        void render() {
            std::lock_guard<std::mutex> lock(dataMutex_);
            if (!mujocoModel_ || !mujocoData_) {
                return;
            }

            try {
                // Get viewport
                int viewport[4];
                glGetIntegerv(GL_VIEWPORT, viewport);

                // Update scene
                mjv_updateScene(mujocoModel_, mujocoData_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

                // Render scene
                mjrRect viewport_rect = {0, 0, viewport[2], viewport[3]};
                mjr_render(viewport_rect, &scene_, &context_);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Exception during MuJoCo rendering: %s", e.what());
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Unknown exception during MuJoCo rendering");
            }
        }

        /**
         * Add node to executor for ROS2 callbacks.
         */
        void addToExecutor(rclcpp::executors::SingleThreadedExecutor& executor) {
            executor.add_node(shared_from_this());
            executor_ = &executor;
        }

        /**
         * Restart the robot.
         */
        void restart() {
            try {
                crl::ros::GateWrapper wrapper(gate);
                if (!wrapper.is_succ()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to acquire gate lock for restart");
                    return;
                }

                // Send FSM state change to ESTOP (restart)
                fsmClient_.broadcast_switch(States::ESTOP);

                // Reset UI command values
                commandUI_.targetForwardSpeed = 0;
                commandUI_.targetSidewaysSpeed = 0;
                commandUI_.targetTurningSpeed = 0;

                // Send restart service request
                if (restartServiceClient_ && restartServiceClient_->service_is_ready()) {
                    auto request = std::make_shared<crl_humanoid_msgs::srv::Restart::Request>();

                    // Send async request but don't wait for response to avoid blocking
                    auto future = restartServiceClient_->async_send_request(request);

                } else {
                    RCLCPP_WARN(this->get_logger(), "Restart service not ready");
                }

                RCLCPP_INFO(this->get_logger(), "Restart robot completed");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in restart(): %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in restart()");
            }
        }

        /**
         * Toggle the elastic band support for the robot in simulation.
         */
        void toggleElasticBand() {
            try {
                if (!elasticBandServiceClient_ || !elasticBandServiceClient_->service_is_ready()) {
                    RCLCPP_WARN(this->get_logger(), "Elastic band service not available");
                    return;
                }

                // Toggle the state
                elasticBandEnabled_ = !elasticBandEnabled_;

                auto request = std::make_shared<crl_humanoid_msgs::srv::ElasticBand::Request>();
                request->enable = elasticBandEnabled_;
                request->stiffness = 500.0;  // Default values
                request->damping = 100.0;
                request->target_height = 1.45; // Use current height

                // Send async request but don't wait for response to avoid blocking
                auto future = elasticBandServiceClient_->async_send_request(request);

                RCLCPP_INFO(this->get_logger(), "Elastic band %s command sent",
                           elasticBandEnabled_ ? "ENABLE" : "DISABLE");

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in toggleElasticBand(): %s", e.what());
                // Revert the state on error
                elasticBandEnabled_ = !elasticBandEnabled_;
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in toggleElasticBand()");
                // Revert the state on error
                elasticBandEnabled_ = !elasticBandEnabled_;
            }
        }

        /**
         * Get the current elastic band state.
         */
        bool isElasticBandEnabled() const {
            return elasticBandEnabled_;
        }

        /**
         * Initialize the OpenGL rendering context (call after OpenGL context is current).
         */
        void initializeRenderingContext() {
            if (!mujocoModel_) {
                RCLCPP_ERROR(this->get_logger(), "Cannot initialize rendering context: MuJoCo model is null");
                return;
            }

            // Check if OpenGL context is current
            if (!glfwGetCurrentContext()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot initialize rendering context: No OpenGL context current");
                return;
            }

            try {
                // Re-initialize the context structure before making the OpenGL context
                mjr_defaultContext(&context_);
                // Create the MuJoCo rendering context (following official MuJoCo pattern)
                mjr_makeContext(mujocoModel_, &context_, mjFONTSCALE_150);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during rendering context initialization: %s", e.what());
                throw;
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception during rendering context initialization");
                throw;
            }
        }

        /**
         * Get MuJoCo rendering context for text overlay.
         */
        mjrContext& getContext() {
            return context_;
        }

        /**
         * Get MuJoCo camera for camera control.
         */
        mjvCamera& getCamera() {
            return camera_;
        }

        /**
         * Get the robot's current base position.
         */
        std::array<double, 3> getRobotBasePosition() {
            std::array<double, 3> position = {0.0, 0.0, 0.8}; // Default fallback

            try {
                if (data_) {
                    auto state = data_->getLeggedRobotState();
                    position[0] = state.basePosition.x;
                    position[1] = state.basePosition.y;
                    position[2] = state.basePosition.z;
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Failed to get robot base position: %s", e.what());
            }

            return position;
        }

        /**
         * Get current FSM states for all monitored machines.
         */
        void getCurrentStates(std::array<States, N>& states) {
            try {
                fsmClient_.get_states(states);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to get FSM states: %s", e.what());
                // Return current known states as fallback
                states = curStates_;
            }
        }

        /**
         * Execute a state transition.
         */
        void executeTransition(States targetState) {
            try {
                auto result = fsmClient_.checked_broadcast_switch(targetState);

                switch (result) {
                    case crl::fsm::StateSwitchRes::SUCCESS:
                        RCLCPP_INFO(this->get_logger(), "Successfully transitioned to state %d (%s)",
                                  static_cast<int>(targetState), stateToString(targetState));
                        break;
                    case crl::fsm::StateSwitchRes::CANNOT:
                        RCLCPP_WARN(this->get_logger(), "Transition to state %d (%s) not allowed",
                                  static_cast<int>(targetState), stateToString(targetState));
                        break;
                    case crl::fsm::StateSwitchRes::TIMEOUT:
                        RCLCPP_ERROR(this->get_logger(), "Timeout during transition to state %d (%s)",
                                   static_cast<int>(targetState), stateToString(targetState));
                        break;
                    case crl::fsm::StateSwitchRes::OUTOFRANGE:
                        RCLCPP_ERROR(this->get_logger(), "Invalid state %d (%s) (out of range)",
                                   static_cast<int>(targetState), stateToString(targetState));
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown error during transition to state %d (%s)",
                                   static_cast<int>(targetState), stateToString(targetState));
                        break;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during state transition: %s", e.what());
                throw;
            }
        }

    protected:
        /**
         * Initialize MuJoCo components (without GLFW window).
         */
        void initializeMuJoCo() {
            // Load the same model as the simulator
            std::string xmlPath = std::string(CRL_HUMANOID_COMMONS_DATA_FOLDER) + "/robots/g1_description/scene_crl.xml";

            char error[1000];
            mujocoModel_ = mj_loadXML(xmlPath.c_str(), nullptr, error, sizeof(error));

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

            // Initialize visualization structures (following official MuJoCo pattern)
            mjv_defaultCamera(&camera_);
            mjv_defaultOption(&option_);
            mjv_defaultScene(&scene_);
            // Note: mjr_defaultContext() will be called later in initializeRenderingContext()

            // Create scene (use same buffer size as official MuJoCo basic.cc example)
            mjv_makeScene(mujocoModel_, &scene_, 2000);
        }

        /**
         * Update MuJoCo data from robot state.
         */
        void updateMuJoCoData() {
            std::lock_guard<std::mutex> lock(dataMutex_);
            if (!data_ || !mujocoModel_ || !mujocoData_) return;

            // Safely get robot state with exception handling
            crl::unitree::commons::LeggedRobotState state;
            try {
                state = data_->getLeggedRobotState();
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to get robot state during MuJoCo update: %s", e.what());
                return;
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Unknown exception while getting robot state during MuJoCo update");
                return;
            }

            // Update base position (first 3 DOFs)
            mujocoData_->qpos[0] = state.basePosition.x;  // x
            mujocoData_->qpos[1] = state.basePosition.y;  // y
            mujocoData_->qpos[2] = state.basePosition.z;  // z

            // Update base orientation (quaternion: w, x, y, z)
            mujocoData_->qpos[3] = state.baseOrientation.w();
            mujocoData_->qpos[4] = state.baseOrientation.x();
            mujocoData_->qpos[5] = state.baseOrientation.y();
            mujocoData_->qpos[6] = state.baseOrientation.z();

            // MuJoCo joint order (same as in SimNode.h)
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

            // Update joint positions with proper mapping (starting after base DOFs)
            int jointStartIndex = 7; // 3 pos + 4 quat for floating base
            for (const auto& jointState : state.jointStates) {
                // Find this joint in the MuJoCo XML order
                auto it = std::find(mujocoJointOrder.begin(), mujocoJointOrder.end(), jointState.jointName);
                if (it != mujocoJointOrder.end()) {
                    size_t mujocoIdx = std::distance(mujocoJointOrder.begin(), it);
                    int qposIndex = jointStartIndex + static_cast<int>(mujocoIdx);

                    if (qposIndex < mujocoModel_->nq) {
                        mujocoData_->qpos[qposIndex] = jointState.jointPos;
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                        "Joint '%s' not found in MuJoCo joint order", jointState.jointName.c_str());
                }
            }

            // Update base velocity (first 3 DOFs in velocity space)
            mujocoData_->qvel[0] = state.baseVelocity[0];  // vx
            mujocoData_->qvel[1] = state.baseVelocity[1];  // vy
            mujocoData_->qvel[2] = state.baseVelocity[2];  // vz

            // Update base angular velocity (next 3 DOFs in velocity space)
            mujocoData_->qvel[3] = state.baseAngularVelocity[0];  // wx
            mujocoData_->qvel[4] = state.baseAngularVelocity[1];  // wy
            mujocoData_->qvel[5] = state.baseAngularVelocity[2];  // wz

            // Update joint velocities with proper mapping (starting after base DOFs)
            int jointVelStartIndex = 6; // 3 linear + 3 angular for floating base
            for (const auto& jointState : state.jointStates) {
                // Find this joint in the MuJoCo XML order
                auto it = std::find(mujocoJointOrder.begin(), mujocoJointOrder.end(), jointState.jointName);
                if (it != mujocoJointOrder.end()) {
                    size_t mujocoIdx = std::distance(mujocoJointOrder.begin(), it);
                    int qvelIndex = jointVelStartIndex + static_cast<int>(mujocoIdx);

                    if (qvelIndex < mujocoModel_->nv) {
                        mujocoData_->qvel[qvelIndex] = jointState.jointVel;
                    }
                }
            }

            // Forward simulation to update derived quantities
            mj_forward(mujocoModel_, mujocoData_);
        }



        /**
         * Monitor topic subscription callback.
         */
        void monitorTopicSubscriptionCallback(const crl_humanoid_msgs::msg::Monitor::SharedPtr msg) {
            try {
                // Safety check for MuJoCo objects before processing any data
                if (!mujocoModel_ || !mujocoData_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "MuJoCo objects not initialized in monitor callback, skipping");
                    return;
                }

                // Validate message before processing
                if (!msg) {
                    RCLCPP_WARN(this->get_logger(), "Received null monitor message");
                    return;
                }

                // Basic sanity checks on the message content
                if (msg->state.joint.name.size() == 0) {
                    RCLCPP_WARN(this->get_logger(), "Received monitor message with no joint states");
                    return;
                }

                // Check for NaN or infinite values in critical data
                if (!std::isfinite(msg->state.base_pose.pose.position.x) ||
                    !std::isfinite(msg->state.base_pose.pose.position.y) ||
                    !std::isfinite(msg->state.base_pose.pose.position.z)) {
                    RCLCPP_WARN(this->get_logger(), "Received monitor message with invalid base position");
                    return;
                }

                if (!std::isfinite(msg->state.base_pose.pose.orientation.w) ||
                    !std::isfinite(msg->state.base_pose.pose.orientation.x) ||
                    !std::isfinite(msg->state.base_pose.pose.orientation.y) ||
                    !std::isfinite(msg->state.base_pose.pose.orientation.z)) {
                    RCLCPP_WARN(this->get_logger(), "Received monitor message with invalid base orientation");
                    return;
                }

                RCLCPP_DEBUG(this->get_logger(), "Message validation passed, acquiring lock");
                std::lock_guard<std::mutex> lock(dataMutex_);

                if (!data_) {
                    RCLCPP_WARN(this->get_logger(), "Data object is null in monitor callback");
                    return;
                }

                RCLCPP_DEBUG(this->get_logger(), "Starting data update");

                // Update timestamp
                double t = rclcpp::Time(msg->header.stamp).seconds();
                if (!std::isfinite(t)) {
                    RCLCPP_WARN(this->get_logger(), "Received monitor message with invalid timestamp");
                    return;
                }
                data_->timeStamp = t;

                // Update command
                crl::unitree::commons::LeggedRobotCommand command;
                crl::unitree::commons::populateDataFromRemoteMessage(msg->remote, command);
                data_->setCommand(command);

                // Update sensor values
                auto sensor = data_->getSensor();
                crl::unitree::commons::populateDataFromSensorMessage(msg->sensor, sensor);
                data_->setSensor(sensor);

                // Update state estimation
                auto state = data_->getLeggedRobotState();
                crl::unitree::commons::populateDataFromStateMessage(msg->state, state);
                data_->setLeggedRobotState(state);

                // Update control
                auto control = data_->getControlSignal();
                crl::unitree::commons::populateDataFromControlMessage(msg->control, control);
                data_->setControlSignal(control);

                // Update profiling info
                auto profilingInfo = data_->getProfilingInfo();
                crl::unitree::commons::populateDataFromProfilingInfoMessage(msg->profiling_info, profilingInfo);
                data_->setProfilingInfo(profilingInfo);

                // Publish remote commands
                publishRemoteCommands();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in monitor callback: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in monitor callback");
            }
        }

        /**
         * Publish remote commands.
         */
        void publishRemoteCommands() {
            try {
                if (!data_) return;

                // Publish remote topic
                auto message = crl_humanoid_msgs::msg::Remote();
                crl::unitree::commons::populateRemoteMessageFromData(commandUI_, message);
                remotePublisher_->publish(message);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Exception in publishRemoteCommands: %s", e.what());
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Unknown exception in publishRemoteCommands");
            }
        }

        void pingServiceCallback(const std::shared_ptr<crl_humanoid_msgs::srv::Ping::Request>,
                                std::shared_ptr<crl_humanoid_msgs::srv::Ping::Response>) {
            // We don't have to do anything here.
            return;
        }

    protected:
        // Robot model
        crl::unitree::commons::RobotModelType modelType_ = crl::unitree::commons::RobotModelType::UNKNOWN;

        // Data pipeline
        std::shared_ptr<LeggedRobotDataType> data_ = nullptr;
        std::mutex dataMutex_;

        // FSM
        const std::array<Machines, N> monitoring_;
        const TransitionsCont trans_cont_;
        std::array<States, N> curStates_;
        States assumeState_;
        fsm::Client<States, Machines, TransitionsCont, N> fsmClient_;

        // ROS2 executor for callbacks (will be set externally)
        rclcpp::executors::SingleThreadedExecutor* executor_ = nullptr;

    private:
        // Remote command set by UI
        crl::unitree::commons::LeggedRobotCommand commandUI_;

        // Topic communication
        rclcpp::Subscription<crl_humanoid_msgs::msg::Monitor>::SharedPtr monitorSubscription_ = nullptr;
        rclcpp::Publisher<crl_humanoid_msgs::msg::Remote>::SharedPtr remotePublisher_ = nullptr;

        // Service communication
        // Service clients
        rclcpp::Client<crl_humanoid_msgs::srv::Restart>::SharedPtr restartServiceClient_ = nullptr;
        rclcpp::Client<crl_humanoid_msgs::srv::ElasticBand>::SharedPtr elasticBandServiceClient_ = nullptr;

        // Elastic band state
        bool elasticBandEnabled_ = true;
        rclcpp::Service<crl_humanoid_msgs::srv::Ping>::SharedPtr pingService_ = nullptr;

        // MuJoCo objects
        mjModel* mujocoModel_ = nullptr;
        mjData* mujocoData_ = nullptr;
        mjvScene scene_;
        mjvOption option_;
        mjvCamera camera_;
        mjrContext context_;

        // Type definitions for data structures
        typedef crl::unitree::commons::LeggedRobotState State;
        typedef std::vector<crl::unitree::commons::LeggedRobotSensor::LeggedRobotJointSensor> JointSensors;
        typedef std::vector<State::LeggedRobotJointState> JointStates;
        typedef std::vector<crl::unitree::commons::LeggedRobotControlSignal::LeggedRobotJointControlSignal> JointControl;
        typedef crl::unitree::commons::ProfilingInfo ProfilingInfo;
    };

    /**
     * Default MuJoCo monitor node.
     */
    template <typename States, typename Machines, typename TransitionsCont, std::size_t N>
    class UnitreeMuJoCoMonitorNode final : public BaseMuJoCoMonitorNode<States, Machines, TransitionsCont, N, crl::unitree::commons::UnitreeLeggedRobotData> {
        using BaseRobotNode = BaseMuJoCoMonitorNode<States, Machines, TransitionsCont, N, crl::unitree::commons::UnitreeLeggedRobotData>;

    public:
        UnitreeMuJoCoMonitorNode(const std::string& to_monitor, const TransitionsCont& trans_cont, const std::array<Machines, N>& monitoring)
            : BaseRobotNode(to_monitor, trans_cont, monitoring) {}

        ~UnitreeMuJoCoMonitorNode() override = default;
    };

}  // namespace crl::unitree::monitor
