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
#include <algorithm>

namespace crl::humanoid::monitor {

    // Helper function to convert state enum to string
    template<typename States>
    const char* stateToString(States state) {
        // Debug: Log the state value
        RCLCPP_DEBUG(rclcpp::get_logger("monitor_app"), "Converting state with value: %d", static_cast<int>(state));

        // Use static storage to ensure string lifetime
        static thread_local std::string state_str;

        // Convert the underlying value to get the correct string representation
        // The issue was that we were comparing enum instances instead of underlying values
        typename States::UT stateValue = state.to_ut();

        // Dynamically handle all states defined in the enum
        for (std::size_t i = 0; i < States::num_params; ++i) {
            auto enumFromIndex = States::from_ind(i);
            if (enumFromIndex.to_ut() == stateValue) {
                // Convert string_view to string and return c_str()
                auto sv = enumFromIndex.to_string();
                state_str = std::string(sv);
                return state_str.c_str();
            }
        }

        RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Unknown state value: %d", static_cast<int>(stateValue));
        return "UNKNOWN";
    }

    /**
     * MuJoCo-based monitor node for visualizing robot state using MuJoCo's GUI.
     * This is a pure viewer that subscribes to ROS2 topics and renders the robot state.
     */
    template <typename States, typename Machines, typename TransitionsCont, std::size_t N, typename RobotDataType>
    class BaseMuJoCoMonitorNode : public crl::ros::Node {
        static_assert(std::is_convertible<RobotDataType*, crl::humanoid::commons::RobotData*>::value,
                      "RobotDataType must inherit crl::humanoid::commons::RobotData as public");

    public:
        BaseMuJoCoMonitorNode(const std::string& to_monitor, const TransitionsCont& trans_cont, const std::array<Machines, N>& monitoring,
                              const std::string& nodeName = "mujoco_monitor")
            : crl::ros::Node(nodeName), monitoring_(monitoring), trans_cont_(trans_cont), fsmClient_(to_monitor, trans_cont, monitoring) {


            // params
            auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
            paramDesc.description = "MuJoCo monitor parameters";
            paramDesc.read_only = true;
            // Neutral defaults; YAML/launch should override
            this->declare_parameter<std::string>("model", "g1", paramDesc);
            this->declare_parameter<std::string>("robot_xml_file", "g1_description/scene_crl.xml", paramDesc);

            // Initialize ROS2 clients
            restartServiceClient_ = this->create_client<crl_humanoid_msgs::srv::Restart>("restart");
            elasticBandServiceClient_ = this->create_client<crl_humanoid_msgs::srv::ElasticBand>("elastic_band");

            // Note: MuJoCo initialization is delayed until initializeNode() when parameters are fully loaded
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

            // Debug: show node identity and current parameter values (post-YAML load)
            RCLCPP_INFO(this->get_logger(), "FQN: %s | node: %s", this->get_fully_qualified_name(), this->get_name());
            RCLCPP_INFO(this->get_logger(), "Param check -> model: %s | robot_xml_file: %s",
                        this->get_parameter("model").as_string().c_str(),
                        this->get_parameter("robot_xml_file").as_string().c_str());
            // Optionally list parameters visible to this node
            {
                auto listed = this->list_parameters({}, 1);
                std::string names;
                for (const auto &n : listed.names) {
                    if (!names.empty()) names += ", ";
                    names += n;
                }
                RCLCPP_INFO(this->get_logger(), "Visible parameters: [%s]", names.c_str());
            }

            // get model type
            std::string modelParam = this->get_parameter("model").as_string();
            if (modelParam == "g1") {
                modelType_ = crl::humanoid::commons::RobotModelType::UNITREE_G1;
            } else if (modelParam == "wf_tron1a") {
                modelType_ = crl::humanoid::commons::RobotModelType::LIMX_WF_TRON1A;
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown model type: %s", modelParam.c_str());
                return false;
            }

            // Initialize MuJoCo now that parameters are loaded
            initializeMuJoCo();

            data_ = std::make_shared<RobotDataType>();

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
         * Add RGB arrows representing a coordinate frame to the MuJoCo scene.
         * @param position The position of the frame origin [x, y, z]
         * @param orientation The orientation quaternion [w, x, y, z]
         * @param arrowLength The length of the arrows
         */
        void addCoordinateFrameArrows(const double position[3], const double orientation[4], double arrowLength = 0.2) {
            // Extensive safety checks
            if (!mujocoModel_ || !mujocoData_) {
                return;
            }

            // Check if we have enough space (need 3 slots)
            if (scene_.ngeom > scene_.maxgeom - 3) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Not enough space in scene for arrows (ngeom=%d, maxgeom=%d)",
                                    scene_.ngeom, scene_.maxgeom);
                return;
            }

            // Validate input parameters
            if (!position || !orientation) {
                RCLCPP_ERROR(this->get_logger(), "Null position or orientation passed to addCoordinateFrameArrows");
                return;
            }

            // Validate quaternion is not degenerate
            double qnorm = sqrt(orientation[0]*orientation[0] + orientation[1]*orientation[1] +
                               orientation[2]*orientation[2] + orientation[3]*orientation[3]);
            if (qnorm < 0.9 || qnorm > 1.1) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Invalid quaternion norm: %f", qnorm);
                return;
            }

            // Convert quaternion to rotation matrix
            double rotMatrix[9];
            mju_quat2Mat(rotMatrix, orientation);

            // Define axis vectors in local frame (unit vectors along X, Y, Z)
            double axes[3][3] = {
                {1.0, 0.0, 0.0},  // X-axis (Red)
                {0.0, 1.0, 0.0},  // Y-axis (Green)
                {0.0, 0.0, 1.0}   // Z-axis (Blue)
            };

            // RGB colors for each axis (brighter colors)
            float colors[3][4] = {
                {1.0f, 0.2f, 0.2f, 1.0f},  // Brighter Red for X
                {0.2f, 1.0f, 0.2f, 1.0f},  // Brighter Green for Y
                {0.2f, 0.2f, 1.0f, 1.0f}   // Brighter Blue for Z
            };

            // Add an arrow for each axis
            for (int i = 0; i < 3; ++i) {
                // Double-check we still have space
                if (scene_.ngeom >= scene_.maxgeom) {
                    RCLCPP_WARN(this->get_logger(), "Scene full while adding arrow %d", i);
                    break;
                }

                mjvGeom* geom = &scene_.geoms[scene_.ngeom++];

                // Initialize all fields to safe defaults
                std::memset(geom, 0, sizeof(mjvGeom));

                // Set geom type to arrow
                geom->type = mjGEOM_ARROW;
                geom->dataid = -1;
                geom->objtype = mjOBJ_UNKNOWN;
                geom->objid = -1;
                geom->category = mjCAT_DECOR;
                geom->emission = 1.0;    // Full emission for maximum brightness
                geom->specular = 0.0;    // Maximum specular
                geom->shininess = 1.0;   // Maximum shininess
                geom->reflectance = 0.0;
                geom->label[0] = '\0';

                // Rotate the axis vector by the orientation matrix
                double axisWorld[3];
                mju_mulMatVec(axisWorld, rotMatrix, axes[i], 3, 3);

                // Calculate arrow start and end points
                double from[3] = {position[0], position[1], position[2]};
                double to[3] = {
                    position[0] + axisWorld[0] * arrowLength,
                    position[1] + axisWorld[1] * arrowLength,
                    position[2] + axisWorld[2] * arrowLength
                };

                // Set arrow midpoint position
                geom->pos[0] = from[0];
                geom->pos[1] = from[1];
                geom->pos[2] = from[2];

                // Calculate direction vector
                double dir[3] = {
                    to[0] - from[0],
                    to[1] - from[1],
                    to[2] - from[2]
                };

                // Normalize direction
                double length = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
                if (length > 1e-10) {
                    dir[0] /= length;
                    dir[1] /= length;
                    dir[2] /= length;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Zero-length arrow axis %d", i);
                    scene_.ngeom--; // Remove this geom
                    continue;
                }

                // Convert direction to quaternion (arrow points along z-axis in local frame)
                double quat[4];
                mju_quatZ2Vec(quat, dir);

                // Convert quaternion to rotation matrix (double precision)
                double matDouble[9];
                mju_quat2Mat(matDouble, quat);

                // Copy to geom->mat (which is float precision)
                for (int j = 0; j < 9; ++j) {
                    geom->mat[j] = static_cast<float>(matDouble[j]);
                }

                // Set arrow size (make them larger and more visible)
                geom->size[0] = arrowLength * 0.05;  // Arrow shaft radius
                geom->size[1] = arrowLength * 0.05;  // Arrow head radius
                geom->size[2] = arrowLength;   // Half-length

                // Set color
                geom->rgba[0] = colors[i][0];
                geom->rgba[1] = colors[i][1];
                geom->rgba[2] = colors[i][2];
                geom->rgba[3] = colors[i][3];
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

                // Add target frame visualization (RGB arrows)
                // Get target position and orientation from command
                double targetPos[3] = {
                    commandUI_.targetPositionX,
                    commandUI_.targetPositionY,
                    commandUI_.targetPositionZ
                };

                // Convert roll-pitch-yaw to quaternion (already in radians)
                double targetQuat[4];
                double euler[3] = {
                    commandUI_.targetOrientationRoll,
                    commandUI_.targetOrientationPitch,
                    commandUI_.targetOrientationYaw
                };
                mju_euler2Quat(targetQuat, euler, "XYZ");

                // Draw coordinate frame arrows at target location
                addCoordinateFrameArrows(targetPos, targetQuat, 0.3);

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

                // // Send FSM state change to ESTOP (restart)
                // fsmClient_.broadcast_switch(States::ESTOP);

                // Reset UI command values
                commandUI_.targetPositionX = 0;
                commandUI_.targetPositionY = 0;
                commandUI_.targetPositionZ = 0;
                commandUI_.targetOrientationRoll = 0;
                commandUI_.targetOrientationPitch = 0;
                commandUI_.targetOrientationYaw = 0;
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
                    auto state = data_->getRobotState();
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

        /**
         * Set command position and orientation directly.
         */
        void setPositionCommand(double positionX, double positionY, double positionZ) {
            commandUI_.targetPositionX = positionX;
            commandUI_.targetPositionY = positionY;
            commandUI_.targetPositionZ = positionZ;
            // Publish the updated command
            publishRemoteCommands();
        }

        /**
         * Increment command position by the given deltas.
         */
        void incrementPositionCommand(double deltaX, double deltaY, double deltaZ) {
            commandUI_.targetPositionX += deltaX;
            commandUI_.targetPositionY += deltaY;
            commandUI_.targetPositionZ += deltaZ;

            // Publish the updated command
            publishRemoteCommands();
        }

        /**
         * Set command orientation directly.
         */
        void setOrientationCommand(double roll, double pitch, double yaw) {
            commandUI_.targetOrientationRoll = roll;
            commandUI_.targetOrientationPitch = pitch;
            commandUI_.targetOrientationYaw = yaw;

            // wrap all to (-pi, pi]
            if (commandUI_.targetOrientationRoll > M_PI) {
                commandUI_.targetOrientationRoll -= 2 * M_PI;
            } else if (commandUI_.targetOrientationRoll <= -M_PI) {
                commandUI_.targetOrientationRoll += 2 * M_PI;
            }
            if (commandUI_.targetOrientationPitch > M_PI) {
                commandUI_.targetOrientationPitch -= 2 * M_PI;
            } else if (commandUI_.targetOrientationPitch <= -M_PI) {
                commandUI_.targetOrientationPitch += 2 * M_PI;
            }
            if (commandUI_.targetOrientationYaw > M_PI) {
                commandUI_.targetOrientationYaw -= 2 * M_PI;
            } else if (commandUI_.targetOrientationYaw <= -M_PI) {
                commandUI_.targetOrientationYaw += 2 * M_PI;
            }
            // Publish the updated command
            publishRemoteCommands();
        }

        /**
         * Increment command orientation by the given deltas.
         */
        void incrementOrientationCommand(double deltaRoll, double deltaPitch, double deltaYaw) {
            commandUI_.targetOrientationRoll += deltaRoll;
            commandUI_.targetOrientationPitch += deltaPitch;
            commandUI_.targetOrientationYaw += deltaYaw;

            // wrap all to (-pi, pi]
            if (commandUI_.targetOrientationRoll > M_PI) {
                commandUI_.targetOrientationRoll -= 2 * M_PI;
            } else if (commandUI_.targetOrientationRoll <= -M_PI) {
                commandUI_.targetOrientationRoll += 2 * M_PI;
            }
            if (commandUI_.targetOrientationPitch > M_PI) {
                commandUI_.targetOrientationPitch -= 2 * M_PI;
            } else if (commandUI_.targetOrientationPitch <= -M_PI) {
                commandUI_.targetOrientationPitch += 2 * M_PI;
            }
            if (commandUI_.targetOrientationYaw > M_PI) {
                commandUI_.targetOrientationYaw -= 2 * M_PI;
            } else if (commandUI_.targetOrientationYaw <= -M_PI) {
                commandUI_.targetOrientationYaw += 2 * M_PI;
            }

            // Publish the updated command
            publishRemoteCommands();
        }

        /**
         * Set command speed values directly.
         */
        void setSpeedCommand(double forwardSpeed, double sidewaysSpeed, double turningSpeed) {
            commandUI_.targetForwardSpeed = forwardSpeed;
            commandUI_.targetSidewaysSpeed = sidewaysSpeed;
            commandUI_.targetTurningSpeed = turningSpeed;
            // Publish the updated command
            publishRemoteCommands();
        }

        /**
         * Increment command speed values by the given deltas.
         */
        void incrementSpeedCommand(double deltaForward, double deltaSideways, double deltaTurning) {
            commandUI_.targetForwardSpeed += deltaForward;
            commandUI_.targetSidewaysSpeed += deltaSideways;
            commandUI_.targetTurningSpeed += deltaTurning;

            // // Clamp values to reasonable limits
            // const double maxForwardSpeed = 1.0; // m/s
            // const double maxSidewaysSpeed = 1.0; // m/s
            // const double maxTurningSpeed = 1.0; // rad/s

            // commandUI_.targetForwardSpeed = std::clamp(commandUI_.targetForwardSpeed, -maxForwardSpeed, maxForwardSpeed);
            // commandUI_.targetSidewaysSpeed = std::clamp(commandUI_.targetSidewaysSpeed, -maxSidewaysSpeed, maxSidewaysSpeed);
            // commandUI_.targetTurningSpeed = std::clamp(commandUI_.targetTurningSpeed, -maxTurningSpeed, maxTurningSpeed);

            // Publish the updated command
            publishRemoteCommands();
        }

        /**
         * Get current command values.
         */
        crl::humanoid::commons::RobotCommand getCommand() const {
            return commandUI_;
        }

    protected:
        /**
         * Initialize MuJoCo components (without GLFW window).
         */
        void initializeMuJoCo() {
            // Load the same model as the simulator
            std::string robotXmlFileParam = this->get_parameter("robot_xml_file").as_string();
            std::string xmlPath = std::string(CRL_HUMANOID_COMMONS_DATA_FOLDER) + "/robots/" + robotXmlFileParam;

            RCLCPP_INFO(this->get_logger(), "Loading MuJoCo model from: %s (parameter value: %s)", 
                       xmlPath.c_str(), robotXmlFileParam.c_str());

            char error[1000];
            mujocoModel_ = mj_loadXML(xmlPath.c_str(), nullptr, error, sizeof(error));

            if (!mujocoModel_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model from %s: %s", xmlPath.c_str(), error);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully loaded MuJoCo model with %d bodies, %d joints", 
                       mujocoModel_->nbody, mujocoModel_->njnt);

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
            crl::humanoid::commons::RobotState state;
            try {
                state = data_->getRobotState();
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

            // MuJoCo joint order (same as in SimNode.h) - conditional based on model type
            std::vector<std::string> mujocoJointOrder;
            if (modelType_ == crl::humanoid::commons::RobotModelType::UNITREE_G1) {
                // G1 joint order (29 joints)
                mujocoJointOrder = {
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
            } else if (modelType_ == crl::humanoid::commons::RobotModelType::LIMX_WF_TRON1A) {
                // TRON1A joint order (8 joints)
                mujocoJointOrder = {
                    "abad_L_Joint", "hip_L_Joint", "knee_L_Joint", "wheel_L_Joint",
                    "abad_R_Joint", "hip_R_Joint", "knee_R_Joint", "wheel_R_Joint"
                };
            } else {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Unknown model type in monitor, cannot determine joint order");
                return;
            }

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
                crl::humanoid::commons::RobotCommand command;
                crl::humanoid::commons::populateDataFromRemoteMessage(msg->remote, command);
                data_->setCommand(command);

                // Update sensor values
                auto sensor = data_->getSensor();
                crl::humanoid::commons::populateDataFromSensorMessage(msg->sensor, sensor);
                data_->setSensor(sensor);

                // Update state estimation
                auto state = data_->getRobotState();
                crl::humanoid::commons::populateDataFromStateMessage(msg->state, state);
                data_->setRobotState(state);

                // Update control
                auto control = data_->getControlSignal();
                crl::humanoid::commons::populateDataFromControlMessage(msg->control, control);
                data_->setControlSignal(control);

                // Update profiling info
                auto profilingInfo = data_->getProfilingInfo();
                crl::humanoid::commons::populateDataFromProfilingInfoMessage(msg->profiling_info, profilingInfo);
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
                crl::humanoid::commons::populateRemoteMessageFromData(commandUI_, message);
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
        crl::humanoid::commons::RobotModelType modelType_ = crl::humanoid::commons::RobotModelType::UNKNOWN;

        // Data pipeline
        std::shared_ptr<RobotDataType> data_ = nullptr;
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
        crl::humanoid::commons::RobotCommand commandUI_;

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
        typedef crl::humanoid::commons::RobotState State;
        typedef std::vector<crl::humanoid::commons::RobotSensor::RobotJointSensor> JointSensors;
        typedef std::vector<State::RobotJointState> JointStates;
        typedef std::vector<crl::humanoid::commons::RobotControlSignal::RobotJointControlSignal> JointControl;
        typedef crl::humanoid::commons::ProfilingInfo ProfilingInfo;
    };

    /**
     * Default MuJoCo monitor node.
     */
    template <typename States, typename Machines, typename TransitionsCont, std::size_t N>
    class MuJoCoMonitorNode final : public BaseMuJoCoMonitorNode<States, Machines, TransitionsCont, N, crl::humanoid::commons::RobotData> {
        using BaseRobotNode = BaseMuJoCoMonitorNode<States, Machines, TransitionsCont, N, crl::humanoid::commons::RobotData>;

    public:
        MuJoCoMonitorNode(const std::string& to_monitor, const TransitionsCont& trans_cont, const std::array<Machines, N>& monitoring)
            : BaseRobotNode(to_monitor, trans_cont, monitoring) {}

        ~MuJoCoMonitorNode() override = default;
    };

}  // namespace crl::humanoid::monitor
