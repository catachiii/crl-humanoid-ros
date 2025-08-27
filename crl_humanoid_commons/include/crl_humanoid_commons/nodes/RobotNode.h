//
// Created by Dongho Kang on 06.05.23.
//

#ifndef CRL_HUMANOID_ROBOT_NODE
#define CRL_HUMANOID_ROBOT_NODE

#include "crl_humanoid_commons/nodes/BaseNode.h"

// crl_fsm
#include <crl_fsm/client.h>

// crl_humanoid_msgs
#include "crl_humanoid_msgs/srv/model.hpp"
#include "crl_humanoid_msgs/srv/ping.hpp"
#include "crl_humanoid_msgs/srv/restart.hpp"

namespace crl::unitree::commons {

    template <typename States, typename Machines, std::size_t N>
    class RobotNode : public BaseNode {
    public:
        RobotNode(const crl::unitree::commons::UnitreeRobotModel& model, const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData>& data,
                  const std::array<Machines, N>& monitoring, const std::atomic<bool>& is_transitioning)
            : BaseNode(model, data, "robot"),
              fsm_broadcaster("robot", monitoring),
              fsm_state_informer("robot", monitoring),
              is_transitioning(is_transitioning) {
            // joint parameters
            jointCount = this->model_.jointNames.size();
            JOINT_POSITION_CONTROL_KP.resize(jointCount);
            JOINT_POSITION_CONTROL_KD.resize(jointCount);
            JOINT_TORQUE_CONTROL_KP.resize(jointCount);
            JOINT_TORQUE_CONTROL_KD.resize(jointCount);
            JOINT_POSITION_MAX.resize(jointCount);
            JOINT_POSITION_MIN.resize(jointCount);
            JOINT_VELOCITY_MAX.resize(jointCount);
            JOINT_TORQUE_MAX.resize(jointCount);

            // parameters
            auto jointParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
            jointParamDesc.read_only = false;
            jointParamDesc.description = "Robot joint parameters";
            this->template declare_parameter<std::vector<double>>("joint_position_max", JOINT_POSITION_MAX, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_position_min", JOINT_POSITION_MIN, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_velocity_max", JOINT_VELOCITY_MAX, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_torque_max", JOINT_TORQUE_MAX, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_position_control_kp", JOINT_POSITION_CONTROL_KP, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_position_control_kd", JOINT_POSITION_CONTROL_KD, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_torque_control_kp", JOINT_TORQUE_CONTROL_KP, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_torque_control_kd", JOINT_TORQUE_CONTROL_KD, jointParamDesc);
            this->template declare_parameter<double>("joystick_forward_max", joystickMaximumForwardVelocity, jointParamDesc);
            this->template declare_parameter<double>("joystick_backward_max", joystickMaximumBackwardVelocity, jointParamDesc);
            this->template declare_parameter<double>("joystick_sideways_max", joystickMaximumSidewaysVelocity, jointParamDesc);
            this->template declare_parameter<double>("joystick_turning_max", joystickMaximumTurningVelocity, jointParamDesc);

            // initialize command according to robot model
            auto comm = data_->getCommand();
            comm.targetForwardSpeed = 0;
            comm.targetSidewaysSpeed = 0;
            comm.targetTurningSpeed = 0;
            data_->setCommand(comm);

            // model service (for monitor)
            modelService_ = this->create_safe_service<crl_humanoid_msgs::srv::Model>(
                "model", std::bind(&RobotNode::modelServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

            restartService_ = this->create_safe_service<crl_humanoid_msgs::srv::Restart>(
                "restart", std::bind(&RobotNode::restartServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

            // ping service client
            // TODO: see https://github.com/ros2/examples/blob/humble/rclcpp/services/async_client/main.cpp
        }

        ~RobotNode() override = default;

    protected:
        /**
         * Apply rosparam to robot.
         */
        virtual void applyRobotParameters() {
            // joystick gains
            joystickMaximumForwardVelocity = this->get_parameter("joystick_forward_max").as_double();
            joystickMaximumBackwardVelocity = this->get_parameter("joystick_backward_max").as_double();
            joystickMaximumSidewaysVelocity = this->get_parameter("joystick_sideways_max").as_double();
            joystickMaximumTurningVelocity = this->get_parameter("joystick_turning_max").as_double();

            // joint gains
            for (int i = 0; i < jointCount; i++) {
                JOINT_POSITION_MAX[i] = this->get_parameter("joint_position_max").as_double_array()[i];
                JOINT_POSITION_MIN[i] = this->get_parameter("joint_position_min").as_double_array()[i];
                JOINT_VELOCITY_MAX[i] = this->get_parameter("joint_velocity_max").as_double_array()[i];
                JOINT_TORQUE_MAX[i] = this->get_parameter("joint_torque_max").as_double_array()[i];
                JOINT_POSITION_CONTROL_KP[i] = this->get_parameter("joint_position_control_kp").as_double_array()[i];
                JOINT_POSITION_CONTROL_KD[i] = this->get_parameter("joint_position_control_kd").as_double_array()[i];
                JOINT_TORQUE_CONTROL_KP[i] = this->get_parameter("joint_torque_control_kp").as_double_array()[i];
                JOINT_TORQUE_CONTROL_KD[i] = this->get_parameter("joint_torque_control_kd").as_double_array()[i];
            }
        };

        /**
         * Read sensors through robot's API call.
         */
        virtual void updateDataWithSensorReadings() = 0;

        /**
         * Send command through robot's API call.
         */
        virtual void updateCommandWithData() = 0;

        virtual void resetRobot(const crl::unitree::commons::UnitreeRobotModel& model) {
            // do nothing
            RCLCPP_WARN(this->get_logger(), "Reset robot is not implemented.");
        }

    private:
        void restartServiceCallback(const std::shared_ptr<crl_humanoid_msgs::srv::Restart::Request> request,
                                    std::shared_ptr<crl_humanoid_msgs::srv::Restart::Response> response) {
            resetRobot(model_);
            response->success = true;
        }

        void modelServiceCallback(const std::shared_ptr<crl_humanoid_msgs::srv::Model::Request> request,
                                  std::shared_ptr<crl_humanoid_msgs::srv::Model::Response> response) {
            if (model_.modelType == RobotModelType::UNITREE_G1) {
                response->model = (int8_t)crl::unitree::commons::RobotModelType::UNITREE_G1;
            } else {
                RCLCPP_FATAL(this->get_logger(), "Unknown robot model: model is not initialized properly.");
            }
            RCLCPP_INFO(this->get_logger(), "Robot model queried.");
        }

    protected:
        void timerCallbackImpl() override {
            auto start = this->now();

            // inform robot starts
            static bool initialized;
            if (!initialized) {
                initialized = true;
                RCLCPP_INFO(this->get_logger(), "Start robot.");
            }

            // apply robot parameters
            applyRobotParameters();

            // get robot state through robot API
            updateDataWithSensorReadings();

            // synchronize robot model with state estimation
            auto state = this->data_->getLeggedRobotState();

            // send command through robot API
            updateCommandWithData();

            // update timestamp
            this->data_->advanceInTime(this->timeStepSize_);

            // populate execution time (for profiling)
            auto profileInfo = this->data_->getProfilingInfo();
            profileInfo.mainExecutionTime = (this->now() - start).seconds();
            this->data_->setProfilingInfo(profileInfo);
        }

    protected:
        // joint parameters
        int jointCount = 0;
        std::vector<double> JOINT_POSITION_CONTROL_KP;
        std::vector<double> JOINT_POSITION_CONTROL_KD;
        std::vector<double> JOINT_TORQUE_CONTROL_KP;
        std::vector<double> JOINT_TORQUE_CONTROL_KD;
        std::vector<double> JOINT_POSITION_MAX;
        std::vector<double> JOINT_POSITION_MIN;
        std::vector<double> JOINT_VELOCITY_MAX;
        std::vector<double> JOINT_TORQUE_MAX;

        // joystick parameters
        double joystickMaximumForwardVelocity = 1.0;   // [m/s]
        double joystickMaximumBackwardVelocity = 1.0;  // [m/s]
        double joystickMaximumSidewaysVelocity = 1.0;  // [rad/s]
        double joystickMaximumTurningVelocity = 1.0;   // [rad/s]

        // fsm
        crl::fsm::Broadcaster<States, Machines, N> fsm_broadcaster;
        crl::fsm::StateInformer<States, Machines, N> fsm_state_informer;
        const std::atomic<bool>& is_transitioning;

        // services
        rclcpp::Service<crl_humanoid_msgs::srv::Model>::SharedPtr modelService_ = nullptr;
        rclcpp::Service<crl_humanoid_msgs::srv::Restart>::SharedPtr restartService_ = nullptr;
    };

}  // namespace crl::unitree::commons

#endif  //CRL_HUMANOID_ROBOT_NODE
