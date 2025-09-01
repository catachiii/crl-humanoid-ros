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

namespace crl::humanoid::commons {

    template <typename States, typename Machines, std::size_t N>
    class RobotNode : public BaseNode {
    public:
        RobotNode(const std::shared_ptr<RobotModel>& model, const std::shared_ptr<RobotData>& data,
                  const std::array<Machines, N>& monitoring, const std::atomic<bool>& is_transitioning)
            : BaseNode(model, data, "robot"),
              fsm_broadcaster("robot", monitoring),
              fsm_state_informer("robot", monitoring),
              is_transitioning(is_transitioning) {
            // joint parameters
            jointCount_ = this->model_->jointNames.size();
            jointPosMax_.resize(jointCount_);
            jointPosMin_.resize(jointCount_);
            jointVelMax_.resize(jointCount_);
            jointTorqueMax_.resize(jointCount_);
            jointStiffnessDefault_.resize(jointCount_);
            jointDampingDefault_.resize(jointCount_);

            // parameters
            auto jointParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
            jointParamDesc.read_only = false;
            jointParamDesc.description = "Robot joint parameters";
            this->template declare_parameter<std::vector<double>>("joint_position_max", model->jointPosMax, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_position_min", model->jointPosMin, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_velocity_max", model->jointVelMax, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_torque_max", model->jointTorqueMax, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_stiffness_default", model->jointStiffnessDefault, jointParamDesc);
            this->template declare_parameter<std::vector<double>>("joint_damping_default", model->jointDampingDefault, jointParamDesc);

            // joint gains
            for (int i = 0; i < jointCount_; i++) {
                jointPosMax_[i] = this->get_parameter("joint_position_max").as_double_array()[i];
                jointPosMin_[i] = this->get_parameter("joint_position_min").as_double_array()[i];
                jointVelMax_[i] = this->get_parameter("joint_velocity_max").as_double_array()[i];
                jointTorqueMax_[i] = this->get_parameter("joint_torque_max").as_double_array()[i];
                jointStiffnessDefault_[i] = this->get_parameter("joint_stiffness_default").as_double_array()[i];
                jointDampingDefault_[i] = this->get_parameter("joint_damping_default").as_double_array()[i];
            }

            // initialize command so it does not do anything when starting
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
         * Read sensors through robot's API call.
         */
        virtual void updateDataWithSensorReadings() = 0;

        /**
         * Send command through robot's API call.
         */
        virtual void updateCommandWithData() = 0;

        virtual void resetRobot(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model) {
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
            if (model_->modelType == RobotModelType::UNITREE_G1) {
                response->model = (int8_t)crl::humanoid::commons::RobotModelType::UNITREE_G1;
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

            // get robot state through robot API
            updateDataWithSensorReadings();

            // synchronize robot model with state estimation
            auto state = this->data_->getRobotState();

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
        int jointCount_ = 0;
        std::vector<double> jointPosMax_;
        std::vector<double> jointPosMin_;
        std::vector<double> jointVelMax_;
        std::vector<double> jointTorqueMax_;
        std::vector<double> jointStiffnessDefault_;
        std::vector<double> jointDampingDefault_;

        // fsm
        crl::fsm::Broadcaster<States, Machines, N> fsm_broadcaster;
        crl::fsm::StateInformer<States, Machines, N> fsm_state_informer;
        const std::atomic<bool>& is_transitioning;

        // services
        rclcpp::Service<crl_humanoid_msgs::srv::Model>::SharedPtr modelService_ = nullptr;
        rclcpp::Service<crl_humanoid_msgs::srv::Restart>::SharedPtr restartService_ = nullptr;
    };

}  // namespace crl::humanoid::commons

#endif  //CRL_HUMANOID_ROBOT_NODE
