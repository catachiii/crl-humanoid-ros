//
// Created by Dongho Kang on 07.05.22.
//

#ifndef CRL_HUMANOID_BASENODE_H
#define CRL_HUMANOID_BASENODE_H

#include <atomic>

#include "crl_ros_helper/node.h"
#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/helpers/ConfigurationHelper.h"
#include "rclcpp/rclcpp.hpp"

namespace crl::unitree::commons {

    /**
     * Base class of ros nodes.
     */
    class BaseNode : public crl::ros::Node {
    public:
        /**
         * model is a struct contains basic info of robot model.
         * callbackRate is in Hz.
         */
        BaseNode(const UnitreeRobotModel& model, const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData>& data, const std::string& nodeName)
            : crl::ros::Node(nodeName), model_(model), data_(data) {

            // Note: Joint information is now initialized in the UnitreeLeggedRobotData constructor
            // No need to call PropagateJointInformation here anymore

            // parameters
            auto timeStepSizeParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
            timeStepSizeParamDesc.read_only = true;
            timeStepSizeParamDesc.description = "Time interval of timer callback (read only.)";
            this->declare_parameter<double>("time_step_size", 0.01, timeStepSizeParamDesc);
            timeStepSize_ = this->get_parameter("time_step_size").as_double();

            // initialize timer and add_on_set_parameter callback
            callbackTimer_ = this->create_safe_wall_timer(std::chrono::duration<double>(timeStepSize_), std::bind(&BaseNode::timerCallback, this));
            parameterCallbackHandle_ = this->add_on_set_parameters_callback(std::bind(&BaseNode::parameterCallback, this, std::placeholders::_1));
        }

        virtual ~BaseNode() = default;

        virtual void wait_until_startable() {
            // currently do nothing
        }

        virtual void wait_until_terminatable() {
            terminate();
        }

    protected:
        /**
         * One iteration-logic implementation. This function is called by timer Callback.
         * Derived classes should implement this function.
         */
        virtual void timerCallbackImpl() = 0;

        /**
         * Parameter callback implementation. This function is called by parameter Callback.
         * Derive class should override this function to implement parameter accept or reject logic here.
         * IMPORTANT: do not actually set the given parameter to algorithms.
         */
        virtual rcl_interfaces::msg::SetParametersResult parameterCallbackImpl(const std::vector<rclcpp::Parameter>& parameters) const {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;
            res.reason = "";
            return res;
        }

    private:
        /**
         * Callback function called by timer with fixed time interval.
         */
        void timerCallback() {
            timerCallbackImpl();
        }

        /**
         * Callback function called when parameter of node changed.
         */
        rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters) {
            return parameterCallbackImpl(parameters);
        }

    protected:
        const UnitreeRobotModel model_;
        const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData> data_ = nullptr;
        double timeStepSize_ = 0.01;  // it's not good idea to change this value during the operation!
        rclcpp::TimerBase::SharedPtr callbackTimer_;
        OnSetParametersCallbackHandle::SharedPtr parameterCallbackHandle_ = nullptr;
    };

}  // namespace crl::unitree::commons

#endif  //CRL_HUMANOID_BASENODE_H
