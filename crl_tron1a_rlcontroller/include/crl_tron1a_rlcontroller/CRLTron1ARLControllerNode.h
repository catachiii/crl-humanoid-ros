#ifndef CRL_G1_RLCONTROLLER_CONTROLLERNODE_H
#define CRL_G1_RLCONTROLLER_CONTROLLERNODE_H

#include <string>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "crl_tron1a_rlcontroller/CRLTron1AWalkController.h"
#include "crl_humanoid_commons/nodes/ControllerNode.h"

namespace crl::tron1a::rlcontroller {


    template <typename ControllerT = CRLTron1AWalkController>
    class CRLTron1ARLControllerNode : public crl::humanoid::commons::ControllerNode<ControllerT> {
    public:
        using Base = crl::humanoid::commons::ControllerNode<ControllerT>;
        CRLTron1ARLControllerNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                              const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                              const std::string& nodeName="rlcontroller")
            : Base(model, data, nodeName) {
            // parameters
            auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
            paramDesc.description = "Tron1a RL controller parameters.";
            paramDesc.read_only = true;
            // Optional params for direct ONNX model loading (Wheelfoot-style)
            // Required now: we do not support JSON fallback in this node
            if (!this->has_parameter("robot_controllers_policy_file")) {
                this->template declare_parameter<std::string>("robot_controllers_policy_file", "");
            }
            if (!this->has_parameter("robot_controllers_encoder_file")) {
                this->template declare_parameter<std::string>("robot_controllers_encoder_file", "");
            }

            std::string policyPath;
            std::string encoderPath;
            // Require param-based model loading
            bool havePolicy = this->get_parameter("robot_controllers_policy_file", policyPath) && !policyPath.empty();
            bool haveEncoder = this->get_parameter("robot_controllers_encoder_file", encoderPath) && !encoderPath.empty();
            if (havePolicy && haveEncoder) {
                if (!this->controller_->loadModelFromParams(*this)) {
                    RCLCPP_FATAL(this->get_logger(), "Cannot load ONNX models from params (policy: %s, encoder: %s)", policyPath.c_str(), encoderPath.c_str());
                }
            } else {
                if (!havePolicy) {
                    RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_controllers_policy_file' is required for CRLTron1ARLControllerNode");
                }
                if (!haveEncoder) {
                    RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_controllers_encoder_file' is required for CRLTron1ARLControllerNode");
                }
            }
        }

        virtual ~CRLTron1ARLControllerNode() = default;

    protected:
        void controlCallbackImpl() override {
            // compute and apply control signals
            this->controller_->computeAndApplyControlSignals(this->timeStepSize_);
        }
    };

}  // namespace crl::tron1a::rlcontroller

#endif  // CRL_TRON1A_RLCONTROLLER_CONTROLLERNODE_H
