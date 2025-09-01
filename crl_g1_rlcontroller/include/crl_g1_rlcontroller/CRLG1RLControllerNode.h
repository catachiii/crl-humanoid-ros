#ifndef CRL_G1_RLCONTROLLER_CONTROLLERNODE_H
#define CRL_G1_RLCONTROLLER_CONTROLLERNODE_H

#include "crl_g1_rlcontroller/CRLG1WalkController.h"
#include "crl_humanoid_commons/nodes/ControllerNode.h"

namespace crl::g1::rlcontroller {


    template <typename ControllerT = CRLG1WalkController>
    class CRLG1RLControllerNode final : public crl::humanoid::commons::ControllerNode<ControllerT> {
    public:
        using Base = crl::humanoid::commons::ControllerNode<ControllerT>;
        CRLG1RLControllerNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                              const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                              const std::string& nodeName="rlcontroller")
            : Base(model, data, nodeName) {
            // parameters
            auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
            paramDesc.description = "G1 RL controller parameters.";
            paramDesc.read_only = true;
            this->declare_parameter("model", "/models/g1_walk/g1_walk_config.json", paramDesc);

            if (!this->controller_->loadModelFromFile(CRL_G1_RLCONTROLLER_DATA_FOLDER + this->get_parameter("model").as_string())) {
                RCLCPP_FATAL(this->get_logger(), "Cannot load model file from %s", this->get_parameter("model").as_string().c_str());
            }
        }

        virtual ~CRLG1RLControllerNode() = default;

    private:
        void controlCallbackImpl() override {
            // compute and apply control signals
            this->controller_->computeAndApplyControlSignals(this->timeStepSize_);
        }
    };

}  // namespace crl::g1::rlcontroller

#endif  // CRL_G1_RLCONTROLLER_CONTROLLERNODE_H
