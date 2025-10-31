#ifndef CRL_G1_DEEPMIMICCONTROLLERNODE_H
#define CRL_G1_DEEPMIMICCONTROLLERNODE_H

#include "crl_g1_mimiccontroller/CRLG1RLControllerNode.h"
#include "crl_g1_mimiccontroller/CRLG1DeepMimicController.h"
#include "crl_fsm/client.h"

namespace crl::g1::rlcontroller {

    /**
     * Specialized controller node for CRLG1DeepMimicController that can trigger
     * FSM transitions when the deep mimic sequence is complete.
     */
    template <typename States, typename Machines, std::size_t N>
    class CRLG1DeepMimicControllerNode final : public CRLG1RLControllerNode<CRLG1DeepMimicController> {
    public:
        using Base = CRLG1RLControllerNode<CRLG1DeepMimicController>;

        CRLG1DeepMimicControllerNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                                     const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                                     const std::string& nodeName = "deepmimic_controller")
            : Base(model, data, nodeName) {

            // Additional parameters for transition behavior
            auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
            paramDesc.description = "DeepMimic transition parameters.";
            paramDesc.read_only = false;
            this->declare_parameter("auto_transition", true, paramDesc);
        }

        /**
         * Initialize the FSM client. This should be called after the FSM is set up.
         */
        template<typename TransitionsContGen>
        void initializeFsmClient(const TransitionsContGen& trans_cont, const std::array<Machines, N>& monitoring) {
            try {
                auto trans_inst = trans_cont();
                using TransitionsT = std::decay_t<decltype(trans_inst)>;
                using ClientT = crl::fsm::Client<States, Machines, TransitionsT, N>;
                auto client = std::make_shared<ClientT>("robot", trans_inst, monitoring);

                // Store both checked and unchecked (fire-and-forget) switch functions
                fsm_checked_switch_ = [client](States to){ return client->checked_broadcast_switch(to); };
                fsm_unchecked_switch_ = [client](States to){
                    // Fire-and-forget: just send the request without waiting
                    auto futures = client->broadcast_switch(to);
                    // Futures will be automatically destroyed, requests are sent but we don't wait
                };
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize FSM client: %s", e.what());
            }
        }

        virtual ~CRLG1DeepMimicControllerNode() = default;

    private:
        void controlCallbackImpl() override {
            // Call the parent implementation to compute and apply control signals
            Base::controlCallbackImpl();

            // Check if auto-transition is enabled
            if (!this->get_parameter("auto_transition").as_bool()) {
                return;
            }

            // Check if the deep mimic sequence is complete
            if (this->controller_ && this->controller_->isSequenceComplete()) {
                // Only attempt transition once per sequence completion
                if (!transitionAttempted_) {
                    transitionAttempted_ = true;
                    attemptTransitionToWalk();
                }
            } else {
                // Reset flag when sequence is not complete (e.g., when looping)
                transitionAttempted_ = false;
            }
        }

        void attemptTransitionToWalk() {
            try {
                // Check if FSM client is initialized
                if (!fsm_unchecked_switch_) {
                    RCLCPP_WARN(this->get_logger(), "FSM client not initialized, cannot transition to WALK");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Deep mimic sequence completed, initiating transition to WALK state...");

                // Get the WALK state - assuming it's the 3rd state (index 2) based on the FSM definition
                States walkState = States::from_ind(2); // WALK is typically the 3rd state (ESTOP=0, STAND=1, WALK=2)

                // Use fire-and-forget transition since this node will be terminated during the transition
                fsm_unchecked_switch_(walkState);

                RCLCPP_INFO(this->get_logger(), "Transition to WALK state initiated successfully");

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during transition to WALK: %s", e.what());
            }
        }

    private:
        std::function<crl::fsm::StateSwitchRes(States)> fsm_checked_switch_;
        std::function<void(States)> fsm_unchecked_switch_;
        bool transitionAttempted_ = false;
    };

}  // namespace crl::g1::rlcontroller

#endif  // CRL_G1_DEEPMIMICCONTROLLERNODE_H
