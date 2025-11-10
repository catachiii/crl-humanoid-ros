#include "crl_humanoid_commons/nodes/PyControllerNode.h"
#include "crl_humanoid_commons/helpers/MessageHelper.h"

namespace crl::humanoid::commons {

    PyControllerNode::PyControllerNode(const std::shared_ptr<RobotModel> &model,
                                       const std::shared_ptr<RobotData> &data)
            : BaseNode(model, data, "py_controller") {

        // load configuration
        auto queueSizeParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
        queueSizeParamDesc.read_only = true;
        queueSizeParamDesc.description = "Queue size for topics (read only.)";
        this->declare_parameter<int>("pyControlSubscriberQueueSize", 10, queueSizeParamDesc);

        // Initialize subscriber for control messages
        controlSubscription_ = this->create_subscription<crl_humanoid_msgs::msg::Control>(
                "py_control",
                this->get_parameter("pyControlSubscriberQueueSize").as_int(),
                std::bind(&PyControllerNode::controlSubscriptionCallback, this, std::placeholders::_1)
        );
    }

    void PyControllerNode::timerCallbackImpl() {
        // This node is passive - it only processes control messages from subscriptions
        // No active timer-based processing is required
    }

    void PyControllerNode::controlSubscriptionCallback(const crl_humanoid_msgs::msg::Control::SharedPtr msg) {
        try {
            if (!data_) return;

            // Update control signal from the received message
            auto control = data_->getControlSignal();
            crl::humanoid::commons::populateDataFromControlMessage(*msg, control);
            data_->setControlSignal(control);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in controlSubscriptionCallback: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception in controlSubscriptionCallback");
        }
    }

}  // namespace crl::humanoid::commons
