//
// Created by Jin Cheng on 25.04.24.
//
#include "crl_humanoid_commons/nodes/PyControllerNode.h"

#include "crl_humanoid_commons/helpers/MessageHelper.h"

namespace crl::unitree::commons {

    PyControllerNode::PyControllerNode(const UnitreeRobotModel &model,
                                       const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData> &data)
            : BaseNode(model, data, "py_controller") {
        // load configuration
        auto queueSizeParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
        queueSizeParamDesc.read_only = true;
        queueSizeParamDesc.description = "Queue size for topics (read only.)";
        this->declare_parameter<int>("pyStatePublisherQueueSize", 10, queueSizeParamDesc);
        this->declare_parameter<int>("pyControlSubscriberQueueSize", 10, queueSizeParamDesc);

        // update start timer
        startTime_ = this->now();

        // communication
        pyStatePublisher_ = this->create_publisher<crl_humanoid_msgs::msg::PyState>("py_state", this->get_parameter(
                "pyStatePublisherQueueSize").as_int());
        pyControlSubscription_ = this->create_safe_subscription<crl_humanoid_msgs::msg::PyControl>(
                "py_control", this->get_parameter("pyControlSubscriberQueueSize").as_int(),
                std::bind(&PyControllerNode::PyControlSubscriptionCallback, this, std::placeholders::_1));
    }

    void PyControllerNode::timerCallbackImpl() {
        auto start = this->now();

        // populate messages
        // populate timestamp
        double t = data_->getTimeStamp();
        int tSec = (int) t;
        int tNanoSec = double(t - tSec) * 1e9;
        auto message = crl_humanoid_msgs::msg::PyState();
        message.header.stamp = rclcpp::Time(tSec, tNanoSec);

        message.header.frame_id = std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        // populate data
        const auto &command = data_->getCommand();
        populateRemoteMessageFromData(command, message.remote);
        const auto &sensorData = data_->getSensor();
        populateSensorMessageFromData(sensorData, message.sensor);
        const auto &stateData = data_->getLeggedRobotState();
        populateStateMessageFromData(stateData, message.state);
        const auto &controlData = data_->getControlSignal();
        populateControlMessageFromData(controlData, message.control);

        // publish message
        pyStatePublisher_->publish(message);
    }

    void PyControllerNode::PyControlSubscriptionCallback(const crl_humanoid_msgs::msg::PyControl::SharedPtr msg) {
        // int currTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // int pubTime = std::stol(msg->header.frame_id);
        // RCLCPP_INFO(this->get_logger(), "py2cpp time_diff: %f", (currTime - pubTime) / 1e9);
        auto control_signal = data_->getControlSignal();
        crl::unitree::commons::populateDataFromPyControlMessage(*msg, control_signal);
        data_->setControlSignal(control_signal);
    }

}  // namespace crl::unitree::commons
