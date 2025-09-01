//
// Created by Dongho Kang on 31.03.23.
//

#include "crl_humanoid_commons/nodes/CommNode.h"

#include "crl_humanoid_commons/helpers/MessageHelper.h"

namespace crl::humanoid::commons {

    CommNode::CommNode(const std::shared_ptr<RobotModel>& model, const std::shared_ptr<RobotData>& data)
            : BaseNode(model, data, "comm") {
        // load configuration
        auto queueSizeParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
        queueSizeParamDesc.read_only = true;
        queueSizeParamDesc.description = "Queue size for topics (read only.)";
        this->declare_parameter<int>("monitorPublisherQueueSize", 10, queueSizeParamDesc);
        this->declare_parameter<int>("remoteSubscriberQueueSize", 10, queueSizeParamDesc);

        // update start timer
        startTime_ = this->now();

        // communication
        monitorPublisher_ = this->create_publisher<crl_humanoid_msgs::msg::Monitor>("monitor", this->get_parameter("monitorPublisherQueueSize").as_int());
        remoteSubscription_ = this->create_safe_subscription<crl_humanoid_msgs::msg::Remote>(
                "remote", this->get_parameter("remoteSubscriberQueueSize").as_int(), std::bind(&CommNode::remoteSubscriptionCallback, this, std::placeholders::_1));
    }

    void CommNode::timerCallbackImpl() {
        auto start = this->now();

        // call comm callback
        commCallbackImpl();

        // populate execution time (for profiling)
        auto profileInfo = data_->getProfilingInfo();
        profileInfo.communicationExecutionTime = (this->now() - start).seconds();
        data_->setProfilingInfo(profileInfo);
    }

    void CommNode::commCallbackImpl() {
        // populate messages
        // populate timestamp
        double t = data_->getTimeStamp();
        int tSec = (int)t;
        int tNanoSec = double(t - tSec) * 1e9;
        auto message = crl_humanoid_msgs::msg::Monitor();
        message.header.stamp = rclcpp::Time(tSec, tNanoSec);
        // populate data
        const auto& command = data_->getCommand();
        populateRemoteMessageFromData(command, message.remote);
        const auto& sensorData = data_->getSensor();
        populateSensorMessageFromData(sensorData, message.sensor);
        const auto& stateData = data_->getRobotState();
        populateStateMessageFromData(stateData, message.state);
        const auto& controlData = data_->getControlSignal();
        populateControlMessageFromData(controlData, message.control);
        const auto& profilingData = data_->getProfilingInfo();
        populateProfilingInfoMessageFromData(profilingData, message.profiling_info);

        // publish message
        monitorPublisher_->publish(message);
    }

    void CommNode::remoteSubscriptionCallback(const crl_humanoid_msgs::msg::Remote::SharedPtr msg) {
        auto comm = data_->getCommand();
        crl::humanoid::commons::populateDataFromRemoteMessage(*msg, comm);
        data_->setCommand(comm);
    }

}  // namespace crl::humanoid::commons
