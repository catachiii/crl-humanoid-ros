#include "crl_humanoid_commons/nodes/MocapNode.h"

namespace crl::humanoid::commons {

    MocapNode::MocapNode(const std::shared_ptr<RobotModel>& model, const std::shared_ptr<RobotData>& data)
            : BaseNode(model, data, "mocap") {
        // load configuration
        auto queueSizeParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
        queueSizeParamDesc.read_only = true;
        queueSizeParamDesc.description = "Queue size for mocap topic subscription (read only.)";
        this->declare_parameter<int>("mocapSubscriberQueueSize", 10, queueSizeParamDesc);

        // declare rigid body ID parameter
        auto rigidBodyIdParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
        rigidBodyIdParamDesc.description = "Rigid body ID to track from OptiTrack system";
        this->declare_parameter<int>("rigid_body_id", 0, rigidBodyIdParamDesc);
        rigidBodyId_ = this->get_parameter("rigid_body_id").as_int();

        // declare offset parameter
        auto offsetParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
        offsetParamDesc.description = "Offset to apply to the mocap position [x, y, z]";
        this->declare_parameter<std::vector<double>>("offset", std::vector<double>{0.0, 0.0, 0.0}, offsetParamDesc);
        auto offsetVec = this->get_parameter("offset").as_double_array();
        if (offsetVec.size() == 3) {
            offset_ = P3D(offsetVec[0], offsetVec[1], offsetVec[2]);
            RCLCPP_INFO(this->get_logger(), "Mocap offset set to: [%.3f, %.3f, %.3f]", offset_.x, offset_.y, offset_.z);
        } else {
            offset_ = P3D(0.0, 0.0, 0.0);
            RCLCPP_WARN(this->get_logger(), "Invalid offset parameter size. Using default [0.0, 0.0, 0.0]");
        }

        // create subscription
        mocapSubscription_ = this->create_safe_subscription<optitrack_msgs::msg::MocapFrameData>(
                "/optitrack_adaptor/mocap_frame",
                this->get_parameter("mocapSubscriberQueueSize").as_int(),
                std::bind(&MocapNode::mocapSubscriptionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MocapNode initialized with rigid_body_id: %d", rigidBodyId_);
    }

    void MocapNode::timerCallbackImpl() {
        // call mocap callback
        mocapCallbackImpl();
    }

    void MocapNode::mocapCallbackImpl() {
        // This method can be overridden in derived classes if needed
        // Currently, all processing is done in the subscription callback
    }

    void MocapNode::mocapSubscriptionCallback(const optitrack_msgs::msg::MocapFrameData::SharedPtr msg) {
        // Get current sensor data and state
        auto sensorData = data_->getSensor();
        auto stateData = data_->getRobotState();

        // Search for the rigid body with the specified ID
        bool found = false;
        for (const auto& rigidbody : msg->rigidbodies) {
            if (rigidbody.id == rigidBodyId_) {
                // Apply offset to mocap position
                P3D position = P3D(rigidbody.x, rigidbody.y, rigidbody.z) + offset_;

                // Update mocap pose in sensor
                sensorData.mocapPose.position = position;
                sensorData.mocapPose.orientation = Quaternion(rigidbody.qw, rigidbody.qx, rigidbody.qy, rigidbody.qz);
                sensorData.mocapPose.valid = true;

                // Sync mocap pose to robot state
                stateData.basePosition = position;
                stateData.baseOrientation = Quaternion(rigidbody.qw, rigidbody.qx, rigidbody.qy, rigidbody.qz);

                found = true;
                break;
            }
        }

        // If rigid body not found, mark as invalid
        if (!found) {
            sensorData.mocapPose.valid = false;
        }

        // Update sensor data and state
        data_->setSensor(sensorData);
        data_->setRobotState(stateData);
    }

}  // namespace crl::humanoid::commons
