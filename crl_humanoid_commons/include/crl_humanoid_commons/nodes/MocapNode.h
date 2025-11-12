//
// Created by MocapNode on 12.11.25.
//

#ifndef CRL_HUMANOID_MOCAPNODE_H
#define CRL_HUMANOID_MOCAPNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"
#include "optitrack_msgs/msg/mocap_frame_data.hpp"

namespace crl::humanoid::commons {

    /**
     * Motion capture node to receive OptiTrack data.
     */
    class MocapNode : public BaseNode {
    public:
        MocapNode(const std::shared_ptr<RobotModel> &model,
                  const std::shared_ptr<RobotData> &data);

        ~MocapNode() override = default;

    protected:
        virtual void mocapCallbackImpl();

    private:
        void timerCallbackImpl() override;

        void mocapSubscriptionCallback(const optitrack_msgs::msg::MocapFrameData::SharedPtr msg);

    private:
        // ROS parameter for rigid body ID
        int rigidBodyId_;

        // Subscription
        rclcpp::Subscription<optitrack_msgs::msg::MocapFrameData>::SharedPtr mocapSubscription_;
    };

}  // namespace crl::humanoid::commons

#endif  // CRL_HUMANOID_MOCAPNODE_H
