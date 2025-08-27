//
// Created by Dongho Kang on 31.03.23.
//

#ifndef CRL_HUMANOID_COMMNODE_H
#define CRL_HUMANOID_COMMNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"
#include "crl_humanoid_msgs/msg/monitor.hpp"
#include "crl_humanoid_msgs/msg/remote.hpp"

namespace crl::unitree::commons {

    /**
     * High-level communication node.
     */
    class CommNode : public BaseNode {
    public:
        CommNode(const UnitreeRobotModel &model,
                 const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData> &data);

        ~CommNode() override = default;

    protected:
        virtual void commCallbackImpl();

    private:
        void timerCallbackImpl() override;

        void remoteSubscriptionCallback(const crl_humanoid_msgs::msg::Remote::SharedPtr msg);

    private:
        // for time stamp
        rclcpp::Time startTime_;

        // pub and sub
        rclcpp::Publisher<crl_humanoid_msgs::msg::Monitor>::SharedPtr monitorPublisher_;
        rclcpp::Subscription<crl_humanoid_msgs::msg::Remote>::SharedPtr remoteSubscription_;
    };

}  // namespace crl::unitree::commons

#endif
