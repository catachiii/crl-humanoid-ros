//
// Created by Jin Cheng on 25.04.24.
//

#ifndef CRL_HUMANOID_PYCONTROLLERNODE_H
#define CRL_HUMANOID_PYCONTROLLERNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"
#include "crl_humanoid_msgs/msg/py_state.hpp"
#include "crl_humanoid_msgs/msg/py_control.hpp"

namespace crl::unitree::commons {

    /**
     * High-level communication node.
     */
    class PyControllerNode : public BaseNode {
    public:
        PyControllerNode(const UnitreeRobotModel &model,
                         const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData> &data);

        ~PyControllerNode() override = default;

    private:
        void timerCallbackImpl() override;

        void PyControlSubscriptionCallback(const crl_humanoid_msgs::msg::PyControl::SharedPtr msg);

    private:

        // for time stamp
        rclcpp::Time startTime_;

        // pub and sub
        rclcpp::Publisher<crl_humanoid_msgs::msg::PyState>::SharedPtr pyStatePublisher_;
        rclcpp::Subscription<crl_humanoid_msgs::msg::PyControl>::SharedPtr pyControlSubscription_;
    };

}  // namespace crl::unitree::commons

#endif
