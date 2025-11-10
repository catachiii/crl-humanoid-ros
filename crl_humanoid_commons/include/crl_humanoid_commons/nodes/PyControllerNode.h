#ifndef CRL_HUMANOID_COMMONS_NODES_PYCONTROLLERNODE_H
#define CRL_HUMANOID_COMMONS_NODES_PYCONTROLLERNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"
#include "crl_humanoid_msgs/msg/control.hpp"

namespace crl::humanoid::commons {

    /**
     * Python controller node.
     */
    class PyControllerNode : public BaseNode {
    public:
        PyControllerNode(const std::shared_ptr<RobotModel> &model,
                         const std::shared_ptr<RobotData> &data);

        ~PyControllerNode() override = default;

    protected:
        void timerCallbackImpl() override;

    private:

        void controlSubscriptionCallback(const crl_humanoid_msgs::msg::Control::SharedPtr msg);

    private:
        // pub and sub
        rclcpp::Subscription<crl_humanoid_msgs::msg::Control>::SharedPtr controlSubscription_;
    };

}  // namespace crl::humanoid::commons

#endif  // CRL_HUMANOID_COMMONS_NODES_PYCONTROLLERNODE_H
