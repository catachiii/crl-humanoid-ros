//
// Created by Dongho Kang on 09.04.23.
//

#ifndef CRL_HUMANOID_STARTERNODE_H
#define CRL_HUMANOID_STARTERNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"

namespace crl::unitree::commons {

        /**
         * Stand logic for unitree robots.
         */
        class StarterNode : public BaseNode {
            public:
            enum class TargetMode { ZERO = 0, STAND = 1 };

            public:
            StarterNode(const StarterNode::TargetMode target, const UnitreeRobotModel& model,
            const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData>& data, const std::string& nodeName);

            ~StarterNode() = default;

            private:
            void timerCallbackImpl() override;

            protected:
            int jointCount_ = 0;  // number of joints
            // joint angles at the transition
            crl::dVector initialJointAngle_;
            // target joint angles
            crl::dVector targetJointAngle_;
            // initialized flags
            double timeAtTransition_ = 0.0;
            bool initialized_ = false;
        };

}  // namespace crl::unitree::commons

#endif
