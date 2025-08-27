//
// Created by Dongho Kang on 09.04.23.
//

#ifndef CRL_HUMANOID_ESTOPNODE_H
#define CRL_HUMANOID_ESTOPNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"

namespace crl::unitree::commons {

        /**
         * E-Stop logic for unitree robots.
         */
        class EstopNode final : public BaseNode {
            public:
            EstopNode(const UnitreeRobotModel& model, const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData>& data);

            ~EstopNode() override;

            private:
            void timerCallbackImpl() override;
        };

}  // namespace crl::unitree::commons

#endif