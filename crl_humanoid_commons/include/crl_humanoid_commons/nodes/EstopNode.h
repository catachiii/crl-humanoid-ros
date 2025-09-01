//
// Created by Dongho Kang on 09.04.23.
//

#ifndef CRL_HUMANOID_ESTOPNODE_H
#define CRL_HUMANOID_ESTOPNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"

namespace crl::humanoid::commons {

        /**
         * E-Stop logic for robots.
         */
        class EstopNode final : public BaseNode {
            public:
            EstopNode(const std::shared_ptr<RobotModel>& model, const std::shared_ptr<RobotData>& data);

            ~EstopNode() override;

            private:
            void timerCallbackImpl() override;

            protected:
            int jointCount_ = 0;  // number of joints
            // damping
            crl::dVector jointDamping_;
        };

}  // namespace crl::humanoid::commons

#endif
