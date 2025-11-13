//
// LimX-style stand-up node matching deployment behavior
//

#ifndef CRL_TRON1A_RLCONTROLLER_LIMXSTANDNODE_H
#define CRL_TRON1A_RLCONTROLLER_LIMXSTANDNODE_H

#include "crl_humanoid_commons/nodes/BaseNode.h"

namespace crl::tron1a::rlcontroller {

    /**
     * A node to control the robot from current pose to standing pose using LimX deployment behavior.
     * This matches the stand-up logic from the LimX deployment repo.
     */
    class LimXStandNode : public crl::humanoid::commons::BaseNode {
    public:
        LimXStandNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                     const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                     const std::string& nodeName);

        ~LimXStandNode() = default;

    private:
        void timerCallbackImpl() override;

    protected:
        int jointCount_ = 0;  // number of joints
        // joint angles at the transition (captured from current pose)
        crl::dVector initialJointAngle_;
        // target joint angles (standing pose with hip adjustments)
        crl::dVector targetJointAngle_;
        // stiffness
        crl::dVector jointStiffness_;
        // damping
        crl::dVector jointDamping_;
        // wheel joint damping (separate from regular joints)
        double wheelJointDamping_{0.8};
        // initialized flags
        double timeAtTransition_ = 0.0;
        bool initialized_ = false;
        // Stand-up progress (0.0 to 1.0)
        double standPercent_{0.0};
        // Stand-up duration in seconds
        double standDuration_{1.0};
        // Control loop frequency (Hz)
        double loopFrequency_{500.0};
        // Wheel joint indices
        int wheel_L_idx_{-1};
        int wheel_R_idx_{-1};
    };

}  // namespace crl::tron1a::rlcontroller

#endif  // CRL_TRON1A_RLCONTROLLER_LIMXSTANDNODE_H

