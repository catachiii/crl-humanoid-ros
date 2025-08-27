//
// Created by Dongho Kang on 09.04.23.
//

#include "crl_humanoid_commons/nodes/EstopNode.h"

namespace crl::unitree::commons {

    EstopNode::EstopNode(const UnitreeRobotModel &model,
                         const std::shared_ptr<crl::unitree::commons::UnitreeLeggedRobotData> &data)
            : BaseNode(model, data, "estop") {
        data_->softEStop = true;

        // reset command
        auto comm = data_->getCommand();
        comm.targetForwardSpeed = 0;
        comm.targetSidewaysSpeed = 0;
        comm.targetTurningSpeed = 0;
        data_->setCommand(comm);

        // reset control input to zero
        crl::unitree::commons::LeggedRobotControlSignal control = data_->getControlSignal();
        for (size_t i = 0; i < control.jointControl.size(); i++) {
            control.jointControl[i].mode = crl::unitree::commons::RBJointControlMode::FORCE_MODE;
            control.jointControl[i].desiredPos = 0;
            control.jointControl[i].desiredSpeed = 0;
            control.jointControl[i].desiredTorque = 0;
        }

        // other information
        control.targetBasePos = crl::P3D();
        control.targetBaseVel = crl::V3D();
        control.targetBaseOrientation = crl::Quaternion::Identity();
        control.targetBaseAngularVelocity = crl::V3D();
        // these are not used in e-stop, but we set them to zero to prevent jumping
        // when the state transition happens
        control.targetBaseAcc = crl::V3D();
        control.targetBaseAngularAcc = crl::V3D();

        data_->setControlSignal(control);
    }

    EstopNode::~EstopNode() {
        // reset control input to hold current position
        // this is necessary to prevent jumping while the state transition happens
        crl::unitree::commons::LeggedRobotState state = data_->getLeggedRobotState();
        crl::unitree::commons::LeggedRobotControlSignal control = data_->getControlSignal();
        for (size_t i = 0; i < control.jointControl.size(); i++) {
            control.jointControl[i].mode = crl::unitree::commons::RBJointControlMode::POSITION_MODE;
            control.jointControl[i].desiredPos = state.jointStates[i].jointPos;
            control.jointControl[i].desiredSpeed = 0;
            control.jointControl[i].desiredTorque = 0;
        }
        data_->setControlSignal(control);
        data_->softEStop = false;
    }

    void EstopNode::timerCallbackImpl() {
        // populate execution time (for profiling)
        auto profileInfo = data_->getProfilingInfo();
        profileInfo.controllerExecutionTime = 0;
        data_->setProfilingInfo(profileInfo);
    }

}  // namespace crl::unitree::commons
