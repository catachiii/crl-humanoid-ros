//
// Created by Dongho Kang on 09.04.23.
//

#include "crl_humanoid_commons/nodes/EstopNode.h"

namespace crl::humanoid::commons {

    EstopNode::EstopNode(const std::shared_ptr<RobotModel> &model,
                         const std::shared_ptr<RobotData> &data)
            : BaseNode(model, data, "estop") {
        // parameters
        auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
        paramDesc.read_only = true;
        paramDesc.description = "estop parameters";
        this->declare_parameter<std::vector<double>>("joint_damping", std::vector<double>(model->defaultJointConf.size(), 0.0), paramDesc);

        jointCount_ = model->defaultJointConf.size();
        // stiffness and damping
        crl::resize(jointDamping_, jointCount_);
        // init joint stiffness and damping
        for (int i = 0; i < jointCount_; i++) {
            jointDamping_[i] = this->get_parameter("joint_damping").as_double_array()[i];
        }

        data_->softEStop = true;

        // reset command
        auto comm = data_->getCommand();
        comm.targetForwardSpeed = 0;
        comm.targetSidewaysSpeed = 0;
        comm.targetTurningSpeed = 0;
        data_->setCommand(comm);

        // reset control input to zero
        RobotControlSignal control = data_->getControlSignal();
        for (size_t i = 0; i < control.jointControl.size(); i++) {
            control.jointControl[i].mode = RBJointControlMode::FORCE_MODE;
            control.jointControl[i].desiredPos = 0;
            control.jointControl[i].desiredSpeed = 0;
            control.jointControl[i].desiredTorque = 0;
            control.jointControl[i].stiffness = 0;
            control.jointControl[i].damping = 0;
        }

        data_->setControlSignal(control);
    }

    EstopNode::~EstopNode() {
        // reset control input to hold current position
        // this is necessary to prevent jumping while the state transition happens
        crl::humanoid::commons::RobotState state = data_->getRobotState();
        crl::humanoid::commons::RobotControlSignal control = data_->getControlSignal();
        for (size_t i = 0; i < control.jointControl.size(); i++) {
            control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
            control.jointControl[i].desiredPos = state.jointStates[i].jointPos;
            control.jointControl[i].desiredSpeed = 0;
            control.jointControl[i].desiredTorque = 0;
            control.jointControl[i].stiffness = 0;
            control.jointControl[i].damping = 0;
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

}  // namespace crl::humanoid::commons
