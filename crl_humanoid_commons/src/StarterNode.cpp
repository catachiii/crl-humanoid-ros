//
// Created by Dongho Kang on 09.04.23.
//

#include "crl_humanoid_commons/nodes/StarterNode.h"

namespace crl::humanoid::commons {

    StarterNode::StarterNode(const StarterNode::TargetMode target, const std::shared_ptr<RobotModel>& model,
                             const std::shared_ptr<RobotData>& data, const std::string& nodeName)
            : BaseNode(model, data, nodeName) {
        // parameters
        auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
        paramDesc.read_only = true;
        paramDesc.description = "Starter parameters";
        this->declare_parameter<double>("duration", 1.0, paramDesc);
        this->declare_parameter<std::vector<double>>("target_joint_angles", model->defaultJointConf, paramDesc);
        this->declare_parameter<std::vector<double>>("joint_stiffness", model->jointStiffnessDefault, paramDesc);
        this->declare_parameter<std::vector<double>>("joint_damping", model->jointDampingDefault, paramDesc);

        // reset command
        auto comm = data_->getCommand();
        comm.targetForwardSpeed = 0;
        comm.targetSidewaysSpeed = 0;
        comm.targetTurningSpeed = 0;
        data_->setCommand(comm);

        // init time at transition
        timeAtTransition_ = data_->getTimeStamp();

        jointCount_ = model->defaultJointConf.size();
        // stiffness and damping
        crl::resize(jointStiffness_, jointCount_);
        crl::resize(jointDamping_, jointCount_);
        // init joint stiffness and damping
        for (int i = 0; i < jointCount_; i++) {
            jointStiffness_[i] = this->get_parameter("joint_stiffness").as_double_array()[i];
            jointDamping_[i] = this->get_parameter("joint_damping").as_double_array()[i];
        }

        // init initial joint angle
        crl::resize(initialJointAngle_, jointCount_);
        auto sensor = data_->getSensor();
        for (int i = 0; i < jointCount_; i++) {
            initialJointAngle_[i] = sensor.jointSensors[i].jointPos;
        }

        // target joint angles
        crl::resize(targetJointAngle_, jointCount_);
        for (int i = 0; i < jointCount_; i++) {
            if (target == TargetMode::ZERO)
                targetJointAngle_[i] = 0.0;
            else if (target == TargetMode::STAND)
                targetJointAngle_[i] = this->get_parameter("target_joint_angles").as_double_array()[i];
            else
                RCLCPP_ERROR(this->get_logger(), "Unknown target mode.");
        }
    }

    void StarterNode::timerCallbackImpl() {
        auto start = this->now();

        if (!initialized_) {
            // init time at transition
            timeAtTransition_ = data_->getTimeStamp();

            // init initial joint angle
            crl::resize(initialJointAngle_, jointCount_);
            auto sensor = data_->getSensor();
            for (int i = 0; i < jointCount_; i++) {
                initialJointAngle_[i] = sensor.jointSensors[i].jointPos;
            }
            // bandage solution...
            data_->softEStop = false;

            initialized_ = true;
        }

        // for interpolation
        double t = data_->getTimeStamp();
        double percent = std::max(std::min((t - timeAtTransition_) / this->get_parameter("duration").as_double(), 1.0), 0.0);

        // update joint target
        crl::humanoid::commons::RobotControlSignal control = data_->getControlSignal();
        for (int i = 0; i < (int)control.jointControl.size(); i++) {
            control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
            control.jointControl[i].stiffness = jointStiffness_[i];
            control.jointControl[i].damping = jointDamping_[i];
            control.jointControl[i].desiredPos = percent * targetJointAngle_[i] + (1 - percent) * initialJointAngle_[i];
            control.jointControl[i].desiredSpeed = 0;
            control.jointControl[i].desiredTorque = 0;
        }

        // other information
        data_->setControlSignal(control);

        // populate execution time (for profiling)
        auto profileInfo = data_->getProfilingInfo();
        profileInfo.controllerExecutionTime = (this->now() - start).seconds();
        data_->setProfilingInfo(profileInfo);
    }

}  // namespace crl::humanoid::commons
