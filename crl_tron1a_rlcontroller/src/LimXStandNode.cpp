//
// LimX-style stand-up node matching deployment behavior
//

#include "crl_tron1a_rlcontroller/LimXStandNode.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace crl::tron1a::rlcontroller {

    LimXStandNode::LimXStandNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                                 const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                                 const std::string& nodeName)
            : BaseNode(model, data, nodeName) {
        RCLCPP_INFO(rclcpp::get_logger("LimXStandNode"), "LimXStandNode constructor called");
        
        // Safety check: ensure model and data are valid
        if (!model) {
            RCLCPP_FATAL(rclcpp::get_logger("LimXStandNode"), "Model pointer is null!");
            return;
        }
        if (!data) {
            RCLCPP_FATAL(rclcpp::get_logger("LimXStandNode"), "Data pointer is null!");
            return;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("LimXStandNode"), "Model and data pointers are valid");
        
        // parameters
        auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
        paramDesc.read_only = true;
        paramDesc.description = "LimX stand-up parameters";
        this->declare_parameter<double>("duration", 1.0, paramDesc);
        this->declare_parameter<std::vector<double>>("target_joint_angles", model->defaultJointConf, paramDesc);
        this->declare_parameter<std::vector<double>>("joint_stiffness", model->jointStiffnessDefault, paramDesc);
        this->declare_parameter<std::vector<double>>("joint_damping", model->jointDampingDefault, paramDesc);
        this->declare_parameter<double>("loop_frequency", 500.0, paramDesc);

        // reset command
        auto comm = data_->getCommand();
        comm.targetForwardSpeed = 0;
        comm.targetSidewaysSpeed = 0;
        comm.targetTurningSpeed = 0;
        data_->setCommand(comm);

        // init time at transition
        timeAtTransition_ = data_->getTimeStamp();

        // Safety check: ensure model data is valid
        if (model->jointNames.empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("LimXStandNode"), 
                        "Model jointNames is empty! Cannot initialize LimXStandNode.");
            return;
        }
        
        // Use jointNames.size() instead of defaultJointConf.size() to match actual joint count
        jointCount_ = model->jointNames.size();
        
        RCLCPP_INFO(rclcpp::get_logger("LimXStandNode"), 
                   "Joint count: %d (from jointNames.size())", jointCount_);
        
        // Get parameters
        standDuration_ = this->get_parameter("duration").as_double();
        loopFrequency_ = this->get_parameter("loop_frequency").as_double();
        
        // Safety check: ensure parameters are valid
        if (standDuration_ <= 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "Invalid standDuration_: %.2f (must be > 0), using default 1.0", standDuration_);
            standDuration_ = 1.0;
        }
        if (loopFrequency_ <= 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "Invalid loopFrequency_: %.2f (must be > 0), using default 500.0", loopFrequency_);
            loopFrequency_ = 500.0;
        }
        
        // Get wheel joint damping from RobotParameters (find wheel joint index first)
        int wheel_idx = -1;
        for (int i = 0; i < jointCount_ && i < static_cast<int>(model->jointNames.size()); i++) {
            if (model->jointNames[i] == "wheel_L_Joint" || model->jointNames[i] == "wheel_R_Joint") {
                wheel_idx = i;
                break;
            }
        }
        if (wheel_idx >= 0 && wheel_idx < static_cast<int>(model->jointDampingDefault.size())) {
            wheelJointDamping_ = model->jointDampingDefault[wheel_idx];
        } else {
            wheelJointDamping_ = 0.8;  // fallback default
            RCLCPP_WARN(rclcpp::get_logger("LimXStandNode"), 
                       "Could not find wheel joint in model, using default wheel damping: %.2f", wheelJointDamping_);
        }
        
        // stiffness and damping
        crl::resize(jointStiffness_, jointCount_);
        crl::resize(jointDamping_, jointCount_);
        auto stiffnessArray = this->get_parameter("joint_stiffness").as_double_array();
        auto dampingArray = this->get_parameter("joint_damping").as_double_array();
        
        // Safety check: ensure parameter arrays are properly sized
        if (stiffnessArray.size() < static_cast<size_t>(jointCount_)) {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "joint_stiffness array size (%zu) < jointCount_ (%d)", 
                       stiffnessArray.size(), jointCount_);
        }
        if (dampingArray.size() < static_cast<size_t>(jointCount_)) {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "joint_damping array size (%zu) < jointCount_ (%d)", 
                       dampingArray.size(), jointCount_);
        }
        
        for (int i = 0; i < jointCount_; i++) {
            if (i < static_cast<int>(stiffnessArray.size())) {
                jointStiffness_[i] = stiffnessArray[i];
            } else {
                jointStiffness_[i] = model->jointStiffnessDefault[i < static_cast<int>(model->jointStiffnessDefault.size()) ? i : 0];
            }
            if (i < static_cast<int>(dampingArray.size())) {
                jointDamping_[i] = dampingArray[i];
            } else {
                jointDamping_[i] = model->jointDampingDefault[i < static_cast<int>(model->jointDampingDefault.size()) ? i : 0];
            }
        }

        // Find wheel joint indices
        if (model->jointNames.size() < static_cast<size_t>(jointCount_)) {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "Model jointNames size (%zu) < jointCount_ (%d)", 
                       model->jointNames.size(), jointCount_);
        }
        for (int i = 0; i < jointCount_ && i < static_cast<int>(model->jointNames.size()); i++) {
            if (model->jointNames[i] == "wheel_L_Joint") {
                wheel_L_idx_ = i;
            } else if (model->jointNames[i] == "wheel_R_Joint") {
                wheel_R_idx_ = i;
            }
        }

        // init initial joint angle (will be captured on first timer callback, not in constructor)
        crl::resize(initialJointAngle_, jointCount_);
        // Don't access sensor here - it may not be initialized yet. Do it in timerCallbackImpl instead.

        // target joint angles (matches LimX: adjust hip angles for STAND mode)
        crl::resize(targetJointAngle_, jointCount_);
        auto targetArray = this->get_parameter("target_joint_angles").as_double_array();
        
        // Safety check: ensure target array is properly sized
        if (targetArray.size() < static_cast<size_t>(jointCount_)) {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "target_joint_angles array size (%zu) < jointCount_ (%d)", 
                       targetArray.size(), jointCount_);
        }
        
        for (int i = 0; i < jointCount_; i++) {
            if (i < static_cast<int>(targetArray.size())) {
                targetJointAngle_[i] = targetArray[i];
            } else if (i < static_cast<int>(model->defaultJointConf.size())) {
                targetJointAngle_[i] = model->defaultJointConf[i];
            } else {
                targetJointAngle_[i] = 0.0;  // fallback
            }
        }
        
        // Adjust hip joint angles for STAND mode (matches LimX: hip_L = -0.9, hip_R = 0.9)
        for (int i = 0; i < jointCount_ && i < static_cast<int>(model->jointNames.size()); i++) {
            if (model->jointNames[i] == "hip_L_Joint") {
                targetJointAngle_[i] = -0.9;
            } else if (model->jointNames[i] == "hip_R_Joint") {
                targetJointAngle_[i] = 0.9;
            }
        }
        
        // Initialize standPercent_ (matches LimX: starts with 1 / (standDuration_ * loopFrequency_))
        double denominator = standDuration_ * loopFrequency_;
        if (denominator > 0.0) {
            standPercent_ = 1.0 / denominator;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                       "Invalid denominator for standPercent_: standDuration_=%.2f, loopFrequency_=%.2f, using default 0.0",
                       standDuration_, loopFrequency_);
            standPercent_ = 0.0;
        }
        
        // Log target joint angles for verification
        std::stringstream targetAnglesStr;
        targetAnglesStr << "Target joint angles: ";
        for (int i = 0; i < jointCount_ && i < static_cast<int>(model->jointNames.size()); i++) {
            targetAnglesStr << model->jointNames[i] << "=" << targetJointAngle_[i];
            if (i < jointCount_ - 1) targetAnglesStr << ", ";
        }
        RCLCPP_INFO(rclcpp::get_logger("LimXStandNode"), "%s", targetAnglesStr.str().c_str());
        
        RCLCPP_INFO(rclcpp::get_logger("LimXStandNode"),
                   "LimXStandNode initialized: duration=%.2f s, loop_frequency=%.1f Hz, wheel_damping=%.2f (from RobotParameters), "
                   "wheel_L_idx=%d, wheel_R_idx=%d, initial_standPercent=%.6f",
                   standDuration_, loopFrequency_, wheelJointDamping_, wheel_L_idx_, wheel_R_idx_, standPercent_);
    }

    void LimXStandNode::timerCallbackImpl() {
        auto start = this->now();

        if (!initialized_) {
            // init time at transition
            timeAtTransition_ = data_->getTimeStamp();

            // init initial joint angle (capture current pose)
            crl::resize(initialJointAngle_, jointCount_);
            auto sensor = data_->getSensor();
            
            // Safety check: ensure sensor data is available and properly sized
            if (sensor.jointSensors.size() < static_cast<size_t>(jointCount_)) {
                RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                           "Sensor data not ready: jointCount_=%d but sensor.jointSensors.size()=%zu", 
                           jointCount_, sensor.jointSensors.size());
                return;  // Wait for next callback
            }
            
            for (int i = 0; i < jointCount_; i++) {
                if (i < static_cast<int>(sensor.jointSensors.size())) {
                    initialJointAngle_[i] = sensor.jointSensors[i].jointPos;
                    if (i < static_cast<int>(model_->jointNames.size())) {
                        RCLCPP_INFO(rclcpp::get_logger("LimXStandNode"), 
                                   "Captured initial joint position %s: %.3f", 
                                   model_->jointNames[i].c_str(), initialJointAngle_[i]);
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("LimXStandNode"), 
                               "Joint index %d out of bounds for sensor data", i);
                    initialJointAngle_[i] = 0.0;  // fallback
                }
            }
            // bandage solution...
            data_->softEStop = false;

            initialized_ = true;
        }
        
        // Safety check: ensure we're initialized before proceeding
        if (!initialized_) {
            return;  // Wait for initialization (shouldn't happen, but safety check)
        }
        
        // Handle stand-up interpolation (matches LimX handleStandMode)
        if (standPercent_ < 1.0) {
            // Update joint target - create new control signal and resize properly
            crl::humanoid::commons::RobotControlSignal control;
            control.jointControl.resize(jointCount_);
            for (int i = 0; i < jointCount_; i++) {
                if (i >= static_cast<int>(model_->jointNames.size())) {
                    RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                               "Joint index %d out of bounds for jointNames (size=%zu)", 
                               i, model_->jointNames.size());
                    continue;
                }
                control.jointControl[i].name = model_->jointNames[i];
                if (i == wheel_L_idx_ || i == wheel_R_idx_) {
                    // Wheel joints: velocity mode with damping (matches LimX handleStandMode)
                    control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::VELOCITY_MODE;
                    control.jointControl[i].desiredPos = 0;
                    control.jointControl[i].desiredSpeed = 0;  // Lock wheels during stand-up
                    control.jointControl[i].stiffness = 0;
                    control.jointControl[i].damping = wheelJointDamping_;
                    control.jointControl[i].desiredTorque = 0;
                } else {
                    // Regular joints: position mode (matches LimX handleStandMode)
                    // Interpolation formula matches LimX: initial * (1 - percent) + target * percent
                    if (i >= static_cast<int>(initialJointAngle_.size()) || i >= static_cast<int>(targetJointAngle_.size())) {
                        RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                                   "Joint index %d out of bounds for angle arrays", i);
                        continue;
                    }
                    double pos_des = initialJointAngle_[i] * (1 - standPercent_) + targetJointAngle_[i] * standPercent_;
                    
                    control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
                    control.jointControl[i].desiredPos = pos_des;
                    control.jointControl[i].desiredSpeed = 0;
                    if (i < static_cast<int>(jointStiffness_.size()) && i < static_cast<int>(jointDamping_.size())) {
                        control.jointControl[i].stiffness = jointStiffness_[i];
                        control.jointControl[i].damping = jointDamping_[i];
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                                   "Joint index %d out of bounds for stiffness/damping arrays", i);
                        control.jointControl[i].stiffness = 0.0;
                        control.jointControl[i].damping = 0.0;
                    }
                    control.jointControl[i].desiredTorque = 0;
                }
            }

            data_->setControlSignal(control);
            
            // Increment standPercent_ (matches LimX: += 3 / (standDuration_ * loopFrequency_))
            double incrementDenom = standDuration_ * loopFrequency_;
            if (incrementDenom > 0.0) {
                standPercent_ += 3.0 / incrementDenom;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                           "Invalid denominator for standPercent_ increment: standDuration_=%.2f, loopFrequency_=%.2f",
                           standDuration_, loopFrequency_);
                standPercent_ = 1.0;  // Force completion to avoid infinite loop
            }
            
            // Clamp to 1.0
            if (standPercent_ > 1.0) {
                standPercent_ = 1.0;
            }
        } else {
            // Stand-up complete - continue holding the target pose
            crl::humanoid::commons::RobotControlSignal control;
            control.jointControl.resize(jointCount_);
            for (int i = 0; i < jointCount_; i++) {
                if (i >= static_cast<int>(model_->jointNames.size())) {
                    RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                               "Joint index %d out of bounds for jointNames (size=%zu)", 
                               i, model_->jointNames.size());
                    continue;
                }
                control.jointControl[i].name = model_->jointNames[i];
                if (i == wheel_L_idx_ || i == wheel_R_idx_) {
                    // Wheel joints: velocity mode with damping
                    control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::VELOCITY_MODE;
                    control.jointControl[i].desiredPos = 0;
                    control.jointControl[i].desiredSpeed = 0;
                    control.jointControl[i].stiffness = 0;
                    control.jointControl[i].damping = wheelJointDamping_;
                    control.jointControl[i].desiredTorque = 0;
                } else {
                    // Regular joints: hold target position
                    if (i >= static_cast<int>(targetJointAngle_.size())) {
                        RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                                   "Joint index %d out of bounds for targetJointAngle_", i);
                        continue;
                    }
                    control.jointControl[i].mode = crl::humanoid::commons::RBJointControlMode::POSITION_MODE;
                    control.jointControl[i].desiredPos = targetJointAngle_[i];
                    control.jointControl[i].desiredSpeed = 0;
                    if (i < static_cast<int>(jointStiffness_.size()) && i < static_cast<int>(jointDamping_.size())) {
                        control.jointControl[i].stiffness = jointStiffness_[i];
                        control.jointControl[i].damping = jointDamping_[i];
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("LimXStandNode"), 
                                   "Joint index %d out of bounds for stiffness/damping arrays", i);
                        control.jointControl[i].stiffness = 0.0;
                        control.jointControl[i].damping = 0.0;
                    }
                    control.jointControl[i].desiredTorque = 0;
                }
            }
            data_->setControlSignal(control);
        }

        // populate execution time (for profiling)
        auto profileInfo = data_->getProfilingInfo();
        profileInfo.controllerExecutionTime = (this->now() - start).seconds();
        data_->setProfilingInfo(profileInfo);
    }

}  // namespace crl::tron1a::rlcontroller

