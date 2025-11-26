#ifndef CRL_TRON1A_RLCONTROLLER_WALKCONTROLLER_H
#define CRL_TRON1A_RLCONTROLLER_WALKCONTROLLER_H

#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <crl-basic/utils/mathDefs.h>
#include "crl_humanoid_commons/nodes/ControllerNode.h"

namespace crl::tron1a::rlcontroller {

    class CRLTron1AWalkController : public crl::humanoid::commons::LocomotionController {
    public:
        explicit CRLTron1AWalkController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                     const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                     const rclcpp::Logger &logger);

        ~CRLTron1AWalkController() override = default;

        bool loadModelFromParams(const std::string &fileName);

        void computeAndApplyControlSignals(double dt) override;

    private:
        // Helper methods
        void computeObservation();
        void computeEncoder();
        void computeActions();

        // cache
        crl::dVector action_;
        crl::dVector lastActions_;

        crl::dVector currentObs_;
        std::vector<crl::dVector> obsHistory_;
        std::vector<float> observations_;
        std::vector<float> proprioHistoryVector_;
        crl::dVector proprioHistoryBuffer_;

        int numObs_;
        int numActions_;
        int numHistory_;
        int observationSize_;
        int obsHistoryLength_;
        int encoderInputSize_;
        int encoderOutputSize_;

        crl::dVector defaultJointPos_;
        crl::dVector initJointAngles_;
        crl::dVector actionScale_;
        crl::dVector jointStiffness_;
        crl::dVector jointDamping_;

        // Command handling
        crl::dVector commands_;
        crl::dVector scaled_commands_;

        // Configuration (similar to RobotCfg)
        struct ControlCfg {
            double stiffness{0.0};
            double damping{0.0};
            double action_scale_pos{0.0};
            double user_torque_limit{0.0};
        } controlCfg_;

        struct ObsScales {
            double linVel{0.0};
            double angVel{0.0};
            double dofPos{0.0};
            double dofVel{0.0};
        } obsScales_;

        struct UserCmdCfg {
            double linVel_x{0.0};
            double linVel_y{0.0};
            double angVel_yaw{0.0};
        } userCmdCfg_;

        double clipActions_{0.0};
        double clipObs_{0.0};
        double imuOrientationOffset_[3] = {0.0, 0.0, 0.0};
        std::vector<int> jointPosIdxs_;

        // Wheel joint handling
        double wheelJointDamping_{0.0};
        double wheelJointTorqueLimit_{0.0};
        int wheel_L_idx_{6};
        int wheel_R_idx_{7};

        // onnx
        Ort::Env env_;
        Ort::MemoryInfo memoryInfo_;
        Ort::Session session_{nullptr};  // Policy session
        std::unique_ptr<Ort::Session> encoderSessionPtr_{nullptr};  // Encoder session

        // Pre-allocated tensors and buffers
        std::vector<Ort::Value> encoderInputTensors_;
        std::vector<Ort::Value> encoderOutputTensors_;
        std::vector<Ort::Value> policyInputTensors_;
        std::vector<Ort::Value> policyOutputTensors_;

        std::vector<float> combinedObs_; // Buffer for policy input
        std::vector<float> encoderInputData_; // Buffer for encoder input (float)

        // C-string arrays for ONNX Run
        std::vector<const char*> encoderInputNamesCStr_;
        std::vector<const char*> encoderOutputNamesCStr_;
        std::vector<const char*> policyInputNamesCStr_;
        std::vector<const char*> policyOutputNamesCStr_;

        Ort::Value inputTensor_{nullptr};
        Ort::Value outputTensor_{nullptr};
        std::vector<float> inputData_;
        std::vector<float> outputData_;
        std::vector<float> encoderOut_;
        std::array<int64_t, 2> inputShape_;
        std::array<int64_t, 2> outputShape_;
        std::vector<std::vector<int64_t>> encoderInputShapes_;
        std::vector<std::vector<int64_t>> encoderOutputShapes_;
        std::vector<std::string> encoderInputNames_;
        std::vector<std::string> encoderOutputNames_;
        std::vector<std::string> policyInputNames_;
        std::vector<std::string> policyOutputNames_;

        // Timing and control
        double timer = 0.0;
        bool isfirstRecObs_{true};

    };

}  // namespace crl::tron1a::rlcontroller

#endif  // CRL_TRON1A_RLCONTROLLER_WALKCONTROLLER_H
