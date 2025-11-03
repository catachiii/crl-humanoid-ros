
#ifndef CRL_G1_DEEPMIMICCONTROLLER_H
#define CRL_G1_DEEPMIMICCONTROLLER_H

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>
#include "crl_humanoid_commons/nodes/ControllerNode.h"
#include <rclcpp/logger.hpp>

namespace crl::g1::mimiccontroller {

    class CRLG1DeepTrackController : public crl::humanoid::commons::LocomotionController {
    public:
        explicit CRLG1DeepTrackController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                     const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                     const rclcpp::Logger &logger);

        ~CRLG1DeepTrackController() override = default;

        bool loadModelFromFile(const std::string &fileName);

        void computeAndApplyControlSignals(double dt) override;

        /**
         * Check if the deep mimic sequence is complete
         */
        bool isSequenceComplete() const;

        /**
         * Set whether to loop the sequence when it completes
         */
        void setLoopSequence(bool loop);

    private:
        /**
         * Preprocess motion data to reorder joints and reorganize frame structure
         */
        void preprocessMotionData();

        // Helper functions for quaternion operations
        crl::V3D quatApplyInverse(const crl::Quaternion& quat, const crl::V3D& vec) const;
        crl::Quaternion quatConjugate(const crl::Quaternion& q) const;
        crl::Quaternion quatMul(const crl::Quaternion& q1, const crl::Quaternion& q2) const;
        crl::Matrix3x3 matrixFromQuat(const crl::Quaternion& quat) const;

        // cache
        crl::dVector action_;

        crl::dVector currentObs_;
        std::vector<crl::dVector> obsHistory_;
        int numObs_;
        int numActions_;
        int numHistory_;
        bool sequenceCompleted_ = false;
        bool loopSequence_ = false;
        crl::dVector defaultJointPos_;
        crl::dVector actionScale_;
        crl::dVector jointStiffness_;
        crl::dVector jointDamping_;

        // onnx
        Ort::Env env_;
        Ort::Session session_{nullptr};
        Ort::Value inputTensor_{nullptr};
        Ort::Value outputTensor_{nullptr};
        std::vector<float> inputData_;
        std::vector<float> outputData_;
        std::array<int64_t, 2> inputShape_;
        std::array<int64_t, 2> outputShape_;

        // Timing
        double timer = 0.0;

        // Motion data
        std::vector<std::vector<double>> motionData_;
        std::vector<std::vector<double>> processedMotionData_;
        int motionDataLength_ = 0;
        int currentFrameIndex_ = 0;
        std::vector<int> featureBodyIds_;

    };

}  // namespace crl::g1::mimiccontroller

#endif  // CRL_G1_DEEPMIMICCONTROLLER_H
