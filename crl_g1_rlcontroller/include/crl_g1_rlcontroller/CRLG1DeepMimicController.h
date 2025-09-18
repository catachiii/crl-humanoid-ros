
#ifndef CRL_G1_DEEPMIMICCONTROLLER_H
#define CRL_G1_DEEPMIMICCONTROLLER_H

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>
#include "crl_humanoid_commons/nodes/ControllerNode.h"
#include <rclcpp/logger.hpp>

namespace crl::g1::rlcontroller {

    class CRLG1DeepMimicController : public crl::humanoid::commons::LocomotionController {
    public:
        explicit CRLG1DeepMimicController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                     const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                     const rclcpp::Logger &logger);

        ~CRLG1DeepMimicController() override = default;

        bool loadModelFromFile(const std::string &fileName);

        void computeAndApplyControlSignals(double dt) override;

        /**
         * Check if the deep mimic sequence is complete (phase >= 1.0)
         */
        bool isSequenceComplete() const;

        /**
         * Set whether to loop the sequence when it completes
         */
        void setLoopSequence(bool loop);

    private:
        // cache
        crl::dVector action_;

        crl::dVector currentObs_;
        std::vector<crl::dVector> obsHistory_;
        int numObs_;
        int numActions_;
        int numHistory_;
        double phase_;
        double phaseIncrement_;
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

    };

}  // namespace crl::g1::rlcontroller

#endif  // CRL_G1_DEEPMIMICCONTROLLER_H
