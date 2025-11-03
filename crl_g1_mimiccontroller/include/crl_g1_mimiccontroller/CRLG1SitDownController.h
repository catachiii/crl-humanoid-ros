
#ifndef CRL_G1_SITDOWNCONTROLLER_H
#define CRL_G1_SITDOWNCONTROLLER_H

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>
#include "crl_humanoid_commons/nodes/ControllerNode.h"
#include <rclcpp/logger.hpp>

namespace crl::g1::mimiccontroller {

    class CRLG1SitDownController : public crl::humanoid::commons::LocomotionController {
    public:
        explicit CRLG1SitDownController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                     const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                     const rclcpp::Logger &logger);

        ~CRLG1SitDownController() override = default;

        bool loadModelFromFile(const std::string &fileName);

        void computeAndApplyControlSignals(double dt) override;

    private:
        // cache
        crl::dVector action_;

        crl::dVector currentObs_;
        std::vector<crl::dVector> obsHistory_;
        int numObs_;
        int numActions_;
        int numHistory_;
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

}  // namespace crl::g1::mimiccontroller

#endif  // CRL_G1_SITDOWNCONTROLLER_H
