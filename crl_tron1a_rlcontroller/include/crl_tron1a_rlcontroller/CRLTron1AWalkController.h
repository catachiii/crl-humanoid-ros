#ifndef CRL_TRON1A_RLCONTROLLER_WALKCONTROLLER_H
#define CRL_TRON1A_RLCONTROLLER_WALKCONTROLLER_H

#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <crl-basic/utils/mathDefs.h>
#include "crl_humanoid_commons/nodes/ControllerNode.h"

namespace robot_controllers {

template <typename T>
constexpr T square(T value) {
    return value * value;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(static_cast<SCALAR_T>(-2.) * (q.x() * q.z() - q.w() * q.y()), static_cast<SCALAR_T>(.99999));
    zyx(0) = std::atan2(static_cast<SCALAR_T>(2) * (q.x() * q.y() + q.w() * q.z()),
                        square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(static_cast<SCALAR_T>(2) * (q.y() * q.z() + q.w() * q.x()),
                        square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(
    const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) {
    const SCALAR_T z = eulerAngles(0);
    const SCALAR_T y = eulerAngles(1);
    const SCALAR_T x = eulerAngles(2);

    const SCALAR_T c1 = std::cos(z);
    const SCALAR_T c2 = std::cos(y);
    const SCALAR_T c3 = std::cos(x);
    const SCALAR_T s1 = std::sin(z);
    const SCALAR_T s2 = std::sin(y);
    const SCALAR_T s3 = std::sin(x);

    const SCALAR_T s2s3 = s2 * s3;
    const SCALAR_T s2c3 = s2 * c3;

    Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
    rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                      s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                      -s2,          c2 * s3,                   c2 * c3;
    return rotationMatrix;
}

using vector3_t = Eigen::Matrix<double, 3, 1>;
using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using tensor_element_t = float;

}  // namespace robot_controllers

namespace crl::tron1a::rlcontroller {

    class CRLTron1AWalkController : public crl::humanoid::commons::LocomotionController {
    public:
        explicit CRLTron1AWalkController(const std::shared_ptr<crl::humanoid::commons::RobotModel> &model,
                                     const std::shared_ptr<crl::humanoid::commons::RobotData> &data,
                                     const rclcpp::Logger &logger);

        ~CRLTron1AWalkController() override = default;

        bool loadModelFromFile(const std::string &fileName);
        bool loadModelFromParams(rclcpp::Node& node);

        void computeAndApplyControlSignals(double dt) override;

    private:
        // Helper methods
        void computeObservation();
        void computeEncoder();
        void computeActions();
        bool loadRLCfg(rclcpp::Node& node);

        // cache
        crl::dVector action_;
        crl::dVector lastActions_;

        crl::dVector currentObs_;
        std::vector<crl::dVector> obsHistory_;
        std::vector<robot_controllers::tensor_element_t> observations_;
        std::vector<robot_controllers::tensor_element_t> proprioHistoryVector_;
        Eigen::Matrix<robot_controllers::tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
        
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
        Eigen::Vector3d commands_;
        Eigen::Vector3d scaled_commands_;

        // Configuration (similar to RobotCfg)
        struct ControlCfg {
            double stiffness{0.0};
            double damping{0.0};
            double action_scale_pos{0.0};
            int decimation{0};
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
        Ort::Session session_{nullptr};  // Policy session
        std::unique_ptr<Ort::Session> encoderSessionPtr_{nullptr};  // Encoder session
        Ort::Value inputTensor_{nullptr};
        Ort::Value outputTensor_{nullptr};
        std::vector<robot_controllers::tensor_element_t> inputData_;
        std::vector<robot_controllers::tensor_element_t> outputData_;
        std::vector<robot_controllers::tensor_element_t> encoderOut_;
        std::array<int64_t, 2> inputShape_;
        std::array<int64_t, 2> outputShape_;
        std::vector<std::vector<int64_t>> encoderInputShapes_;
        std::vector<std::vector<int64_t>> encoderOutputShapes_;
        std::vector<const char*> encoderInputNames_;
        std::vector<const char*> encoderOutputNames_;
        std::vector<const char*> policyInputNames_;
        std::vector<const char*> policyOutputNames_;

        // Timing and control
        double timer = 0.0;
        int64_t loopCount_{0};
        bool isfirstRecObs_{true};

    };

}  // namespace crl::tron1a::rlcontroller

#endif  // CRL_TRON1A_RLCONTROLLER_WALKCONTROLLER_H