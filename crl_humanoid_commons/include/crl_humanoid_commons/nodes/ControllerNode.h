//
// Created by Dongho Kang on 07.05.22.
//

#ifndef CRL_HUMANOID_CONTROLLERNODE_H
#define CRL_HUMANOID_CONTROLLERNODE_H

#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/RobotParameters.h"
#include "crl_humanoid_commons/nodes/BaseNode.h"
#include <rclcpp/logger.hpp>

namespace crl::humanoid::commons {

        /**
         * Abstract class for legged locomotion controllers.
         */
        class LocomotionController {
        public:
            // for debugging
            bool verbose = false;

        protected:
            // Robot pointer
            std::shared_ptr<RobotModel> model_ = nullptr;
            std::shared_ptr<RobotData> data_ = nullptr;

            // ROS2 logger
            rclcpp::Logger logger_;

            // Plan
            double timer = 0;

        public:
            explicit LocomotionController(const std::shared_ptr<RobotModel>& model,
                                          const std::shared_ptr<RobotData>& data,
                                          const rclcpp::Logger& logger)
                : model_(model), data_(data), logger_(logger) {}

            virtual ~LocomotionController() = default;

            /**
             * Compute and apply control signal with timestep size dt.
             */
            virtual void computeAndApplyControlSignals(double dt) = 0;
        };

        /**
         * Simple concrete implementation of LocomotionController that does nothing.
         * Useful as a default implementation or for testing purposes.
         */
        class DoNothingController : public LocomotionController {
        public:
            DoNothingController(const std::shared_ptr<RobotModel>& robot,
                                const std::shared_ptr<RobotData>& data,
                                const rclcpp::Logger& logger)
                : LocomotionController(robot, data, logger) {}

            /**
             * Implementation of pure virtual method that does nothing.
             */
            void computeAndApplyControlSignals(double dt) override {
                // Do nothing - this is a placeholder implementation
                (void)dt; // Suppress unused parameter warning
            }
        };

        /**
         * Abstract controller wrapper for robots.
         */
        template <typename ControlAlgorithm = DoNothingController>
        class ControllerNode : public BaseNode {
            static_assert(std::is_convertible<ControlAlgorithm*, LocomotionController*>::value,
                          "ControlAlgorithm must inherit LocomotionController as public");

        public:
            ControllerNode(const std::shared_ptr<RobotModel>& model,
                           const std::shared_ptr<RobotData>& data,
                           const std::string& nodeName="controller")
                : BaseNode(model, data, nodeName) {
                controller_ = std::make_shared<ControlAlgorithm>(model_, data_, this->get_logger());
            }

            ~ControllerNode() override = default;

        protected:
            /**
             * One iteration-control logic implementation. This function is called by timer Callback.
             * Derived classes should implement this function.
             */
            virtual void controlCallbackImpl() {
                controller_->computeAndApplyControlSignals(timeStepSize_);
            };

        private:
            void timerCallbackImpl() override {
                auto start = this->now();

                // call control callback
                controlCallbackImpl();

                // populate execution time (for profiling)
                auto profileInfo = data_->getProfilingInfo();
                profileInfo.controllerExecutionTime = (this->now() - start).seconds();
                data_->setProfilingInfo(profileInfo);
            }

        protected:
            std::shared_ptr<ControlAlgorithm> controller_ = nullptr;
        };
}  // namespace crl::humanoid::commons

#endif // CRL_HUMANOID_CONTROLLERNODE_H
