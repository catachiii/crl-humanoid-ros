//
// Created by Dongho Kang on 07.05.22.
//

#ifndef CRL_HUMANOID_CONTROLLERNODE_H
#define CRL_HUMANOID_CONTROLLERNODE_H

#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/RobotParameters.h"
#include "crl_humanoid_commons/nodes/BaseNode.h"

namespace crl::unitree::commons {

        // Forward declaration
        class LocomotionController;

        /**
         * Tracking controller wrapper for unitree robots.
         */
        template <typename ControlAlgorithm>
        class ControllerNode : public BaseNode {
            static_assert(std::is_convertible<ControlAlgorithm*, crl::unitree::commons::LocomotionController*>::value,
                          "ControlAlgorithm must inherit crl::unitree::commons::LocomotionController as public");

            public:
            ControllerNode(const UnitreeRobotModel& model, const std::shared_ptr<UnitreeLeggedRobotData>& data, const std::string& nodeName="controller")
            : BaseNode(model, data, nodeName) {
                // control algorithm
                controller_ = std::make_shared<ControlAlgorithm>(model_, data_);
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

        /**
         * Abstract class for legged locomotion controllers.
         */
        class LocomotionController {
            public:
            // for debugging
            bool verbose = false;

            protected:
            // Robot pointer
            std::shared_ptr<UnitreeRobotModel> model = nullptr;
            std::shared_ptr<UnitreeLeggedRobotData> data = nullptr;

            // Plan
            double timer = 0;

            public:
            explicit LocomotionController(const std::shared_ptr<UnitreeRobotModel>& model, const std::shared_ptr<UnitreeLeggedRobotData>& data)
            : model(model), data(data) {}

            virtual ~LocomotionController() = default;

            /**
             * Compute and apply control signal with timestep size dt.
             */
            virtual void computeAndApplyControlSignals(double dt) {
                prepareForControlStep(dt);
                computeControlSignals(dt);
                applyControlSignals(dt);
                populateData();
            }

            protected:
            virtual void prepareForControlStep(double dt) = 0;
            virtual void computeControlSignals(double dt) = 0;
            virtual void applyControlSignals(double dt) = 0;
            virtual void populateData() = 0;
        };
}  // namespace crl::unitree::commons

#endif // CRL_HUMANOID_CONTROLLERNODE_H
