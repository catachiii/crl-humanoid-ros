//
// Created by Dongho Kang on 07.05.22.
//

#ifndef CRL_HUMANOID_CONTROLLERNODE_H
#define CRL_HUMANOID_CONTROLLERNODE_H

#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/RobotParameters.h"
#include "crl_humanoid_commons/nodes/BaseNode.h"

namespace crl::unitree::commons {

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

        /**
         * Simple concrete implementation of LocomotionController that does nothing.
         * Useful as a default implementation or for testing purposes.
         */
        class DoNothingController : public LocomotionController {
            public:
            DoNothingController(const UnitreeRobotModel& robot,
                               const std::shared_ptr<UnitreeLeggedRobotData>& data)
                : LocomotionController(std::make_shared<UnitreeRobotModel>(robot), data) {}

            protected:
            void prepareForControlStep(double /* dt */) override {
                // Do nothing
            }

            void computeControlSignals(double /* dt */) override {
                // Do nothing
            }

            void applyControlSignals(double /* dt */) override {
                // Do nothing
            }

            void populateData() override {
                // Do nothing
            }
        };

        /**
         * Tracking controller wrapper for unitree robots.
         */
        template <typename ControlAlgorithm = DoNothingController>
        class ControllerNode : public BaseNode {
            static_assert(std::is_convertible<ControlAlgorithm*, crl::unitree::commons::LocomotionController*>::value,
                          "ControlAlgorithm must inherit crl::unitree::commons::LocomotionController as public");

            public:
            ControllerNode(const UnitreeRobotModel& model, const std::shared_ptr<UnitreeLeggedRobotData>& data, const std::string& nodeName="controller")
            : BaseNode(model, data, nodeName), controller_(model, data) {}

            ~ControllerNode() override = default;

            protected:
            /**
             * One iteration-control logic implementation. This function is called by timer Callback.
             * Derived classes should implement this function.
             */
            virtual void controlCallbackImpl() {
                controller_.computeAndApplyControlSignals(timeStepSize_);
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
            ControlAlgorithm controller_;
        };
}  // namespace crl::unitree::commons

#endif // CRL_HUMANOID_CONTROLLERNODE_H
