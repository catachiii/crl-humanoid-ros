#ifndef CRL_TRON1A_RLCONTROLLER_CONTROLLERNODE_H
#define CRL_TRON1A_RLCONTROLLER_CONTROLLERNODE_H

#include <string>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "crl_tron1a_rlcontroller/CRLTron1AWalkController.h"
#include "crl_humanoid_commons/nodes/ControllerNode.h"
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace crl::tron1a::rlcontroller {


    template <typename ControllerT = CRLTron1AWalkController>
    class CRLTron1ARLControllerNode : public crl::humanoid::commons::ControllerNode<ControllerT> {
    public:
        using Base = crl::humanoid::commons::ControllerNode<ControllerT>;
        CRLTron1ARLControllerNode(const std::shared_ptr<crl::humanoid::commons::RobotModel>& model,
                              const std::shared_ptr<crl::humanoid::commons::RobotData>& data,
                              const std::string& nodeName="rlcontroller")
            : Base(model, data, nodeName) {
            // parameters
            auto paramDesc = rcl_interfaces::msg::ParameterDescriptor{};
            paramDesc.description = "Tron1a RL controller parameters.";
            paramDesc.read_only = true;

            if (!this->has_parameter("config_file")) {
                this->template declare_parameter<std::string>("config_file", "");
            }

            std::string configFile;
            if (this->get_parameter("config_file", configFile) && !configFile.empty()) {
                // Resolve config file path if relative
                std::filesystem::path configPath(configFile);
                if (!configPath.is_absolute()) {
#ifdef CRL_TRON1A_RLCONTROLLER_DATA_FOLDER
                    // If data folder is defined (development mode), use it to resolve path relative to package root
                    std::filesystem::path packageRoot = std::filesystem::path(CRL_TRON1A_RLCONTROLLER_DATA_FOLDER).parent_path();
                    configPath = packageRoot / configPath;
#else
                    try {
                        std::string packageShareDir = ament_index_cpp::get_package_share_directory("crl_tron1a_rlcontroller");
                        configPath = std::filesystem::path(packageShareDir) / configPath;
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to locate package share directory: %s", e.what());
                    }
#endif
                }

                if (!this->controller_->loadModelFromParams(configPath.string())) {
                    RCLCPP_FATAL(this->get_logger(), "Cannot load RL config from: %s", configPath.c_str());
                }
            } else {
                RCLCPP_FATAL(this->get_logger(), "Parameter 'config_file' is required for CRLTron1ARLControllerNode");
            }
        }

        virtual ~CRLTron1ARLControllerNode() = default;

    protected:
        void controlCallbackImpl() override {
            // compute and apply control signals
            this->controller_->computeAndApplyControlSignals(this->timeStepSize_);
        }
    };

}  // namespace crl::tron1a::rlcontroller

#endif  // CRL_TRON1A_RLCONTROLLER_CONTROLLERNODE_H
