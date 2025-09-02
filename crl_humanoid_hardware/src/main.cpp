//
// Created by copilot on 02.09.25.
//

#include <crl_fsm/fsm.h>
#include <crl_humanoid_commons/RobotData.h>
#include <crl_humanoid_commons/nodes/ControllerNode.h>
#include <crl_humanoid_commons/nodes/EstopNode.h>
#include <crl_humanoid_commons/nodes/StarterNode.h>
#include <crl_humanoid_commons/nodes/CommNode.h>
#include <crl_humanoid_hardware/G1Node.h>
#include <rclcpp/rclcpp.hpp>
#include <cxxopts.hpp>

crl_fsm_states(States, ESTOP, STAND, WALK);
crl_fsm_machines(Machines, ONBOARD);

int main(int argc, char** argv) {
    // parse arguments
    cxxopts::Options options("hardware", "crl_unitree_hardware");
    options.allow_unrecognised_options();
    options.add_options()("model", "Robot model", cxxopts::value<std::string>()->default_value("g1"));
    auto result = options.parse(argc, argv);

    std::string modelName = "G1";
    if (result["model"].as<std::string>() == "g1") {
        modelName = "G1";
    } else {
        RCLCPP_WARN(rclcpp::get_logger("hardware"), "Unknown robot model: %s", result["model"].as<std::string>().c_str());
    }

    // transitions
    crl::fsm::Transition<States::ESTOP, States::STAND> t1;
    crl::fsm::Transition<States::STAND, States::ESTOP> t2;
    crl::fsm::Transition<States::STAND, States::WALK> t3;
    crl::fsm::Transition<States::WALK, States::ESTOP> t4;
    crl::fsm::Transition<States::WALK, States::STAND> t5;

    // data
    const auto model = std::make_shared<crl::humanoid::commons::RobotModel>(crl::humanoid::commons::robotModels.at(modelName));
    const auto data = std::make_shared<crl::humanoid::commons::RobotData>(model->jointNames, model->defaultJointConf);

    // machines
    auto m1 =
        crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::ESTOP>([&]() { return std::make_shared<crl::humanoid::commons::EstopNode>(model, data); });
    auto m2 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::STAND>(
        [&]() { return std::make_shared<crl::humanoid::commons::StarterNode>(crl::humanoid::commons::StarterNode::TargetMode::STAND, model, data, "stand"); });
    auto m3 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::WALK>(
        [&]() { return std::make_shared<crl::humanoid::commons::ControllerNode<>>(model, data); });

    auto s_cols = crl::fsm::make_states_collection_for_machine<Machines::ONBOARD, States>(m1, m2, m3);
    constexpr auto t_cols = crl::fsm::make_transitions_collection<States>(t1, t2, t3, t4, t5);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // init ros process
    auto machine = crl::fsm::make_fsm<Machines, Machines::ONBOARD>("robot", States::ESTOP, s_cols, t_cols);

    std::array<Machines, 1> monitoring = {Machines::ONBOARD};

    // Create robot node (real hardware communication)
    const auto commNode = std::make_shared<crl::humanoid::commons::CommNode>(model, data);
    std::shared_ptr<crl::unitree::hardware::g1::G1Node<States, Machines, 1>> robotNode;

    if (modelName == "G1") {
        robotNode = std::make_shared<crl::unitree::hardware::g1::G1Node<States, Machines, 1>>(
            model, data, monitoring, machine.is_transitioning());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "Unsupported robot model: %s", modelName.c_str());
        return -1;
    }

    auto& executor = machine.get_executor();
    executor.add_node(robotNode);
    executor.add_node(commNode);

    // Start the FSM machine (this will handle the hardware communication loop)
    try {
        machine.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "Hardware interface error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Hardware interface shutting down...");
    rclcpp::shutdown();
    return 0;
}
