//
// Created by donghok on 23.06.21.
//

#include <crl_fsm/fsm.h>
#include <crl_humanoid_commons/RobotData.h>
#include <crl_humanoid_commons/nodes/ControllerNode.h>
#include <crl_humanoid_commons/nodes/EstopNode.h>
#include <crl_humanoid_commons/nodes/StarterNode.h>
#include <crl_humanoid_commons/nodes/CommNode.h>
#include <crl_humanoid_simulator/SimNode.h>
#include <rclcpp/rclcpp.hpp>

crl_fsm_states(States, ESTOP, STAND, WALK);
crl_fsm_machines(Machines, ONBOARD);

int main(int argc, char** argv) {
    // Default robot model
    std::string modelName = "G1";

    // transitions
    crl::fsm::Transition<States::ESTOP, States::STAND> t1;
    crl::fsm::Transition<States::STAND, States::ESTOP> t2;
    crl::fsm::Transition<States::STAND, States::WALK> t3;
    crl::fsm::Transition<States::WALK, States::ESTOP> t4;
    crl::fsm::Transition<States::WALK, States::STAND> t5;

    // data
    const auto& model = crl::unitree::commons::robotModels.at(modelName);
    const auto data = std::make_shared<crl::unitree::commons::UnitreeLeggedRobotData>(model.jointNames, model.defaultJointConf);

    // machines
    auto m1 =
        crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::ESTOP>([&]() { return std::make_shared<crl::unitree::commons::EstopNode>(model, data); });
    auto m2 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::STAND>(
        [&]() { return std::make_shared<crl::unitree::commons::StarterNode>(crl::unitree::commons::StarterNode::TargetMode::STAND, model, data, "stand"); });
    auto m3 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::WALK>(
        [&]() { return std::make_shared<crl::unitree::commons::ControllerNode<>>(model, data); });

    auto s_cols = crl::fsm::make_states_collection_for_machine<Machines::ONBOARD, States>(m1, m2, m3);
    constexpr auto t_cols = crl::fsm::make_transitions_collection<States>(t1, t2, t3, t4, t5);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // init ros process
    auto machine = crl::fsm::make_fsm<Machines, Machines::ONBOARD>("robot", States::ESTOP, s_cols, t_cols);

    std::array<Machines, 1> monitoring = {Machines::ONBOARD};

    // Create robot node (MuJoCo simulation is integrated into SimNode)
    const auto commNode = std::make_shared<crl::unitree::commons::CommNode>(model, data);
    const auto robotNode = std::make_shared<crl::unitree::simulator::SimNode<States, Machines, 1>>(model, data, monitoring, machine.is_transitioning());

    auto& executor = machine.get_executor();
    executor.add_node(robotNode);
    executor.add_node(commNode);

    RCLCPP_INFO(robotNode->get_logger(), "Starting ROS2 executor with integrated MuJoCo simulation...");

    // Start the FSM machine (this will handle the simulation loop)
    machine.spin();

    rclcpp::shutdown();
    return 0;
}
