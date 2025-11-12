#include <crl_fsm/fsm.h>
#include <crl_humanoid_commons/RobotData.h>
#include <crl_humanoid_commons/nodes/ControllerNode.h>
#include <crl_humanoid_commons/nodes/EstopNode.h>
#include <crl_humanoid_commons/nodes/StarterNode.h>
#include <crl_humanoid_commons/nodes/CommNode.h>
#include <crl_humanoid_commons/nodes/MocapNode.h>
#include <crl_humanoid_commons/nodes/PyControllerNode.h>
#include <crl_humanoid_hardware/G1Node.h>
#include <crl_g1_rlcontroller/CRLG1RLControllerNode.h>
#include <crl_g1_rlcontroller/CRLG1WalkController.h>
#include <crl_g1_rlcontroller/CRLG1GetUpController.h>
#include <crl_g1_rlcontroller/CRLG1SitDownController.h>
#include <cxxopts.hpp>


crl_fsm_states(States, ESTOP, STAND, WALK, GETUP0, CROUCH, GETUP1, SITDOWN, GOAL);
crl_fsm_machines(Machines, ONBOARD);

int main(int argc, char** argv) {
    // parse arguments
    cxxopts::Options options("hardware", "crl_g1_goalcontroller_hardware");
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
    crl::fsm::Transition<States::ESTOP, States::STAND> tm12;     // ESTOP(1) -> STAND(2)
    crl::fsm::Transition<States::STAND, States::ESTOP> tm21;     // STAND(2) -> ESTOP(1)
    crl::fsm::Transition<States::CROUCH, States::STAND> tm52;     // CROUCH(5) -> STAND(2)
    crl::fsm::Transition<States::STAND, States::CROUCH> tm25;     // STAND(2) -> CROUCH(5)
    crl::fsm::Transition<States::CROUCH, States::ESTOP> tm51;     // CROUCH(5) -> ESTOP(1)
    crl::fsm::Transition<States::GETUP0, States::ESTOP> tm41;     // GETUP0(4) -> ESTOP(1)
    crl::fsm::Transition<States::GETUP0, States::STAND> tm42;     // GETUP0(4) -> STAND(2)
    crl::fsm::Transition<States::GETUP0, States::WALK> tm43;     // GETUP0(4) -> WALK(3)
    crl::fsm::Transition<States::STAND, States::GETUP0> tm24;     // STAND(2) -> GETUP0(4)
    crl::fsm::Transition<States::STAND, States::WALK> tm23;      // STAND(2) -> WALK(3)
    crl::fsm::Transition<States::WALK, States::ESTOP> tm31;      // WALK(3) -> ESTOP(1)
    crl::fsm::Transition<States::WALK, States::STAND> tm32;      // WALK(3) -> STAND(2)
    crl::fsm::Transition<States::GETUP1, States::ESTOP> tm61;     // GETUP1(6) -> ESTOP(1)
    crl::fsm::Transition<States::CROUCH, States::GETUP1> tm56;     // CROUCH(5) -> GETUP1(6)
    crl::fsm::Transition<States::GETUP1, States::STAND> tm62;     // GETUP1(6) -> STAND(2)
    crl::fsm::Transition<States::GETUP1, States::WALK> tm63;     // GETUP1(6) -> WALK(3)
    crl::fsm::Transition<States::WALK, States::SITDOWN> tm37;    // WALK(3) -> SITDOWN(7)
    crl::fsm::Transition<States::SITDOWN, States::STAND> tm72;   // SITDOWN(7) -> STAND(2)
    crl::fsm::Transition<States::SITDOWN, States::ESTOP> tm71;   // SITDOWN(7) -> ESTOP(1)
    crl::fsm::Transition<States::WALK, States::GOAL> tm38; // WALK(3) -> GOAL(8)
    crl::fsm::Transition<States::GOAL, States::WALK> tm83; // GOAL(8) -> WALK(3)
    crl::fsm::Transition<States::GOAL, States::ESTOP> tm81; // GOAL(8) -> ESTOP(1)

    // transitions collection and monitoring list must be defined before state generators use them
    constexpr auto t_cols = crl::fsm::make_transitions_collection<States>(
        tm12, tm21, tm52, tm25, tm51, tm41, tm42, tm43, tm24, tm23,
        tm31, tm32, tm61, tm56, tm62, tm63, tm37, tm72, tm71, tm38, tm83, tm81);
    std::array<Machines, 1> monitoring = {Machines::ONBOARD};

    // data
    const auto model = std::make_shared<crl::humanoid::commons::RobotModel>(crl::humanoid::commons::robotModels.at(modelName));
    const auto data = std::make_shared<crl::humanoid::commons::RobotData>(model->jointNames, model->defaultJointConf);

    // machines
    auto m1 =
        crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::ESTOP>([&]() { return std::make_shared<crl::humanoid::commons::EstopNode>(model, data); });
    auto m2 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::STAND>(
        [&]() { return std::make_shared<crl::humanoid::commons::StarterNode>(crl::humanoid::commons::StarterNode::TargetMode::STAND, model, data, "stand"); });
    auto m3 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::WALK>(
        [&]() { return std::make_shared<crl::g1::rlcontroller::CRLG1RLControllerNode<crl::g1::rlcontroller::CRLG1WalkController>>(model, data, "walk"); });
    auto m4 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::GETUP0>(
        [&]() { return std::make_shared<crl::g1::rlcontroller::CRLG1RLControllerNode<crl::g1::rlcontroller::CRLG1GetUpController>>(model, data, "getup0"); });
    auto m5 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::CROUCH>(
        [&]() { return std::make_shared<crl::humanoid::commons::StarterNode>(crl::humanoid::commons::StarterNode::TargetMode::CROUCH, model, data, "crouch"); });
    auto m6 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::GETUP1>(
        [&]() { return std::make_shared<crl::g1::rlcontroller::CRLG1RLControllerNode<crl::g1::rlcontroller::CRLG1GetUpController>>(model, data, "getup1"); });
    auto m7 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::SITDOWN>(
        [&]() { return std::make_shared<crl::g1::rlcontroller::CRLG1RLControllerNode<crl::g1::rlcontroller::CRLG1SitDownController>>(model, data, "sitdown"); });
    auto m8 = crl::fsm::make_non_persistent_ps<Machines::ONBOARD, States::GOAL>(
        [&]() { return std::make_shared<crl::humanoid::commons::PyControllerNode>(model, data); });


    auto s_cols = crl::fsm::make_states_collection_for_machine<Machines::ONBOARD, States>(m1, m2, m3, m4, m5, m6, m7, m8);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // init ros process
    auto machine = crl::fsm::make_fsm<Machines, Machines::ONBOARD>("robot", States::ESTOP, s_cols, t_cols);

    // Create robot node (real hardware communication) - using G1Node
    const auto commNode = std::make_shared<crl::humanoid::commons::CommNode>(model, data);
    const auto mocapNode = std::make_shared<crl::humanoid::commons::MocapNode>(model, data);
    std::shared_ptr<crl::unitree::hardware::g1::G1Node<States, Machines, 1>> robotNode;

    if (modelName == "G1") {
        // Create state name to enum mapping for G1Node with all states
        using G1NodeType = crl::unitree::hardware::g1::G1Node<States, Machines, 1>;
        auto stateMapping = G1NodeType::createStateMapping({
            {"ESTOP", States::ESTOP},
            {"STAND", States::STAND},
            {"WALK", States::WALK},
            {"GETUP0", States::GETUP0},
            {"CROUCH", States::CROUCH},
            {"GETUP1", States::GETUP1},
            {"SITDOWN", States::SITDOWN},
            {"GOAL", States::GOAL}
        });

        robotNode = std::make_shared<G1NodeType>(
            model, data, monitoring, machine.is_transitioning(), stateMapping);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "Unsupported robot model: %s", modelName.c_str());
        return -1;
    }

    auto& executor = machine.get_executor();
    executor.add_node(robotNode);
    executor.add_node(commNode);
    executor.add_node(mocapNode);

    // Start the FSM machine (this will handle the hardware communication loop)
    machine.spin();

    rclcpp::shutdown();
    return 0;
}
