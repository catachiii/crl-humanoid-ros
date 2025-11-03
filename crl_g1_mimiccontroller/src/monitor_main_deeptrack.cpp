#include "crl_humanoid_monitor/MuJoCoMonitorApp.h"
#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/RobotParameters.h"
#include "crl_fsm/client.h"

// FSM states and machines - match the G1 RL controller simulator exactly
crl_fsm_states(States, ESTOP, STAND, WALK, GETUP0, CROUCH, GETUP1, SITDOWN, DEEPTRACK);
crl_fsm_machines(Machines, ONBOARD);

int main(int argc, char ** argv)
{
  // This should have the same FSM logic as in crl_g1_mimiccontroller simulator_main.cpp
  // transitions - matching the G1 RL controller FSM
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
  crl::fsm::Transition<States::WALK, States::DEEPTRACK> tm38; // WALK(3) -> DEEPTRACK(8)
  crl::fsm::Transition<States::DEEPTRACK, States::WALK> tm83; // DEEPTRACK(8) -> WALK(3)

  constexpr auto t_cols = crl::fsm::make_transitions_collection<States>(
        tm12, tm21, tm52, tm25, tm51, tm41, tm42, tm43, tm24, tm23,
        tm31, tm32, tm61, tm56, tm62, tm63, tm37, tm72, tm71, tm38, tm83);

  std::array<Machines, 1> monitoring = {Machines::ONBOARD};

  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and run the MuJoCo monitor app with G1 RL controller FSM
  auto app = crl::humanoid::monitor::make_mujoco_monitor_app<States, Machines, decltype(t_cols), 1>(
    "robot", t_cols, monitoring);
  app.run();

  rclcpp::shutdown();
  return 0;
}
