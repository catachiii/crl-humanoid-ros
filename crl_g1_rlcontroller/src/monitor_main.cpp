#include "crl_humanoid_monitor/MuJoCoMonitorApp.h"
#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_commons/RobotParameters.h"
#include "crl_fsm/client.h"

// FSM states and machines - match the G1 RL controller simulator exactly
crl_fsm_states(States, ESTOP, STAND, WALK);
crl_fsm_machines(Machines, ONBOARD);

int main(int argc, char ** argv)
{
  // This should have the same FSM logic as in crl_g1_rlcontroller simulator_main.cpp
  // transitions - matching the G1 RL controller FSM
  crl::fsm::Transition<States::ESTOP, States::STAND> t1;
  crl::fsm::Transition<States::STAND, States::ESTOP> t2;
  crl::fsm::Transition<States::STAND, States::WALK> t3;
  crl::fsm::Transition<States::WALK, States::ESTOP> t4;
  crl::fsm::Transition<States::WALK, States::STAND> t5;

  constexpr auto t_cols = crl::fsm::make_transitions_collection<States>(t1, t2, t3, t4, t5);

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
