#pragma once

#include <chrono>
#include <crl_ros_helper/meta.hpp>
#include <string>
#include <string_view>

namespace crl::fsm {
    using StatesRepr = signed char;
    using MachinesRepr = signed char;
    using namespace std::chrono_literals;

    template <typename Machines>
    std::string fsm_node_name(const std::string& prefix, Machines m) {
        return std::string("FSM_node_") + prefix + "_" + std::string(m.to_string());
    }

    std::string client_node_name(const std::string& prefix) {
        return std::string("FSM_client_") + prefix;
    }

    std::string client_comm_node_name(const std::string& prefix) {
        return std::string("FSM_comm_client_") + prefix;
    }

    constexpr std::string_view state_switch_srv_name("/state_switch");
    constexpr std::string_view state_info_msg_name("/state_info");

    constexpr auto srv_wait_timeout = 100ms;
    constexpr auto msg_publish_freq = 50ms;

    // matches int8 in srv definition
    ENUM17(StateSwitchRes, signed char, SUCCESS = 0, OUTOFRANGE = -1, CANNOT = -2, UNKNOWN = -3, TIMEOUT = -4)
}  // namespace crl::fsm

#define crl_fsm_states(name, ...) ENUM17(name, crl::fsm::StatesRepr, __VA_ARGS__)
#define crl_fsm_machines(name, ...) ENUM17(name, crl::fsm::MachinesRepr, __VA_ARGS__)
