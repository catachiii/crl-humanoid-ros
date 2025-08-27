#pragma once

#include <crl_ros_helper/node.h>

#include <atomic>
#include <chrono>
#include <crl_fsm_msgs/msg/state_info.hpp>
#include <crl_fsm_msgs/srv/state_switch.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <vector>

#include "blueprint.h"
#include "common.h"
#include "state.h"
#include "transitions.h"

namespace crl::fsm {
    using namespace crl::ros::meta;
    using namespace std::chrono_literals;

    template <typename States, typename Machines, std::size_t N>
    class ClientNode : public crl::ros::Node {
    public:
        ClientNode(const std::string& prefix, const std::array<Machines, N>& monitoring) : Node(client_node_name(prefix)), monitoring(monitoring) {
            for (std::size_t i = 0; i < N; i++) {
                clients[i] = create_client<crl_fsm_msgs::srv::StateSwitch>(fsm_node_name(prefix, monitoring[i]) + std::string(state_switch_srv_name));
                if (!clients[i]->wait_for_service(5s)) {
                    throw std::runtime_error("Client not connected after 5s.");
                }
            }
        }

        virtual ~ClientNode() {
            terminate();
        }

        auto broadcast_switch(States to) {
            std::array<rclcpp::Client<crl_fsm_msgs::srv::StateSwitch>::SharedFuture, N> responses;

            auto request = std::make_shared<crl_fsm_msgs::srv::StateSwitch::Request>();
            request->to = to;

            for (std::size_t i = 0; i < N; i++) {
                responses[i] = clients[i]->async_send_request(request);
            }

            return responses;
        }

    private:
        std::array<Machines, N> monitoring;
        std::array<rclcpp::Client<crl_fsm_msgs::srv::StateSwitch>::SharedPtr, N> clients;
    };

    template <typename States, typename Machines, std::size_t N>
    class ClientCommNode : public crl::ros::Node {
    public:
        ClientCommNode(const std::string& prefix, const std::array<Machines, N>& monitoring, std::array<std::atomic<States>, N>& states)
            : Node(client_comm_node_name(prefix)), monitoring(monitoring), states(states) {
            state_info_init_helper(prefix, std::make_index_sequence<N>{});
        }

        virtual ~ClientCommNode() {
            terminate();
        }

        template <std::size_t... Is>
        void state_info_init_helper(const std::string& prefix, const std::index_sequence<Is...>&) {
            ((state_info_options[Is].callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant)), ...);
            ((state_info_listeners[Is] = create_safe_subscription<crl_fsm_msgs::msg::StateInfo>(
                  fsm_node_name(prefix, monitoring[Is]) + std::string(state_info_msg_name), 10,
                  std::bind(&std::remove_reference_t<decltype(*this)>::template store_state_info<Is>, this, std::placeholders::_1), state_info_options[Is])),
             ...);
        }

        template <std::size_t I>
        void store_state_info(const crl_fsm_msgs::msg::StateInfo::SharedPtr message) {
            // avoid contention
            if (states[I].load().to_ut() != message->cur_state) {
                states[I] = message->cur_state;
            }
        }

    private:
        std::array<Machines, N> monitoring;
        std::array<rclcpp::SubscriptionOptions, N> state_info_options;
        std::array<rclcpp::Subscription<crl_fsm_msgs::msg::StateInfo>::SharedPtr, N> state_info_listeners;
        std::array<std::atomic<States>, N>& states;
    };

    // This performs broadcasting, with no switches
    template <typename States, typename Machines, std::size_t N>
    class Broadcaster {
    public:
        Broadcaster(const std::string& prefix, const std::array<Machines, N>& monitoring)
            : client_node(std::make_shared<std::remove_reference_t<decltype(*client_node)>>(prefix, monitoring)) {}

        StateSwitchRes broadcast_switch(States to) {
            auto responses = client_node->broadcast_switch(to);

            for (auto& response : responses) {
                if (!(rclcpp::spin_until_future_complete(client_node, response, srv_wait_timeout) == rclcpp::FutureReturnCode::SUCCESS)) {
                    return StateSwitchRes::TIMEOUT;
                } else if (response.get()->ret != StateSwitchRes::SUCCESS) {
                    return response.get()->ret;
                }
            }
            return StateSwitchRes::SUCCESS;
        }

    private:
        std::shared_ptr<ClientNode<States, Machines, N>> client_node;
    };

    template <typename States, typename Machines, std::size_t N>
    class StateInformer {
        static_assert(N != 0, "Cannot have no machines being observed");

    public:
        StateInformer(const std::string& prefix, const std::array<Machines, N>& monitoring)
            : client_comm_node(std::make_shared<std::remove_reference_t<decltype(*client_comm_node)>>(prefix, monitoring, states)), exectutor_thread{} {
            executor.add_node(client_comm_node);
            exectutor_thread = std::thread([&]() { executor.spin(); });
        }

        ~StateInformer() {
            executor.cancel();
            exectutor_thread.join();
        }

        auto get_states() {
            std::array<States, N> ret;
            for (std::size_t i = 0; i < N; i++) {
                ret[i] = states[i];
            }
            return ret;
        }

        void get_states(std::array<States, N>& ret) {
            for (std::size_t i = 0; i < N; i++) {
                ret[i] = states[i];
            }
        }

        auto get_first_state() {
            return states[0].load();
        }

    protected:
        std::array<std::atomic<States>, N> states;

    private:
        rclcpp::executors::MultiThreadedExecutor executor;
        std::shared_ptr<ClientCommNode<States, Machines, N>> client_comm_node;
        std::thread exectutor_thread;
    };

    // This client performs checks for transition feasibility
    // Also receives the states of the machines
    // Designed to be standalone and for debugging
    template <typename States, typename Machines, typename TransitionsCont, std::size_t N>
    class Client : public Broadcaster<States, Machines, N>, public StateInformer<States, Machines, N> {
        using ParentBroadcaster = Broadcaster<States, Machines, N>;
        using ParentStateInformer = StateInformer<States, Machines, N>;

    public:
        Client(const std::string& prefix, const TransitionsCont& trans_cont, const std::array<Machines, N>& monitoring)
            : ParentBroadcaster(prefix, monitoring), ParentStateInformer(prefix, monitoring), trans_cont(trans_cont) {
            // sleep for msg to be populated
            std::this_thread::sleep_for(msg_publish_freq);

            States cur_state = ParentStateInformer::states[0];
            for (int i = 1; i < N; i++) {
                if (cur_state != ParentStateInformer::states[i].load()) {
                    assert(false && "States do not match");
                }
            }
        }

        StateSwitchRes checked_broadcast_switch(States to) {
            // Sanity check
            States cur_state = ParentStateInformer::states[0];
            TransitionsPossible res = trans_cont.possible(cur_state, to);

            if (res == TransitionsPossible::YES) {
                return ParentBroadcaster::broadcast_switch(to);
            } else if (res == TransitionsPossible::NO) {
                return StateSwitchRes::CANNOT;
            } else if (res == TransitionsPossible::OUTOFRANGE) {
                return StateSwitchRes::OUTOFRANGE;
            } else {
                return StateSwitchRes::UNKNOWN;
            }
        }

    private:
        const TransitionsCont trans_cont;
    };

    template <typename States, typename Machines, typename TransitionsContGen, std::size_t N>
    auto make_client(const std::string& prefix, const TransitionsContGen& trans_cont, const std::array<Machines, N>& monitoring) {
        return Client<States, Machines, std::invoke_result_t<TransitionsContGen>, N>(prefix, trans_cont(), monitoring);
    }

}  // namespace crl::fsm
