#pragma once

#include <crl_ros_helper/node.h>

#include <atomic>
#include <crl_fsm_msgs/msg/state_info.hpp>
#include <crl_fsm_msgs/srv/state_switch.hpp>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>
#include <utility>

#include "blueprint.h"
#include "common.h"
#include "partial_state.h"
#include "state.h"
#include "transitions.h"

namespace crl::fsm {
    using namespace crl::ros::meta;

    template <typename Machines, MachinesRepr MachineID, typename StatesCont, typename TransitionsCont>
    class FSMNode : public crl::ros::Node {
    public:
        FSMNode(const std::string& prefix, std::atomic<StatesRepr>& cur_state, const StatesCont& state_cont_inp, const TransitionsCont& trans_cont,
                rclcpp::Executor& executor, std::atomic<bool>& transitioning)
            : Node(fsm_node_name(prefix, Machines(MachineID))),
              executor(executor),
              transitioning(transitioning),
              cur_state(cur_state),
              state_cont(state_cont_inp),
              trans_cont(trans_cont) {
            transitioning = true;
            switch_service = create_safe_service<crl_fsm_msgs::srv::StateSwitch>(
                get_name() + std::string(state_switch_srv_name),
                std::bind(&std::remove_reference_t<decltype(*this)>::state_switch, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default, switch_cbg);
            state_cont.begin(executor, cur_state);
            transitioning = false;
            state_info_publisher = create_publisher<crl_fsm_msgs::msg::StateInfo>(get_name() + std::string(state_info_msg_name), 10);
            state_info_timer = create_safe_wall_timer(msg_publish_freq, std::bind(&std::remove_reference_t<decltype(*this)>::publish, this), publish_cbg);
        }

        virtual ~FSMNode() {
            terminate();
        }

        void state_switch(const std::shared_ptr<crl_fsm_msgs::srv::StateSwitch::Request> request,
                          std::shared_ptr<crl_fsm_msgs::srv::StateSwitch::Response> response) {
            TransitionsPossible res = trans_cont.possible(cur_state, request->to);
            if (res == TransitionsPossible::YES) {
                state_cont.wait_until_terminatable(cur_state);
                transitioning = true;
                state_cont.end(executor, cur_state);
                cur_state = request->to;
                state_cont.begin(executor, cur_state);
                state_cont.wait_until_startable(cur_state);
                transitioning = false;
                response->ret = StateSwitchRes::SUCCESS;
            } else if (res == TransitionsPossible::NO) {
                response->ret = StateSwitchRes::CANNOT;
            } else if (res == TransitionsPossible::OUTOFRANGE) {
                response->ret = StateSwitchRes::OUTOFRANGE;
            } else {
                response->ret = StateSwitchRes::UNKNOWN;
            }
        }

    private:
        void publish() {
            auto message = crl_fsm_msgs::msg::StateInfo();

            message.cur_state = cur_state;

            state_info_publisher->publish(message);
        }

    private:
        rclcpp::Executor& executor;
        rclcpp::Service<crl_fsm_msgs::srv::StateSwitch>::SharedPtr switch_service;
        rclcpp::Publisher<crl_fsm_msgs::msg::StateInfo>::SharedPtr state_info_publisher;
        rclcpp::TimerBase::SharedPtr state_info_timer;

        std::atomic<bool>& transitioning;
        std::atomic<StatesRepr>& cur_state;
        StatesCont state_cont;
        const TransitionsCont trans_cont;

        rclcpp::CallbackGroup::SharedPtr switch_cbg = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::CallbackGroup::SharedPtr publish_cbg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    };

    template <typename Machines, MachinesRepr MachineID, typename StatesCont, typename TransitionsCont>
    class FSM {
    public:
        FSM(const std::string& prefix, StatesRepr starting_state, const StatesCont& state_cont_inp, const TransitionsCont& trans_cont)
            : cur_state(starting_state),
              fsm_node(std::make_shared<std::remove_reference_t<decltype(*fsm_node)>>(prefix, cur_state, state_cont_inp, trans_cont, executor, transitioning)) {
            executor.add_node(fsm_node);
        }

        void spin() {
            executor.spin();
        }

        void cancel() {
            executor.cancel();
        }

        const std::atomic<bool>& is_transitioning() {
            return transitioning;
        }

        rclcpp::Executor& get_executor() {
            return executor;
        }

    private:
        std::atomic<bool> transitioning;
        std::atomic<StatesRepr> cur_state;
        rclcpp::executors::MultiThreadedExecutor executor;
        std::shared_ptr<FSMNode<Machines, MachineID, StatesCont, TransitionsCont>> fsm_node;
    };

    template <typename Machines, MachinesRepr MachineID, typename StatesContGen, typename TransitionsContGen>
    auto make_fsm(const std::string& prefix, StatesRepr starting_state, const StatesContGen& state_cont, const TransitionsContGen& trans_cont) {
        return FSM<Machines, MachineID, std::invoke_result_t<StatesContGen>, std::invoke_result_t<TransitionsContGen>>(prefix, starting_state, state_cont(),
                                                                                                                       trans_cont());
    }
}  // namespace crl::fsm
