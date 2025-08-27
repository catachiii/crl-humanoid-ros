#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "blueprint.h"
#include "common.h"

namespace crl::fsm {
    using namespace crl::ros;

    template <MachinesRepr MachineID, StatesRepr StateID, typename... PartialGenTs>
    class MachineState {
    public:
        static constexpr StatesRepr state = StateID;

        constexpr MachineState(PartialGenTs... generators) : persistent_partials(generators()...) {}

        void wait_until_startable() {
            wait_until_startable_impl(std::make_index_sequence<sizeof...(PartialGenTs)>{});
        }

        void wait_until_terminatable() {
            wait_until_terminatable_impl(std::make_index_sequence<sizeof...(PartialGenTs)>{});
        }

        void begin(rclcpp::Executor& executor) {
            begin_impl(executor, std::make_index_sequence<sizeof...(PartialGenTs)>{});
        }

        void end(rclcpp::Executor& executor) {
            end_impl(executor, std::make_index_sequence<sizeof...(PartialGenTs)>{});
        }

    private:
        template <std::size_t... Is>
        void wait_until_startable_impl(const std::index_sequence<Is...>&) {
            (std::get<Is>(persistent_partials).wait_until_startable(), ...);
        }

        template <std::size_t... Is>
        void wait_until_terminatable_impl(const std::index_sequence<Is...>&) {
            (std::get<Is>(persistent_partials).wait_until_terminatable(), ...);
        }

        template <std::size_t... Is>
        void begin_impl(rclcpp::Executor& executor, const std::index_sequence<Is...>&) {
            (std::get<Is>(persistent_partials).begin(executor), ...);
        }

        template <std::size_t... Is>
        void end_impl(rclcpp::Executor& executor, const std::index_sequence<Is...>&) {
            (std::get<Is>(persistent_partials).end(executor), ...);
        }

        std::tuple<std::invoke_result_t<PartialGenTs>...> persistent_partials;
    };

    template <MachinesRepr MachineID, StatesRepr StateID>
    struct make_machine_state_helper {
        template <typename... GenT, std::size_t... Is>
        static constexpr auto gen(const std::tuple<GenT...>& generators, const std::index_sequence<Is...>&) {
            return [=] { return MachineState<MachineID, StateID, GenT...>(std::get<Is>(generators)...); };
        }
    };

    template <MachinesRepr MachineID, StatesRepr StateID, typename... PartialS>
    constexpr auto make_machine_state(const StateSelect<PartialS...>& state_select) {
        auto generators = state_select.template select<MachineID, StateID>();
        return make_machine_state_helper<MachineID, StateID>::gen(generators, std::make_index_sequence<std::tuple_size_v<decltype(generators)>>{});
    }

    template <typename... StateTs>
    class StatesCollection {
    public:
        constexpr StatesCollection(const StateTs&... machine_states) : machine_states(machine_states...) {}

        constexpr static std::size_t find_state(StatesRepr state) {
            return find_state_helper<0>(state);
        }

        void wait_until_startable(StatesRepr state) {
            std::size_t cur_state = find_state(state);
            meta::tuple_rt_get([&](auto& inp) { inp.wait_until_startable(); }, machine_states, cur_state);
        }

        void wait_until_terminatable(StatesRepr state) {
            std::size_t cur_state = find_state(state);
            meta::tuple_rt_get([&](auto& inp) { inp.wait_until_terminatable(); }, machine_states, cur_state);
        }

        void begin(rclcpp::Executor& executor, StatesRepr state) {
            std::size_t cur_state = find_state(state);
            meta::tuple_rt_get([&](auto& inp) { inp.begin(executor); }, machine_states, cur_state);
        }

        void end(rclcpp::Executor& executor, StatesRepr state) {
            std::size_t cur_state = find_state(state);
            meta::tuple_rt_get([&](auto& inp) { inp.end(executor); }, machine_states, cur_state);
        }

    private:
        template <std::size_t cur_ind>
        static constexpr std::size_t find_state_helper(StatesRepr state) {
            if constexpr (cur_ind == sizeof...(StateTs)) {
                return 0;
            } else if (std::tuple_element_t<cur_ind, std::tuple<StateTs...>>::state == state) {
                return cur_ind;
            } else {
                return find_state_helper<cur_ind + 1>(state);
            }
        }

        std::tuple<StateTs...> machine_states;
    };

    template <typename... GenTs>
    constexpr auto make_states_collection(const GenTs&... machine_states_gen) {
        return [=] { return StatesCollection<std::invoke_result_t<GenTs>...>(machine_states_gen()...); };
    }

    template <MachinesRepr MachineID, typename States, typename... PartialTs, std::size_t... Is>
    constexpr auto make_states_collection_for_machine_helper(const std::index_sequence<Is...>&, const PartialTs&... partials) {
        fsm::StateSelect ss(partials...);
        return make_states_collection(fsm::make_machine_state<MachineID, States::from_ind(Is)>(ss)...);
    }

    template <MachinesRepr MachineID, typename States, typename... PartialTs>
    constexpr auto make_states_collection_for_machine(const PartialTs&... partials) {
        return make_states_collection_for_machine_helper<MachineID, States>(std::make_index_sequence<States::num_params>(), partials...);
    }

    template <MachinesRepr MachineID, typename States, typename... PartialTs>
    constexpr auto make_states_collection_for_machine(const std::tuple<PartialTs...>& partials_tup) {
        return std::apply([](const PartialTs&... partials) { return make_states_collection_for_machine<MachineID, States>(partials...); }, partials_tup);
    }
}  // namespace crl::fsm