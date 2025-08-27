#pragma once

#include "blueprint.h"

namespace crl::fsm {
    template <MachinesRepr MachineID, StatesRepr StateID, typename... GenT>
    class NonPersistentPartialState : public PartialStateTemplate<NonPersistentPartialState<MachineID, StateID, GenT...>, MachineID, StateID> {
    public:
        constexpr NonPersistentPartialState(GenT... generators) : generators(std::make_tuple(generators...)) {}

        void wait_until_startable_cb() {
            wait_until_startable_cb_impl(std::make_index_sequence<sizeof...(GenT)>{});
        }

        void wait_until_terminatable_cb() {
            wait_until_terminatable_cb_impl(std::make_index_sequence<sizeof...(GenT)>{});
        }

        void begin_cb(rclcpp::Executor& executor) {
            begin_cb_impl(executor, std::make_index_sequence<sizeof...(GenT)>{});
        }

        void end_cb(rclcpp::Executor& executor) {
            end_cb_impl(executor, std::make_index_sequence<sizeof...(GenT)>{});
        }

    private:
        template <std::size_t... Is>
        void wait_until_startable_cb_impl(const std::index_sequence<Is...>&) {
            (std::get<Is>(nodes).value()->wait_until_startable(), ...);
        }

        template <std::size_t... Is>
        void wait_until_terminatable_cb_impl(const std::index_sequence<Is...>&) {
            (std::get<Is>(nodes).value()->wait_until_terminatable(), ...);
        }

        template <std::size_t... Is>
        void begin_cb_impl(rclcpp::Executor& executor, const std::index_sequence<Is...>&) {
            (std::get<Is>(nodes).emplace(std::get<Is>(generators)()), ...);
            (executor.add_node(std::get<Is>(nodes).value()), ...);
        }

        template <std::size_t... Is>
        void end_cb_impl(rclcpp::Executor& executor, const std::index_sequence<Is...>&) {
            (executor.remove_node(std::get<Is>(nodes).value()), ...);
            (std::get<Is>(nodes).reset(), ...);
        }

        const std::tuple<GenT...> generators;
        std::tuple<std::optional<std::invoke_result_t<GenT>>...> nodes;
    };

    template <MachinesRepr MachineID, StatesRepr StateID, typename... GenT>
    constexpr auto make_non_persistent_ps(GenT... generators) {
        return [=] { return NonPersistentPartialState<MachineID, StateID, GenT...>(generators...); };
    }

    template <MachinesRepr MachineID, StatesRepr StateID, typename T1, typename T2>
    class PersistentPartialState;

    template <MachinesRepr MachineID, StatesRepr StateID, typename... PersisT, typename... GenT>
    class PersistentPartialState<MachineID, StateID, std::tuple<PersisT...>, std::tuple<GenT...>>
        : public PartialStateTemplate<PersistentPartialState<MachineID, StateID, std::tuple<PersisT...>, std::tuple<GenT...>>, MachineID, StateID> {
    public:
        constexpr PersistentPartialState(const std::tuple<PersisT...>& persistents, GenT... generators)
            : persistents(persistents), generators(std::make_tuple(generators...)) {}

        void wait_until_startable_cb() {
            wait_until_startable_cb_impl(std::make_index_sequence<sizeof...(GenT)>{});
        }

        void wait_until_terminatable_cb() {
            wait_until_terminatable_cb_impl(std::make_index_sequence<sizeof...(GenT)>{});
        }

        void begin_cb(rclcpp::Executor& executor) {
            begin_cb_impl(executor, std::make_index_sequence<sizeof...(GenT)>{});
        }

        void end_cb(rclcpp::Executor& executor) {
            end_cb_impl(executor, std::make_index_sequence<sizeof...(GenT)>{});
        }

    private:
        template <std::size_t... Is>
        void wait_until_startable_cb_impl(const std::index_sequence<Is...>&) {
            (std::get<Is>(nodes).value()->wait_until_startable(), ...);
        }

        template <std::size_t... Is>
        void wait_until_terminatable_cb_impl(const std::index_sequence<Is...>&) {
            (std::get<Is>(nodes).wait_until_terminatable(), ...);
        }

        template <std::size_t... Is>
        void begin_cb_impl(rclcpp::Executor& executor, const std::index_sequence<Is...>&) {
            (std::get<Is>(nodes).emplace(std::apply(std::get<Is>(generators), persistents)), ...);
            (executor.add_node(std::get<Is>(nodes).value()), ...);
        }

        template <std::size_t... Is>
        void end_cb_impl(rclcpp::Executor& executor, const std::index_sequence<Is...>&) {
            (executor.remove_node(std::get<Is>(nodes).value()), ...);
            (std::get<Is>(nodes).reset(), ...);
        }

        const std::tuple<GenT...> generators;
        std::tuple<PersisT...> persistents;
        //        no rvalues
        std::tuple<std::optional<std::invoke_result_t<GenT, PersisT&...>>...> nodes;
    };

    template <MachinesRepr MachineID, StatesRepr StateID, typename... PersisT, typename... GenT>
    constexpr auto make_persistent_ps(const std::tuple<PersisT...>& persistents, GenT... generators) {
        return [=] { return PersistentPartialState<MachineID, StateID, std::tuple<PersisT...>, std::tuple<GenT...>>(persistents, generators...); };
    }

}  // namespace crl::fsm