#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tuple>

#include "common.h"

namespace crl::fsm {
    using namespace crl::ros;

    template <typename T, MachinesRepr MachineID, StatesRepr StateID>
    class PartialStateTemplate {
    public:
        static constexpr MachinesRepr machine = MachineID;
        static constexpr StatesRepr state = StateID;

        void wait_until_startable() {
            static_cast<T*>(this)->wait_until_startable_cb();
        }

        void wait_until_terminatable() {
            static_cast<T*>(this)->wait_until_terminatable_cb();
        }

        void begin(rclcpp::Executor& executor) {
            static_cast<T*>(this)->begin_cb(executor);
        }

        void end(rclcpp::Executor& executor) {
            static_cast<T*>(this)->end_cb(executor);
        }

    private:
        void wait_until_startable_cb() {
            static_assert(meta::always_false_v<T>, "PartialState void wait_until_startable_cb() needs implementation");
        }

        void wait_until_terminatable_cb() {
            static_assert(meta::always_false_v<T>, "PartialState void wait_until_terminatable_cb() needs implementation");
        }

        void begin_cb(rclcpp::Executor&) {
            static_assert(meta::always_false_v<T>, "PartialState void begin_cb(rclcpp::Executor& executor) needs implementation");
        }

        void end_cb(rclcpp::Executor&) {
            static_assert(meta::always_false_v<T>, "PartialState void end_cb(rclcpp::Executor& executor) needs implementation");
        }
    };

    template <typename... GenPartialTs>
    class StateSelect {
    public:
        constexpr StateSelect(const std::tuple<GenPartialTs...>& partial_states) : partial_states(partial_states) {}

        constexpr StateSelect(const GenPartialTs&... partial_states) : partial_states(std::make_tuple(partial_states...)) {}

        template <MachinesRepr MachineID, StatesRepr StateID>
        constexpr auto select() const {
            using namespace meta;

            constexpr auto state_mask = gen_state_mask<StateID>();
            constexpr auto machine_mask = gen_machine_mask<MachineID>();

            return get_masked(partial_states, state_mask & machine_mask);
        }

    private:
        template <MachinesRepr MachineID>
        static constexpr auto gen_machine_mask() {
            return std::make_tuple(std::conditional_t<std::invoke_result_t<GenPartialTs>::machine == MachineID, std::true_type, std::false_type>{}...);
        }

        template <StatesRepr StateID>
        static constexpr auto gen_state_mask() {
            return std::make_tuple(std::conditional_t<std::invoke_result_t<GenPartialTs>::state == StateID, std::true_type, std::false_type>{}...);
        }

        const std::tuple<GenPartialTs...> partial_states;
    };

    template <StatesRepr S1, StatesRepr S2>
    class Transition {
    public:
        static constexpr StatesRepr from = S1;
        static constexpr StatesRepr to = S2;
    };

    template <typename... TransitionTs>
    class TransitionSelect {
    public:
        constexpr TransitionSelect(const std::tuple<TransitionTs...>& transitions) : transitions(transitions) {}

        constexpr TransitionSelect(const TransitionTs&... transitions) : transitions(std::make_tuple(transitions...)) {}

        template <StatesRepr FromS>
        constexpr auto select_from() const {
            return get_masked(transitions, gen_from_mask<FromS>());
        }

        template <StatesRepr ToS>
        constexpr auto select_to() const {
            return get_masked(transitions, gen_to_mask<ToS>());
        }

        template <StatesRepr FromS, StatesRepr ToS>
        constexpr auto select() const {
            using namespace meta;

            constexpr auto from_mask = gen_from_mask<FromS>();
            constexpr auto to_mask = gen_to_mask<ToS>();

            return get_masked(transitions, from_mask & to_mask);
        }

        template <StatesRepr FromS, StatesRepr ToS>
        static constexpr bool exists() {
            using namespace meta;

            constexpr auto from_mask = gen_from_mask<FromS>();
            constexpr auto to_mask = gen_to_mask<ToS>();
            auto sum_type = [](auto const&... inp) { return (inp.value + ...); };
            return std::apply(sum_type, from_mask & to_mask);
        }

    private:
        template <StatesRepr FromS>
        static constexpr auto gen_from_mask() {
            return std::make_tuple(std::conditional_t<TransitionTs::from == FromS, std::true_type, std::false_type>{}...);
        }

        template <StatesRepr ToS>
        static constexpr auto gen_to_mask() {
            return std::make_tuple(std::conditional_t<TransitionTs::to == ToS, std::true_type, std::false_type>{}...);
        }

        const std::tuple<TransitionTs...> transitions;
    };

}  // namespace crl::fsm