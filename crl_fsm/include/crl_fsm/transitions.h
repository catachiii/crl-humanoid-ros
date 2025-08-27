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
    using namespace crl::ros::meta;

    ENUM17(TransitionsPossible, int, YES = 1, NO = 0, OUTOFRANGE = -1)

    template <typename States>
    class TransitionsCollection {
    public:
        constexpr TransitionsCollection(const std::array<std::array<bool, States::num_params>, States::num_params>& transition_matrix)
            : transition_matrix(transition_matrix){};

        constexpr TransitionsPossible possible(StatesRepr from, StatesRepr to) const {
            if (from < 0 || from >= static_cast<MachinesRepr>(States::num_params)) {
                return TransitionsPossible::OUTOFRANGE;
            } else if (to < 0 || to >= static_cast<MachinesRepr>(States::num_params)) {
                return TransitionsPossible::OUTOFRANGE;
            } else if (transition_matrix[from][to]) {
                return TransitionsPossible::YES;
            } else {
                return TransitionsPossible::NO;
            }
        }

    private:
        const std::array<std::array<bool, States::num_params>, States::num_params> transition_matrix;
    };

    template <typename States, typename... TransitionTs>
    struct make_transitions_collection_helper {
        constexpr make_transitions_collection_helper(const TransitionTs&... transitions) : ts(transitions...){};

        template <std::size_t I, std::size_t... Js>
        constexpr static std::array<bool, States::num_params> comp_transition_matrix_row(const std::index_sequence<Js...>&) {
            return {decltype(ts)::template exists<States(I), States(Js)>()...};
        }

        template <std::size_t... Is>
        constexpr static std::array<std::array<bool, States::num_params>, States::num_params> comp_transition_matrix_helper(
            const std::index_sequence<Is...>& inp) {
            return {comp_transition_matrix_row<States(Is)>(inp)...};
        }

        constexpr static auto comp_transition_matrix() {
            return comp_transition_matrix_helper(std::make_index_sequence<States::num_params>{});
        }

        const TransitionSelect<TransitionTs...> ts;
    };

    template <typename States, typename... TransitionTs>
    constexpr auto make_transitions_collection(const TransitionTs&... transitions) {
        auto helper = make_transitions_collection_helper<States, TransitionTs...>(transitions...);
        return [comp_mat = helper.comp_transition_matrix()] { return TransitionsCollection<States>(comp_mat); };
    }

    template <typename States, typename... TransitionTs>
    constexpr auto make_transitions_collection(const std::tuple<TransitionTs...>& transitions_tup) {
        return std::apply([](const TransitionTs&... transitions) { return make_transitions_collection<States>(transitions...); }, transitions_tup);
    }
}  // namespace crl::fsm
