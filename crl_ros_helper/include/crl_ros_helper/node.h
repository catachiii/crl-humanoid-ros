#pragma once

#include <functional>
#include <rclcpp/function_traits.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gate.h"

namespace crl::ros {

    class Node : public rclcpp::Node {
        using rclcpp::Node::Node;

    private:
        template <typename F, std::size_t... Is>
        auto safe_wrapper_helper(F &&f, const std::index_sequence<Is...> &) {
            using traits = rclcpp::function_traits::function_traits<F>;
            using desired_func_type = typename traits::return_type(typename traits::template argument_type<Is>...);
            return std::function<desired_func_type>([this, f](auto &&...args) {
                GateWrapper wrapper(gate);
                if (!wrapper.is_succ()) {
                    return;
                }
                f(std::forward<decltype(args)>(args)...);
            });
        }

        template <typename F>
        auto safe_wrapper(F &&f) {
            using traits = rclcpp::function_traits::function_traits<F>;
            return safe_wrapper_helper<F>(std::forward<F>(f), std::make_index_sequence<traits::arity>{});
        }

    public:
        template <typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
                  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
                  typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType>
        std::shared_ptr<SubscriptionT> create_safe_subscription(
            const std::string &topic_name, const rclcpp::QoS &qos, CallbackT &&callback,
            const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
            typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (MessageMemoryStrategyT::create_default())) {
            auto wrapped = safe_wrapper(std::forward<CallbackT>(callback));
            return create_subscription<MessageT, decltype(wrapped), AllocatorT, SubscriptionT, MessageMemoryStrategyT>(topic_name, qos, std::move(wrapped),
                                                                                                                       options, msg_mem_strat);
        }

        template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
        auto create_safe_wall_timer(const std::chrono::duration<DurationRepT, DurationT> &period, CallbackT &&callback,
                                    rclcpp::CallbackGroup::SharedPtr group = nullptr) ->
            typename rclcpp::WallTimer<decltype(safe_wrapper(std::forward<CallbackT>(callback)))>::SharedPtr {
            auto wrapped = safe_wrapper(std::forward<CallbackT>(callback));
            return create_wall_timer<DurationRepT, DurationT, decltype(wrapped)>(period, std::move(wrapped), group);
        }

        template <typename ServiceT, typename CallbackT>
        typename rclcpp::Service<ServiceT>::SharedPtr create_safe_service(const std::string &service_name, CallbackT &&callback,
                                                                          const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default,
                                                                          rclcpp::CallbackGroup::SharedPtr group = nullptr) {
            auto wrapped = safe_wrapper(std::forward<CallbackT>(callback));
            return create_service<ServiceT, decltype(wrapped)>(service_name, std::move(wrapped), qos_profile, group);
        }

        void terminate() {
            gate.terminate();
        }

        virtual ~Node() {
            terminate();
        }

    protected:
        Gate gate;
    };

}  // namespace crl::ros
