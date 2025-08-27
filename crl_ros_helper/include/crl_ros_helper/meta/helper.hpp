#pragma once

#include <functional>
#include <tuple>

namespace crl::ros::meta {

    //    is_instance
    template <typename, template <typename...> typename>
    struct is_instance : public std::false_type {};

    template <template <typename...> class U, typename... T>
    struct is_instance<U<T...>, U> : public std::true_type {};

    //    get_masked
    template <std::size_t I, typename... Ts, typename... MaskTs>
    constexpr auto get_masked_helper(const std::tuple<Ts...> &content, const std::tuple<MaskTs...> &mask) {
        if constexpr (I == sizeof...(Ts)) {
            return std::make_tuple();
        } else if constexpr (std::is_same_v<std::tuple_element_t<I, std::tuple<MaskTs...>>, std::true_type>) {
            return std::tuple_cat(std::make_tuple(std::get<I>(content)), get_masked_helper<I + 1>(content, mask));
        } else {
            return get_masked_helper<I + 1>(content, mask);
        }
    }

    template <typename... Ts, typename... MaskTs>
    constexpr auto get_masked(const std::tuple<Ts...> &content, const std::tuple<MaskTs...> &mask) {
        return get_masked_helper<0>(content, mask);
    }

    //    op_tuple
    template <typename F, typename... T1s, typename... T2s, std::size_t... I>
    constexpr auto op_tuple_impl(F f, const std::tuple<T1s...> &t1, const std::tuple<T2s...> &t2, const std::index_sequence<I...> &) {
        return std::tuple{f(std::get<I>(t1), std::get<I>(t2))...};
    }

    template <typename F, typename... T1s, typename... T2s>
    constexpr auto op_tuple(F f, const std::tuple<T1s...> &t1, const std::tuple<T2s...> &t2) {
        static_assert(sizeof...(T1s) == sizeof...(T2s), "Tuples not same size");

        return op_tuple_impl(f, t1, t2, std::make_index_sequence<sizeof...(T1s)>());
    }

    //    are_all_tf
    template <typename... Ts>
    struct are_all_tf {
        static constexpr bool value = (true && ... && (std::is_same_v<std::true_type, Ts> || std::is_same_v<std::false_type, Ts>));
    };

    template <typename... Ts>
    inline constexpr bool are_all_tf_v = are_all_tf<Ts...>::value;

    //    tuple tf type | & !
    template <typename... T1s, typename... T2s, typename = typename std::enable_if_t<are_all_tf_v<T1s...> && are_all_tf_v<T2s...>, void>>
    inline constexpr auto operator|(const std::tuple<T1s...> &, const std::tuple<T2s...> &) {
        return std::make_tuple(std::conditional_t < std::is_same_v<T1s, std::true_type> || std::is_same_v<T2s, std::true_type>, std::true_type,
                               std::false_type > ()...);
    }

    template <typename... T1s, typename... T2s, typename = typename std::enable_if_t<are_all_tf_v<T1s...> && are_all_tf_v<T2s...>, void>>
    inline constexpr auto operator&(const std::tuple<T1s...> &, const std::tuple<T2s...> &) {
        return std::make_tuple(
            std::conditional_t<!(std::is_same_v<T1s, std::false_type> || std::is_same_v<T2s, std::false_type>), std::true_type, std::false_type>()...);
    }

    template <typename... Ts, typename = typename std::enable_if_t<are_all_tf_v<Ts...>, void>>
    inline constexpr auto operator!(const std::tuple<Ts...> &) {
        return std::make_tuple(std::conditional_t<std::is_same_v<Ts, std::true_type>, std::false_type, std::true_type>()...);
    }

    //    tuple_rt_get
    template <class F, class T, std::size_t N>
    void tuple_rt_get_helper(F f, T &t, size_t ind) {
        if (N == ind) {
            std::invoke(f, std::get<N>(t));
            return;
        }

        if constexpr (N + 1 < std::tuple_size_v<T>) {
            return tuple_rt_get_helper<F, T, N + 1>(f, t, ind);
        }
    }

    template <class F, class T>
    void tuple_rt_get(F f, T &t, size_t ind) {
        return tuple_rt_get_helper<F, T, 0>(f, t, ind);
    }

    //    always_false
    template <typename T>
    struct always_false : std::false_type {};

    template <typename T>
    inline constexpr bool always_false_v = always_false<T>::value;

}  // namespace crl::ros::meta
