#pragma once

#include <cassert>
#include <iostream>
#include <string_view>
#include <tuple>

namespace enum17::helper {
    constexpr bool is_ws(char c) {
        if (c == ' ' || c == '\n' || c == '\r' || c == '\f' || c == '\v' || c == '\t') {
            return true;
        } else {
            return false;
        }
    }

    constexpr std::size_t next_not_ws(std::string_view str_view, std::size_t cur_ind) {
        std::size_t i = cur_ind;
        for (; i < str_view.size(); i++) {
            if (!is_ws(str_view[i])) {
                return i;
            }
        }
        return i;
    }

    constexpr std::size_t prev_not_ws(std::string_view str_view, std::size_t cur_ind) {
        int i = cur_ind;
        for (; i >= 0; i--) {
            if (!is_ws(str_view[i])) {
                return i;
            }
        }
        return i;
    }

    constexpr auto pair2sv(std::string_view str_view, const std::pair<std::size_t, std::size_t> &inp) {
        return str_view.substr(inp.first, inp.second - inp.first);
    }

    template <std::size_t N>
    constexpr auto pairs2svs(std::string_view str_view, const std::array<std::pair<std::size_t, std::size_t>, N> &inps) {
        std::array<std::string_view, N> ret;
        for (std::size_t i = 0; i < N; i++) {
            ret[i] = str_view.substr(inps[i].first, inps[i].second - inps[i].first);
        }
        return ret;
    }

    template <std::size_t N>
    constexpr auto arr2sv(const std::array<char, N> &arr) {
        return std::string_view(arr.data(), N);
    }

    template <std::size_t... Ns, std::size_t... Is>
    constexpr auto arrs2svs_impl(const std::tuple<std::array<char, Ns>...> &arrs, const std::index_sequence<Is...> &) {
        using stripped_tup = std::remove_const_t<std::remove_reference_t<decltype(arrs)>>;
        return std::array<std::string_view, std::tuple_size_v<stripped_tup>>{arr2sv(std::get<Is>(arrs))...};
    }

    template <std::size_t... Ns>
    constexpr auto arrs2svs(const std::tuple<std::array<char, Ns>...> &arrs) {
        using stripped_tup = std::remove_const_t<std::remove_reference_t<decltype(arrs)>>;
        return arrs2svs_impl(arrs, std::make_index_sequence<std::tuple_size_v<stripped_tup>>{});
    }

    constexpr long long to_base_10(char inp) {
        if (inp >= '0' && inp <= '9') {
            return inp - '0';
        } else {
            return -1;
        }
    }

    constexpr long long to_base_8(char inp) {
        if (inp >= '0' && inp <= '7') {
            return inp - '0';
        } else {
            return -1;
        }
    }

    constexpr long long to_base_16(char inp) {
        if (inp >= '0' && inp <= '9') {
            return inp - '0';
        } else if (inp >= 'a' && inp <= 'f') {
            return inp - 'a' + 10;
        } else if (inp >= 'A' && inp <= 'F') {
            return inp - 'A' + 10;
        } else {
            return -1;
        }
    }

    constexpr long long str2int(std::string_view str_view) {
        if (str_view.size() == 0) {
            return 0;
        }

        bool is_neg = str_view[0] == '-';
        std::size_t start_loc = is_neg;
        std::size_t base = 10;

        if (str_view.size() >= 2 && str_view[is_neg] == '0' && str_view[is_neg + 1] == 'x') {
            start_loc += 2;
            base = 16;
        } else if (str_view[is_neg] == '0') {
            start_loc += 1;
            base = 8;
        }

        long long cur = 0;
        for (std::size_t i = start_loc; i < str_view.size(); i++) {
            long long this_val = 0;
            if (base == 10 && (this_val = to_base_10(str_view[i])) == -1) {
                break;
            } else if (base == 16 && (this_val = to_base_16(str_view[i])) == -1) {
                break;
            } else if (base == 8 && (this_val = to_base_8(str_view[i])) == -1) {
                break;
            }
            cur = cur * base + this_val;
        }

        return cur * (is_neg ? -1 : 1);
    }

}  // namespace enum17::helper

namespace enum17 {

    constexpr static std::size_t num_params(std::string_view str_view) {
        int counter = 0;
        for (std::size_t i = 0; i < str_view.size(); i++) {
            if (str_view[i] == ',') {
                counter++;
            }
        }
        return counter + 1;
    }

    template <std::size_t N>
    constexpr static auto param_loc(std::string_view str_view) {
        std::array<std::pair<std::size_t, std::size_t>, N> ret;
        ret[0].first = 0;
        if constexpr (N != 0) {
            ret[N - 1].second = str_view.size();
        }

        int cur_param = 0;
        for (std::size_t i = 0; i < str_view.size(); i++) {
            if (str_view[i] == ',') {
                ret[cur_param].second = i;
                ret[cur_param + 1].first = helper::next_not_ws(str_view, i + 1);
                cur_param++;
            }
        }

        return helper::pairs2svs(str_view, ret);
    }

    template <typename UT>
    struct param_info {
        constexpr static std::size_t find_equ(std::string_view str_view) {
            for (std::size_t i = 0; i < str_view.size(); i++) {
                if (str_view[i] == '=') {
                    return i;
                }
            }
            return str_view.size();
        }

        constexpr static std::pair<std::string_view, UT> get_data(std::string_view str_view, UT cur_value) {
            std::size_t equ_loc = find_equ(str_view);
            if (equ_loc == str_view.size()) {
                return {str_view, cur_value};
            } else {
                std::size_t prev_end = helper::prev_not_ws(str_view, equ_loc - 1) + 1;
                std::size_t next_start = helper::next_not_ws(str_view, equ_loc + 1);
                return {str_view.substr(0, prev_end), helper::str2int(str_view.substr(next_start, str_view.size()))};
            }
        }

        template <std::size_t N>
        constexpr static auto get_datas(const std::array<std::string_view, N> &str_views) {
            std::array<std::pair<std::string_view, UT>, N> ret;
            UT cur_value = 0;
            for (std::size_t i = 0; i < N; i++) {
                auto res = get_data(str_views[i], cur_value);
                ret[i].first = res.first;
                ret[i].second = res.second;
                cur_value = ret[i].second + 1;
            }
            return ret;
        }
    };

    template <typename inp_dataT>
    struct remove_duplicates {
        constexpr static bool is_first(std::size_t cur_ind) {
            typename inp_dataT::UT cur_val = inp_dataT::param_raw_infos[cur_ind].second;
            for (std::size_t i = 0; i < cur_ind; i++) {
                if (cur_val == inp_dataT::param_raw_infos[i].second) {
                    return false;
                }
            }
            return true;
        }

        constexpr static auto is_first_arr() {
            std::array<bool, inp_dataT::num_raw_params> ret{};
            for (std::size_t i = 0; i < inp_dataT::num_raw_params; i++) {
                ret[i] = is_first(i);
            }
            return ret;
        }

        constexpr static std::size_t is_first_num() {
            std::size_t is_firsts = 0;
            for (std::size_t i = 0; i < inp_dataT::num_raw_params; i++) {
                is_firsts += mask[i];
            }
            return is_firsts;
        }

        constexpr static auto param_info() {
            std::array<std::pair<std::string_view, typename inp_dataT::UT>, num> ret{};
            std::size_t cur_ind = 0;
            for (std::size_t i = 0; i < inp_dataT::num_raw_params; i++) {
                if (mask[i]) {
                    ret[cur_ind].first = inp_dataT::param_raw_infos[i].first;
                    ret[cur_ind].second = inp_dataT::param_raw_infos[i].second;
                    cur_ind++;
                }
            }
            return ret;
        }

        constexpr static auto mask = is_first_arr();
        constexpr static auto num = is_first_num();
    };

    template <typename inp_dataT>
    struct gen_strings {
        constexpr static std::string_view cur_sep{"_"};

        template <typename inp_dataT::UT val>
        constexpr static std::size_t calc_str_len() {
            std::size_t len = 0;
            std::size_t num = 0;
            for (std::size_t i = 0; i < inp_dataT::num_raw_params; i++) {
                if (val == inp_dataT::param_raw_infos[i].second) {
                    len += inp_dataT::param_raw_infos[i].first.size();
                    num++;
                }
            }

            return len + (num - 1) * cur_sep.size();
        }

        template <typename inp_dataT::UT val, std::size_t len = calc_str_len<val>()>
        constexpr static auto gen_str() {
            std::array<char, len> ret{};
            std::size_t cur_loc = 0;

            for (std::size_t i = 0; i < inp_dataT::num_raw_params; i++) {
                if (val == inp_dataT::param_raw_infos[i].second) {
                    for (std::size_t j = 0; j < inp_dataT::param_raw_infos[i].first.size(); j++) {
                        ret[cur_loc++] = inp_dataT::param_raw_infos[i].first[j];
                    }
                    if (cur_loc != len) {
                        for (std::size_t j = 0; j < cur_sep.size(); j++) {
                            ret[cur_loc++] = cur_sep[j];
                        }
                    }
                }
            }

            return ret;
        }

        template <std::size_t... Is>
        constexpr static auto gen_strs_impl(const std::index_sequence<Is...> &) {
            return std::make_tuple(gen_str<inp_dataT::param_infos[Is].second>()...);
        }

        constexpr static auto gen_strs() {
            return gen_strs_impl(std::make_index_sequence<inp_dataT::num_params>{});
        }
    };

}  // namespace enum17

#define ENUM17(enum_name, ut, ...)                                                                   \
    struct enum_name {                                                                               \
        using UT = ut;                                                                               \
        enum Inner : UT { __VA_ARGS__ };                                                             \
        UT cur_select;                                                                               \
        constexpr static std::string_view name{#enum_name};                                          \
        constexpr static std::string_view str{#__VA_ARGS__};                                         \
        constexpr static std::size_t num_raw_params = enum17::num_params(str);                       \
        constexpr static auto param_locs = enum17::param_loc<num_raw_params>(str);                   \
        constexpr static auto param_raw_infos = enum17::param_info<UT>::get_datas(param_locs);       \
        using remove_duplicates = enum17::remove_duplicates<enum_name>;                              \
        constexpr static auto num_params = remove_duplicates::num;                                   \
        constexpr static auto param_infos = remove_duplicates::param_info();                         \
        constexpr static auto param_joined_names_str = enum17::gen_strings<enum_name>::gen_strs();   \
        constexpr static auto param_joined_names = enum17::helper::arrs2svs(param_joined_names_str); \
        constexpr enum_name() noexcept : cur_select(0) {}                                                     \
        constexpr enum_name(Inner inp) noexcept : cur_select(calc_ind(inp)) {}                                \
        constexpr enum_name(UT inp) noexcept : enum_name(static_cast<Inner>(inp)) {}                          \
        constexpr std::size_t calc_ind(Inner inp) const {                                            \
            for (std::size_t i = 0; i < num_params; i++) {                                           \
                if (param_infos[i].second == inp) {                                                  \
                    return i;                                                                        \
                }                                                                                    \
            }                                                                                        \
            assert(false && "Corresponding value not found");                                        \
            return 0;                                                                                \
        }                                                                                            \
        constexpr static std::size_t size() {                                                        \
            return num_params;                                                                       \
        }                                                                                            \
        constexpr std::string_view to_string() const {                                               \
            return param_joined_names[cur_select];                                                   \
        }                                                                                            \
        constexpr static enum_name from_ind(std::size_t ind) {                                       \
            auto ret = enum_name();                                                                  \
            ret.cur_select = ind;                                                                    \
            assert(ind < num_params);                                                                \
            return ret;                                                                              \
        }                                                                                            \
        constexpr static enum_name from_raw_ind(std::size_t ind) {                                   \
            UT ut_val = param_raw_infos[ind].second;                                                 \
            return enum_name(ut_val);                                                                \
        }                                                                                            \
        constexpr static enum_name from_string(std::string_view str) {                               \
            for (std::size_t i = 0; i < num_raw_params; i++) {                                       \
                if (str == param_raw_infos[i].first) {                                               \
                    return enum_name::from_raw_ind(i);                                               \
                }                                                                                    \
            }                                                                                        \
            assert(false && "Corresponding string not found");                                       \
            return {};                                                                               \
        }                                                                                            \
        constexpr operator UT() {                                                                    \
            return to_ut();                                                                          \
        }                                                                                            \
        constexpr UT to_ut() const {                                                                 \
            return param_infos[cur_select].second;                                                   \
        }                                                                                            \
        friend inline std::ostream &operator<<(std::ostream &stream, const enum_name &value) {       \
            return stream << name << "::" << value.to_string();                                      \
        }                                                                                            \
    };
