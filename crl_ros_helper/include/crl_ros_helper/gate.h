#pragma once

#include <atomic>

namespace crl::ros {

    class Gate {
        // cannot have more than this value of simultaneous clients
        constexpr static uint64_t min_val = 0x8000000000000000;

    public:
        Gate() : counter(0) {}

        void terminate() {
            counter |= min_val;

            while (counter.load() != min_val) {
            }
        }

        bool start() {
            uint64_t cur_val = counter.load();
            while (true) {
                if (cur_val >= min_val) {
                    return false;
                }
                if (counter.compare_exchange_weak(cur_val, cur_val + 1)) {
                    return true;
                }
            }
        }

        void stop() {
            counter--;
        }

    private:
        std::atomic<uint64_t> counter;
    };

    class GateWrapper {
    public:
        GateWrapper(Gate& counter) : counter(counter) {
            succ = counter.start();
        }

        ~GateWrapper() {
            if (succ) {
                counter.stop();
            }
        }

        bool is_succ() {
            return succ;
        }

    private:
        Gate& counter;
        bool succ;
    };

}  // namespace crl::ros
