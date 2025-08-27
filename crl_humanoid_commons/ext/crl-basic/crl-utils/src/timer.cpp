//
// Created by Dongho Kang on 08.07.23.
//

#include <crl-basic/utils/timer.h>

namespace crl::utils {

Timer::Timer() {
    restart();
}

Timer::~Timer() {}

void Timer::restart() {
    begin = std::chrono::high_resolution_clock::now();
}

double Timer::timeEllapsed() {
    now = std::chrono::high_resolution_clock::now();
    duration = now - begin;
    return duration.count();
}

}  // namespace crl::utils