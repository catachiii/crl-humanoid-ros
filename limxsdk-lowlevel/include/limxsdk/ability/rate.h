/**
 * @file rate.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef RATE_H
#define RATE_H
#include <chrono>
#include <thread>
#include "limxsdk/macros.h"

namespace limxsdk {
namespace ability {

/**
 * @class Rate
 * @brief A utility class for controlling a loop rate in real-time applications.
 * 
 * This class helps maintain a consistent execution frequency for a loop 
 * by calculating the appropriate sleep time based on the desired frequency.
 */
class LIMX_SDK_API Rate {
public:
  /**
   * @brief Constructs a Rate object with the specified frequency.
   * @param frequency The desired frequency in Hertz (Hz).
   */
  explicit Rate(double frequency) 
    : expected_cycle_time_(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / frequency)))
    , start_time_(Clock::now()) {}

  /**
   * @brief Pauses the current thread to maintain the specified loop rate.
   * 
   * This method calculates the time spent since the last call and sleeps
   * for the remaining time to meet the desired frequency. If the loop has
   * taken longer than the expected cycle time, it does not sleep and resets
   * the timing for the next cycle.
   */
  void sleep() {
    auto current_time = Clock::now();
    auto expected_end_time = start_time_ + expected_cycle_time_;

    // Calculate actual cycle time
    actual_cycle_time_ = current_time - start_time_;
    
    // Determine next start time
    start_time_ = expected_end_time;

    // Check if we are behind schedule
    if (current_time > expected_end_time) {
      // If significantly behind, reset start time to avoid excessive catch-up
      if (current_time > expected_end_time + expected_cycle_time_) {
        start_time_ = current_time;
      }
      return; // No sleep needed
    }

    // Calculate remaining time to sleep
    auto sleep_duration = expected_end_time - current_time;
    
    // Sleep for the remaining duration
    std::this_thread::sleep_for(sleep_duration);
  }

private:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  
  Duration expected_cycle_time_;  ///< Expected time per cycle
  Duration actual_cycle_time_;    ///< Actual time taken for the last cycle
  std::chrono::time_point<Clock> start_time_;  ///< Start time of the current cycle
};

} // namespace ability
} // namespace limxsdk
#endif // RATE_H
