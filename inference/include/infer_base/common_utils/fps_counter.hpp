#pragma once

#include <chrono>
#include "infer_base/common_utils/log.hpp"

namespace easy_deploy {

class FPSCounter {
public:
  // Constructor: initialize accumulator and start state
  FPSCounter() : sum(0), is_running(false)
  {}

  // Start timing
  void Start()
  {
    start_time = std::chrono::high_resolution_clock::now();
    sum        = 0;
    is_running = true;
  }

  // Increase frame count
  void Count(int i)
  {
    if (!is_running)
    {
      LOG_ERROR("Please call Start() before counting.");
      return;
    }
    sum += i;
  }

  // Get FPS
  double GetFPS()
  {
    if (!is_running)
    {
      LOG_ERROR("Please call Start() before calculating FPS.");
      return 0.0;
    }

    auto                          current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration     = current_time - start_time;
    double                        duration_seconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    if (duration_seconds == 0)
    {
      return 0.0; // Avoid division by zero
    }

    return sum / duration_seconds * 1000;
  }

private:
  int  sum;        // Accumulated frame count
  bool is_running; // Whether timing is running

  std::chrono::high_resolution_clock::time_point start_time; // Start time
};

} // namespace easy_deploy
