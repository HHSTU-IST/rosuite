#include <cstdlib>
#include <iostream>

#include "kracker/apps/apps.hpp"

/// Runs the example executable.
int main()
{
  const auto summary =
      kracker::apps::offline::run_single_target_kalman_example(42U);
  if (!summary.ok())
  {
    std::cerr << "Offline example failed: "
              << summary.status().message << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Frames: " << summary.value().metrics.truth_frames << '\n';
  std::cout << "Position RMSE: "
            << summary.value().metrics.position_rmse << '\n';
  std::cout << "Velocity RMSE: "
            << summary.value().metrics.velocity_rmse << '\n';
  std::cout << "Final track count: "
            << summary.value().tracker_frames.back().tracks.size() << '\n';
  return EXIT_SUCCESS;
}
