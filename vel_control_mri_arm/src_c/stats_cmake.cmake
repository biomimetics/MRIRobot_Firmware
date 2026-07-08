# cmake/stats-lib.cmake
# Adds stats.c to the LF main target's build, matching the pattern used by
# pulse_motor_model_cmake.cmake/gaussian_motion_cmake.cmake. Kept as its own
# cmake-include so anything needing plain running-statistics/Gaussian-RV
# math can pull it in without also pulling in motion-specific code.

target_sources(${LF_MAIN_TARGET} PRIVATE stats.c stats.h)
