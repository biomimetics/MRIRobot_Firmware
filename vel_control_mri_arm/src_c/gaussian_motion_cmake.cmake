# cmake/gaussian_motion-lib.cmake
# Adds gaussian_motion.c to the LF main target's build, matching the pattern
# used by pulse_motor_model_cmake.cmake. Kept as its own cmake-include so any
# controller that needs Gaussian uncertainty propagation can pull it in
# without also pulling in a specific controller implementation.

target_sources(${LF_MAIN_TARGET} PRIVATE gaussian_motion.c gaussian_motion.h)
