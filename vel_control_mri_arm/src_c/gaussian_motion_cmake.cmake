# cmake/gaussian_motion-lib.cmake
# Adds gaussian_motion.c to the LF main target's build, matching the pattern
# used by pulse_motor_model_cmake.cmake. Kept as its own cmake-include so any
# controller that needs Gaussian uncertainty propagation can pull it in
# without also pulling in a specific controller implementation.

target_sources(${LF_MAIN_TARGET} PRIVATE gaussian_motion.c gaussian_motion.h)

# gaussian_motion.c calls into stats.c's GaussianRV_* basic-statistics
# functions now (see gaussian_motion.h's split-out comment), so stats.c has
# to be linked in wherever gaussian_motion.c is -- duplicated here rather
# than relying on every consumer to separately list stats_cmake.cmake
# themselves (target_sources on an already-added source is a harmless
# no-op, so this is safe even for reactors that also list it explicitly).
target_sources(${LF_MAIN_TARGET} PRIVATE stats.c stats.h)
