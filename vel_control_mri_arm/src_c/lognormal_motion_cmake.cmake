# cmake/lognormal_motion-lib.cmake
# Adds lognormal_motion.c to the LF main target's build, matching the
# pattern used by gaussian_motion_cmake.cmake. Experimental alternative
# arrival-time model (see lognormal_motion.h) -- kept as its own
# cmake-include so it can be pulled in for comparison without disturbing
# anything that already includes gaussian_motion_cmake.cmake.

target_sources(${LF_MAIN_TARGET} PRIVATE lognormal_motion.c lognormal_motion.h)

# lognormal_motion.c calls into stats.c's LogNormalRV_* functions, so
# stats.c has to be linked in wherever lognormal_motion.c is -- same
# duplication reasoning as gaussian_motion_cmake.cmake.
target_sources(${LF_MAIN_TARGET} PRIVATE stats.c stats.h)
