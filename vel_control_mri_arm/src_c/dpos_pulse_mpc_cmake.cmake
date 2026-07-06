# cmake/dpos_pulse_mpc-lib.cmake
# Adds dpos_pulse_mpc.c to the LF main target's build, matching the pattern
# used by pulse_mpc_cmake.cmake.

target_sources(${LF_MAIN_TARGET} PRIVATE dpos_pulse_mpc.c dpos_pulse_mpc.h)
