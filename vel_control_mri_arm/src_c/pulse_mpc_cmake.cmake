# cmake/pulse_mpc-lib.cmake
# Adds pulse_mpc.c to the LF main target's build, matching the pattern used
# by stm_comms_cmake.cmake for stm_comms.c/.h.

target_sources(${LF_MAIN_TARGET} PRIVATE pulse_mpc.c pulse_mpc.h)
