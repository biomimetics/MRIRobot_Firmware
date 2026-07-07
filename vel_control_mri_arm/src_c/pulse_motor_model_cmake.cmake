# cmake/pulse_motor_model-lib.cmake
# Adds pulse_motor_model.c to the LF main target's build, matching the
# pattern used by pulse_mpc_cmake.cmake/stm_comms_cmake.cmake. Kept as its
# own cmake-include (rather than folded into pulse_mpc_cmake.cmake or
# dpos_pulse_mpc_cmake.cmake) so either controller -- or a future
# alternative implementation -- can pull in the shared motor model without
# also pulling in unrelated controller-specific code.

target_sources(${LF_MAIN_TARGET} PRIVATE pulse_motor_model.c pulse_motor_model.h)
