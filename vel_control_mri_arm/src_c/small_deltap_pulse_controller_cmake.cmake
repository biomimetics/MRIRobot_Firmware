# cmake/small_deltap_pulse_controller-lib.cmake
# Adds small_deltap_pulse_controller.c to the LF main target's build,
# matching the pattern used by pulse_motor_model_cmake.cmake/
# gaussian_motion_cmake.cmake. Needed because plain `files:` entries are only
# copied into src-gen, not automatically compiled -- SmallDeltaPPulse_
# PrintDebugInfo (small_deltap_pulse_controller.c) otherwise links as an
# undefined reference from Small_DeltaP_Pulse_Controller_Bank.lf.

target_sources(${LF_MAIN_TARGET} PRIVATE small_deltap_pulse_controller.c small_deltap_pulse_controller.h)
