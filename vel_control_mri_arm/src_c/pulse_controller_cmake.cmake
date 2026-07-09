# cmake/pulse_controller-lib.cmake
# Adds pulse_controller.c to the LF main target's build,
# matching the pattern used by pulse_motor_model_cmake.cmake/
# gaussian_motion_cmake.cmake. Needed because plain `files:` entries are only
# copied into src-gen, not automatically compiled -- PulseController_
# PrintDebugInfo (pulse_controller.c) otherwise links as an
# undefined reference from PulseControllerBank.lf.

target_sources(${LF_MAIN_TARGET} PRIVATE pulse_controller.c pulse_controller.h)
