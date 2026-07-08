# cmake/encoder_state_observer-lib.cmake
# Adds encoder_state_observer.c to the LF main target's build, matching the
# pattern used by pulse_motor_model_cmake.cmake et al.

target_sources(${LF_MAIN_TARGET} PRIVATE encoder_state_observer.c encoder_state_observer.h)

# encoder_state_observer.h includes pulse_motor_model.h (PulseMotorModel/
# MotorState), so pulse_motor_model.c -- and ITS transitive dependencies,
# gaussian_motion.c/stats.c -- have to be linked in wherever this is.
# Duplicated here rather than relying on every consumer to list the other
# cmake-includes themselves; target_sources on an already-added source is a
# harmless no-op (same reasoning as pulse_motor_model_cmake.cmake).
target_sources(${LF_MAIN_TARGET} PRIVATE pulse_motor_model.c pulse_motor_model.h)
target_sources(${LF_MAIN_TARGET} PRIVATE gaussian_motion.c gaussian_motion.h)
target_sources(${LF_MAIN_TARGET} PRIVATE stats.c stats.h)
