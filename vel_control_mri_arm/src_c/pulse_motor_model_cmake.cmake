# cmake/pulse_motor_model-lib.cmake
# Adds pulse_motor_model.c to the LF main target's build, matching the
# pattern used by pulse_mpc_cmake.cmake/stm_comms_cmake.cmake. Kept as its
# own cmake-include (rather than folded into pulse_mpc_cmake.cmake or
# dpos_pulse_mpc_cmake.cmake) so either controller -- or a future
# alternative implementation -- can pull in the shared motor model without
# also pulling in unrelated controller-specific code.

target_sources(${LF_MAIN_TARGET} PRIVATE pulse_motor_model.c pulse_motor_model.h)

# pulse_motor_model.c now calls into GaussianMotion_ArrivalTimeInvCdf
# (PulseMotorModel_PlanPulseWithOvershootBound), so gaussian_motion.c has to
# be linked in wherever pulse_motor_model.c is -- duplicated here rather than
# relying on every consumer to separately list gaussian_motion_cmake.cmake
# themselves (target_sources on an already-added source is a harmless no-op,
# so this is safe even for reactors that also list it explicitly). That in
# turn depends on stats.c (see gaussian_motion_cmake.cmake), so it's
# duplicated one level further here too.
target_sources(${LF_MAIN_TARGET} PRIVATE gaussian_motion.c gaussian_motion.h)
target_sources(${LF_MAIN_TARGET} PRIVATE stats.c stats.h)
