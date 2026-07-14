# cmake/encoder_state_observer-lib.cmake
# Adds encoder_state_observer.c to the LF main target's build, matching the
# pattern used elsewhere.

target_sources(${LF_MAIN_TARGET} PRIVATE encoder_state_observer.c encoder_state_observer.h)
