# cmake/logger-lib.cmake
# Adds logger.c to the LF main target's build, matching the pattern used by
# pulse_controller_cmake.cmake. Needed because plain `files:` entries are
# only copied into src-gen, not automatically compiled -- Logger_Format*Line
# (logger.c) otherwise links as an undefined reference from Logger.lf.

target_sources(${LF_MAIN_TARGET} PRIVATE logger.c logger.h)
