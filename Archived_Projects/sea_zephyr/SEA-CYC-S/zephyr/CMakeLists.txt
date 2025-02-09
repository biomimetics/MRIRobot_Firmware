cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE "nucleo_f446re.overlay")

# next line saves intermediate files for debugging
set(CMAKE_C_FLAGS "-save-temps=obj")
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(ZephyrSampleCode)

FILE(GLOB app_sources ../src/*.c*)
target_sources(app PRIVATE ${app_sources})

