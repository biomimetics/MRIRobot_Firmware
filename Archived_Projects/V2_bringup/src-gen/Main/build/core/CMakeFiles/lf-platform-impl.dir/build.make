# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build

# Include any dependencies generated for this target.
include core/CMakeFiles/lf-platform-impl.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include core/CMakeFiles/lf-platform-impl.dir/compiler_depend.make

# Include the progress variables for this target.
include core/CMakeFiles/lf-platform-impl.dir/progress.make

# Include the compile flags for this target's objects.
include core/CMakeFiles/lf-platform-impl.dir/flags.make

core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj: core/CMakeFiles/lf-platform-impl.dir/flags.make
core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj: /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/platform/impl/platform.c
core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj: core/CMakeFiles/lf-platform-impl.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj"
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core && /opt/homebrew/bin/arm-none-eabi-gcc -fdata-sections -ffunction-sections     --specs=nano.specs -Wl,--gc-sections $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj -MF CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj.d -o CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj -c /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/platform/impl/platform.c

core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.i"
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core && /opt/homebrew/bin/arm-none-eabi-gcc -fdata-sections -ffunction-sections     --specs=nano.specs -Wl,--gc-sections $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/platform/impl/platform.c > CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.i

core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.s"
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core && /opt/homebrew/bin/arm-none-eabi-gcc -fdata-sections -ffunction-sections     --specs=nano.specs -Wl,--gc-sections $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/platform/impl/platform.c -o CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.s

# Object files for target lf-platform-impl
lf__platform__impl_OBJECTS = \
"CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj"

# External object files for target lf-platform-impl
lf__platform__impl_EXTERNAL_OBJECTS =

core/liblf-platform-impl.a: core/CMakeFiles/lf-platform-impl.dir/__/platform/impl/platform.c.obj
core/liblf-platform-impl.a: core/CMakeFiles/lf-platform-impl.dir/build.make
core/liblf-platform-impl.a: core/CMakeFiles/lf-platform-impl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library liblf-platform-impl.a"
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core && $(CMAKE_COMMAND) -P CMakeFiles/lf-platform-impl.dir/cmake_clean_target.cmake
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lf-platform-impl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
core/CMakeFiles/lf-platform-impl.dir/build: core/liblf-platform-impl.a
.PHONY : core/CMakeFiles/lf-platform-impl.dir/build

core/CMakeFiles/lf-platform-impl.dir/clean:
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core && $(CMAKE_COMMAND) -P CMakeFiles/lf-platform-impl.dir/cmake_clean.cmake
.PHONY : core/CMakeFiles/lf-platform-impl.dir/clean

core/CMakeFiles/lf-platform-impl.dir/depend:
	cd /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/core /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core /Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/build/core/CMakeFiles/lf-platform-impl.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : core/CMakeFiles/lf-platform-impl.dir/depend

