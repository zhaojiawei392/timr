# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kai/Downloads/timr

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kai/Downloads/timr/bu

# Include any dependencies generated for this target.
include CMakeFiles/homing.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/homing.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/homing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/homing.dir/flags.make

CMakeFiles/homing.dir/src/homing.cpp.o: CMakeFiles/homing.dir/flags.make
CMakeFiles/homing.dir/src/homing.cpp.o: /home/kai/Downloads/timr/src/homing.cpp
CMakeFiles/homing.dir/src/homing.cpp.o: CMakeFiles/homing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/kai/Downloads/timr/bu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/homing.dir/src/homing.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/homing.dir/src/homing.cpp.o -MF CMakeFiles/homing.dir/src/homing.cpp.o.d -o CMakeFiles/homing.dir/src/homing.cpp.o -c /home/kai/Downloads/timr/src/homing.cpp

CMakeFiles/homing.dir/src/homing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/homing.dir/src/homing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kai/Downloads/timr/src/homing.cpp > CMakeFiles/homing.dir/src/homing.cpp.i

CMakeFiles/homing.dir/src/homing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/homing.dir/src/homing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kai/Downloads/timr/src/homing.cpp -o CMakeFiles/homing.dir/src/homing.cpp.s

# Object files for target homing
homing_OBJECTS = \
"CMakeFiles/homing.dir/src/homing.cpp.o"

# External object files for target homing
homing_EXTERNAL_OBJECTS =

homing: CMakeFiles/homing.dir/src/homing.cpp.o
homing: CMakeFiles/homing.dir/build.make
homing: external/qpOASES/libqpOASES.a
homing: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.83.0
homing: /home/kai/Downloads/timr/external/coppeliasim/share/libremoteApiCoppeliasim.so
homing: CMakeFiles/homing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/kai/Downloads/timr/bu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable homing"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/homing.dir/build: homing
.PHONY : CMakeFiles/homing.dir/build

CMakeFiles/homing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/homing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/homing.dir/clean

CMakeFiles/homing.dir/depend:
	cd /home/kai/Downloads/timr/bu && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kai/Downloads/timr /home/kai/Downloads/timr /home/kai/Downloads/timr/bu /home/kai/Downloads/timr/bu /home/kai/Downloads/timr/bu/CMakeFiles/homing.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/homing.dir/depend

