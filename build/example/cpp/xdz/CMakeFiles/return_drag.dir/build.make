# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robot/robot/roake_param_identify

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/robot/roake_param_identify/build

# Include any dependencies generated for this target.
include example/cpp/xdz/CMakeFiles/return_drag.dir/depend.make

# Include the progress variables for this target.
include example/cpp/xdz/CMakeFiles/return_drag.dir/progress.make

# Include the compile flags for this target's objects.
include example/cpp/xdz/CMakeFiles/return_drag.dir/flags.make

example/cpp/xdz/CMakeFiles/return_drag.dir/return_drag.cpp.o: example/cpp/xdz/CMakeFiles/return_drag.dir/flags.make
example/cpp/xdz/CMakeFiles/return_drag.dir/return_drag.cpp.o: ../example/cpp/xdz/return_drag.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/cpp/xdz/CMakeFiles/return_drag.dir/return_drag.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/return_drag.dir/return_drag.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/xdz/return_drag.cpp

example/cpp/xdz/CMakeFiles/return_drag.dir/return_drag.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/return_drag.dir/return_drag.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/xdz/return_drag.cpp > CMakeFiles/return_drag.dir/return_drag.cpp.i

example/cpp/xdz/CMakeFiles/return_drag.dir/return_drag.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/return_drag.dir/return_drag.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/xdz/return_drag.cpp -o CMakeFiles/return_drag.dir/return_drag.cpp.s

# Object files for target return_drag
return_drag_OBJECTS = \
"CMakeFiles/return_drag.dir/return_drag.cpp.o"

# External object files for target return_drag
return_drag_EXTERNAL_OBJECTS =

bin/return_drag: example/cpp/xdz/CMakeFiles/return_drag.dir/return_drag.cpp.o
bin/return_drag: example/cpp/xdz/CMakeFiles/return_drag.dir/build.make
bin/return_drag: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/return_drag: /usr/lib/x86_64-linux-gnu/libpython3.8.so
bin/return_drag: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/return_drag: example/cpp/xdz/CMakeFiles/return_drag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/return_drag"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/return_drag.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/cpp/xdz/CMakeFiles/return_drag.dir/build: bin/return_drag

.PHONY : example/cpp/xdz/CMakeFiles/return_drag.dir/build

example/cpp/xdz/CMakeFiles/return_drag.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && $(CMAKE_COMMAND) -P CMakeFiles/return_drag.dir/cmake_clean.cmake
.PHONY : example/cpp/xdz/CMakeFiles/return_drag.dir/clean

example/cpp/xdz/CMakeFiles/return_drag.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example/cpp/xdz /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example/cpp/xdz /home/robot/robot/roake_param_identify/build/example/cpp/xdz/CMakeFiles/return_drag.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/cpp/xdz/CMakeFiles/return_drag.dir/depend

