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
include example/cpp/xdz/CMakeFiles/cart_control.dir/depend.make

# Include the progress variables for this target.
include example/cpp/xdz/CMakeFiles/cart_control.dir/progress.make

# Include the compile flags for this target's objects.
include example/cpp/xdz/CMakeFiles/cart_control.dir/flags.make

example/cpp/xdz/CMakeFiles/cart_control.dir/cart_control.cpp.o: example/cpp/xdz/CMakeFiles/cart_control.dir/flags.make
example/cpp/xdz/CMakeFiles/cart_control.dir/cart_control.cpp.o: ../example/cpp/xdz/cart_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/cpp/xdz/CMakeFiles/cart_control.dir/cart_control.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cart_control.dir/cart_control.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/xdz/cart_control.cpp

example/cpp/xdz/CMakeFiles/cart_control.dir/cart_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cart_control.dir/cart_control.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/xdz/cart_control.cpp > CMakeFiles/cart_control.dir/cart_control.cpp.i

example/cpp/xdz/CMakeFiles/cart_control.dir/cart_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cart_control.dir/cart_control.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/xdz/cart_control.cpp -o CMakeFiles/cart_control.dir/cart_control.cpp.s

# Object files for target cart_control
cart_control_OBJECTS = \
"CMakeFiles/cart_control.dir/cart_control.cpp.o"

# External object files for target cart_control
cart_control_EXTERNAL_OBJECTS =

bin/cart_control: example/cpp/xdz/CMakeFiles/cart_control.dir/cart_control.cpp.o
bin/cart_control: example/cpp/xdz/CMakeFiles/cart_control.dir/build.make
bin/cart_control: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/cart_control: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/cart_control: example/cpp/xdz/CMakeFiles/cart_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/cart_control"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cart_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/cpp/xdz/CMakeFiles/cart_control.dir/build: bin/cart_control

.PHONY : example/cpp/xdz/CMakeFiles/cart_control.dir/build

example/cpp/xdz/CMakeFiles/cart_control.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && $(CMAKE_COMMAND) -P CMakeFiles/cart_control.dir/cmake_clean.cmake
.PHONY : example/cpp/xdz/CMakeFiles/cart_control.dir/clean

example/cpp/xdz/CMakeFiles/cart_control.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example/cpp/xdz /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example/cpp/xdz /home/robot/robot/roake_param_identify/build/example/cpp/xdz/CMakeFiles/cart_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/cpp/xdz/CMakeFiles/cart_control.dir/depend

