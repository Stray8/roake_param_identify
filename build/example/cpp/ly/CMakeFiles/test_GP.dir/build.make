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
include example/cpp/ly/CMakeFiles/test_GP.dir/depend.make

# Include the progress variables for this target.
include example/cpp/ly/CMakeFiles/test_GP.dir/progress.make

# Include the compile flags for this target's objects.
include example/cpp/ly/CMakeFiles/test_GP.dir/flags.make

example/cpp/ly/CMakeFiles/test_GP.dir/test_GP.cpp.o: example/cpp/ly/CMakeFiles/test_GP.dir/flags.make
example/cpp/ly/CMakeFiles/test_GP.dir/test_GP.cpp.o: ../example/cpp/ly/test_GP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/cpp/ly/CMakeFiles/test_GP.dir/test_GP.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_GP.dir/test_GP.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/ly/test_GP.cpp

example/cpp/ly/CMakeFiles/test_GP.dir/test_GP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_GP.dir/test_GP.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/ly/test_GP.cpp > CMakeFiles/test_GP.dir/test_GP.cpp.i

example/cpp/ly/CMakeFiles/test_GP.dir/test_GP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_GP.dir/test_GP.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/ly/test_GP.cpp -o CMakeFiles/test_GP.dir/test_GP.cpp.s

# Object files for target test_GP
test_GP_OBJECTS = \
"CMakeFiles/test_GP.dir/test_GP.cpp.o"

# External object files for target test_GP
test_GP_EXTERNAL_OBJECTS =

bin/test_GP: example/cpp/ly/CMakeFiles/test_GP.dir/test_GP.cpp.o
bin/test_GP: example/cpp/ly/CMakeFiles/test_GP.dir/build.make
bin/test_GP: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/test_GP: /usr/lib/x86_64-linux-gnu/libpython3.8.so
bin/test_GP: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/test_GP: example/cpp/ly/CMakeFiles/test_GP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/test_GP"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_GP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/cpp/ly/CMakeFiles/test_GP.dir/build: bin/test_GP

.PHONY : example/cpp/ly/CMakeFiles/test_GP.dir/build

example/cpp/ly/CMakeFiles/test_GP.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && $(CMAKE_COMMAND) -P CMakeFiles/test_GP.dir/cmake_clean.cmake
.PHONY : example/cpp/ly/CMakeFiles/test_GP.dir/clean

example/cpp/ly/CMakeFiles/test_GP.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example/cpp/ly /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example/cpp/ly /home/robot/robot/roake_param_identify/build/example/cpp/ly/CMakeFiles/test_GP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/cpp/ly/CMakeFiles/test_GP.dir/depend

