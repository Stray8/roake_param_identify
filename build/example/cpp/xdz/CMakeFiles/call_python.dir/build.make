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
include example/cpp/xdz/CMakeFiles/call_python.dir/depend.make

# Include the progress variables for this target.
include example/cpp/xdz/CMakeFiles/call_python.dir/progress.make

# Include the compile flags for this target's objects.
include example/cpp/xdz/CMakeFiles/call_python.dir/flags.make

example/cpp/xdz/CMakeFiles/call_python.dir/call_python.cpp.o: example/cpp/xdz/CMakeFiles/call_python.dir/flags.make
example/cpp/xdz/CMakeFiles/call_python.dir/call_python.cpp.o: ../example/cpp/xdz/call_python.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/cpp/xdz/CMakeFiles/call_python.dir/call_python.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/call_python.dir/call_python.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/xdz/call_python.cpp

example/cpp/xdz/CMakeFiles/call_python.dir/call_python.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/call_python.dir/call_python.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/xdz/call_python.cpp > CMakeFiles/call_python.dir/call_python.cpp.i

example/cpp/xdz/CMakeFiles/call_python.dir/call_python.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/call_python.dir/call_python.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/xdz/call_python.cpp -o CMakeFiles/call_python.dir/call_python.cpp.s

# Object files for target call_python
call_python_OBJECTS = \
"CMakeFiles/call_python.dir/call_python.cpp.o"

# External object files for target call_python
call_python_EXTERNAL_OBJECTS =

bin/call_python: example/cpp/xdz/CMakeFiles/call_python.dir/call_python.cpp.o
bin/call_python: example/cpp/xdz/CMakeFiles/call_python.dir/build.make
bin/call_python: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/call_python: /usr/lib/x86_64-linux-gnu/libpython3.8.so
bin/call_python: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/call_python: example/cpp/xdz/CMakeFiles/call_python.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/call_python"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/call_python.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/cpp/xdz/CMakeFiles/call_python.dir/build: bin/call_python

.PHONY : example/cpp/xdz/CMakeFiles/call_python.dir/build

example/cpp/xdz/CMakeFiles/call_python.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example/cpp/xdz && $(CMAKE_COMMAND) -P CMakeFiles/call_python.dir/cmake_clean.cmake
.PHONY : example/cpp/xdz/CMakeFiles/call_python.dir/clean

example/cpp/xdz/CMakeFiles/call_python.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example/cpp/xdz /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example/cpp/xdz /home/robot/robot/roake_param_identify/build/example/cpp/xdz/CMakeFiles/call_python.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/cpp/xdz/CMakeFiles/call_python.dir/depend

