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
include example/cpp/ly/CMakeFiles/wyy.dir/depend.make

# Include the progress variables for this target.
include example/cpp/ly/CMakeFiles/wyy.dir/progress.make

# Include the compile flags for this target's objects.
include example/cpp/ly/CMakeFiles/wyy.dir/flags.make

example/cpp/ly/CMakeFiles/wyy.dir/wyy.cpp.o: example/cpp/ly/CMakeFiles/wyy.dir/flags.make
example/cpp/ly/CMakeFiles/wyy.dir/wyy.cpp.o: ../example/cpp/ly/wyy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/cpp/ly/CMakeFiles/wyy.dir/wyy.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wyy.dir/wyy.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/ly/wyy.cpp

example/cpp/ly/CMakeFiles/wyy.dir/wyy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wyy.dir/wyy.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/ly/wyy.cpp > CMakeFiles/wyy.dir/wyy.cpp.i

example/cpp/ly/CMakeFiles/wyy.dir/wyy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wyy.dir/wyy.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/ly/wyy.cpp -o CMakeFiles/wyy.dir/wyy.cpp.s

# Object files for target wyy
wyy_OBJECTS = \
"CMakeFiles/wyy.dir/wyy.cpp.o"

# External object files for target wyy
wyy_EXTERNAL_OBJECTS =

bin/wyy: example/cpp/ly/CMakeFiles/wyy.dir/wyy.cpp.o
bin/wyy: example/cpp/ly/CMakeFiles/wyy.dir/build.make
bin/wyy: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/wyy: /usr/lib/x86_64-linux-gnu/libpython3.8.so
bin/wyy: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/wyy: example/cpp/ly/CMakeFiles/wyy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/wyy"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/wyy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/cpp/ly/CMakeFiles/wyy.dir/build: bin/wyy

.PHONY : example/cpp/ly/CMakeFiles/wyy.dir/build

example/cpp/ly/CMakeFiles/wyy.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example/cpp/ly && $(CMAKE_COMMAND) -P CMakeFiles/wyy.dir/cmake_clean.cmake
.PHONY : example/cpp/ly/CMakeFiles/wyy.dir/clean

example/cpp/ly/CMakeFiles/wyy.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example/cpp/ly /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example/cpp/ly /home/robot/robot/roake_param_identify/build/example/cpp/ly/CMakeFiles/wyy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/cpp/ly/CMakeFiles/wyy.dir/depend

