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
include example/cpp/rt/CMakeFiles/test_ly.dir/depend.make

# Include the progress variables for this target.
include example/cpp/rt/CMakeFiles/test_ly.dir/progress.make

# Include the compile flags for this target's objects.
include example/cpp/rt/CMakeFiles/test_ly.dir/flags.make

example/cpp/rt/CMakeFiles/test_ly.dir/test_ly.cpp.o: example/cpp/rt/CMakeFiles/test_ly.dir/flags.make
example/cpp/rt/CMakeFiles/test_ly.dir/test_ly.cpp.o: ../example/cpp/rt/test_ly.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/cpp/rt/CMakeFiles/test_ly.dir/test_ly.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/rt && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_ly.dir/test_ly.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/rt/test_ly.cpp

example/cpp/rt/CMakeFiles/test_ly.dir/test_ly.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ly.dir/test_ly.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/rt && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/rt/test_ly.cpp > CMakeFiles/test_ly.dir/test_ly.cpp.i

example/cpp/rt/CMakeFiles/test_ly.dir/test_ly.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ly.dir/test_ly.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/rt && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/rt/test_ly.cpp -o CMakeFiles/test_ly.dir/test_ly.cpp.s

# Object files for target test_ly
test_ly_OBJECTS = \
"CMakeFiles/test_ly.dir/test_ly.cpp.o"

# External object files for target test_ly
test_ly_EXTERNAL_OBJECTS =

bin/test_ly: example/cpp/rt/CMakeFiles/test_ly.dir/test_ly.cpp.o
bin/test_ly: example/cpp/rt/CMakeFiles/test_ly.dir/build.make
bin/test_ly: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/test_ly: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/test_ly: example/cpp/rt/CMakeFiles/test_ly.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/test_ly"
	cd /home/robot/robot/roake_param_identify/build/example/cpp/rt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_ly.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/cpp/rt/CMakeFiles/test_ly.dir/build: bin/test_ly

.PHONY : example/cpp/rt/CMakeFiles/test_ly.dir/build

example/cpp/rt/CMakeFiles/test_ly.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example/cpp/rt && $(CMAKE_COMMAND) -P CMakeFiles/test_ly.dir/cmake_clean.cmake
.PHONY : example/cpp/rt/CMakeFiles/test_ly.dir/clean

example/cpp/rt/CMakeFiles/test_ly.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example/cpp/rt /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example/cpp/rt /home/robot/robot/roake_param_identify/build/example/cpp/rt/CMakeFiles/test_ly.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/cpp/rt/CMakeFiles/test_ly.dir/depend

