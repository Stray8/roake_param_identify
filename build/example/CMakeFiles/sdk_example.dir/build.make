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
CMAKE_SOURCE_DIR = /home/robot/robot/librokae-v0.3.3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/robot/librokae-v0.3.3/build

# Include any dependencies generated for this target.
include example/CMakeFiles/sdk_example.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/sdk_example.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/sdk_example.dir/flags.make

example/CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.o: example/CMakeFiles/sdk_example.dir/flags.make
example/CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.o: ../example/cpp/sdk_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/librokae-v0.3.3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.o"
	cd /home/robot/robot/librokae-v0.3.3/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.o -c /home/robot/robot/librokae-v0.3.3/example/cpp/sdk_example.cpp

example/CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.i"
	cd /home/robot/robot/librokae-v0.3.3/build/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/librokae-v0.3.3/example/cpp/sdk_example.cpp > CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.i

example/CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.s"
	cd /home/robot/robot/librokae-v0.3.3/build/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/librokae-v0.3.3/example/cpp/sdk_example.cpp -o CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.s

# Object files for target sdk_example
sdk_example_OBJECTS = \
"CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.o"

# External object files for target sdk_example
sdk_example_EXTERNAL_OBJECTS =

bin/sdk_example: example/CMakeFiles/sdk_example.dir/cpp/sdk_example.cpp.o
bin/sdk_example: example/CMakeFiles/sdk_example.dir/build.make
bin/sdk_example: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/sdk_example: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/sdk_example: example/CMakeFiles/sdk_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/librokae-v0.3.3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/sdk_example"
	cd /home/robot/robot/librokae-v0.3.3/build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdk_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/sdk_example.dir/build: bin/sdk_example

.PHONY : example/CMakeFiles/sdk_example.dir/build

example/CMakeFiles/sdk_example.dir/clean:
	cd /home/robot/robot/librokae-v0.3.3/build/example && $(CMAKE_COMMAND) -P CMakeFiles/sdk_example.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/sdk_example.dir/clean

example/CMakeFiles/sdk_example.dir/depend:
	cd /home/robot/robot/librokae-v0.3.3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/librokae-v0.3.3 /home/robot/robot/librokae-v0.3.3/example /home/robot/robot/librokae-v0.3.3/build /home/robot/robot/librokae-v0.3.3/build/example /home/robot/robot/librokae-v0.3.3/build/example/CMakeFiles/sdk_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/sdk_example.dir/depend

