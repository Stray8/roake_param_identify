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
include example/CMakeFiles/path_record.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/path_record.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/path_record.dir/flags.make

example/CMakeFiles/path_record.dir/cpp/path_record.cpp.o: example/CMakeFiles/path_record.dir/flags.make
example/CMakeFiles/path_record.dir/cpp/path_record.cpp.o: ../example/cpp/path_record.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/CMakeFiles/path_record.dir/cpp/path_record.cpp.o"
	cd /home/robot/robot/roake_param_identify/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/path_record.dir/cpp/path_record.cpp.o -c /home/robot/robot/roake_param_identify/example/cpp/path_record.cpp

example/CMakeFiles/path_record.dir/cpp/path_record.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/path_record.dir/cpp/path_record.cpp.i"
	cd /home/robot/robot/roake_param_identify/build/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/robot/roake_param_identify/example/cpp/path_record.cpp > CMakeFiles/path_record.dir/cpp/path_record.cpp.i

example/CMakeFiles/path_record.dir/cpp/path_record.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/path_record.dir/cpp/path_record.cpp.s"
	cd /home/robot/robot/roake_param_identify/build/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/robot/roake_param_identify/example/cpp/path_record.cpp -o CMakeFiles/path_record.dir/cpp/path_record.cpp.s

# Object files for target path_record
path_record_OBJECTS = \
"CMakeFiles/path_record.dir/cpp/path_record.cpp.o"

# External object files for target path_record
path_record_EXTERNAL_OBJECTS =

bin/path_record: example/CMakeFiles/path_record.dir/cpp/path_record.cpp.o
bin/path_record: example/CMakeFiles/path_record.dir/build.make
bin/path_record: ../lib/Linux/cpp/x86_64/libxCoreSDK.a
bin/path_record: ../lib/Linux/cpp/x86_64/libxMateModel.a
bin/path_record: example/CMakeFiles/path_record.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/robot/roake_param_identify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/path_record"
	cd /home/robot/robot/roake_param_identify/build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/path_record.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/path_record.dir/build: bin/path_record

.PHONY : example/CMakeFiles/path_record.dir/build

example/CMakeFiles/path_record.dir/clean:
	cd /home/robot/robot/roake_param_identify/build/example && $(CMAKE_COMMAND) -P CMakeFiles/path_record.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/path_record.dir/clean

example/CMakeFiles/path_record.dir/depend:
	cd /home/robot/robot/roake_param_identify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/robot/roake_param_identify /home/robot/robot/roake_param_identify/example /home/robot/robot/roake_param_identify/build /home/robot/robot/roake_param_identify/build/example /home/robot/robot/roake_param_identify/build/example/CMakeFiles/path_record.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/path_record.dir/depend

