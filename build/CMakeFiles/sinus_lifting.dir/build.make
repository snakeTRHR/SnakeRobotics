# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/tsuruharakota/SnakeRobotics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tsuruharakota/SnakeRobotics/build

# Include any dependencies generated for this target.
include CMakeFiles/sinus_lifting.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sinus_lifting.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sinus_lifting.dir/flags.make

CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o: CMakeFiles/sinus_lifting.dir/flags.make
CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o: ../sinus_lifting.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tsuruharakota/SnakeRobotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o -c /home/tsuruharakota/SnakeRobotics/sinus_lifting.cpp

CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tsuruharakota/SnakeRobotics/sinus_lifting.cpp > CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.i

CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tsuruharakota/SnakeRobotics/sinus_lifting.cpp -o CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.s

CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.requires:

.PHONY : CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.requires

CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.provides: CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.requires
	$(MAKE) -f CMakeFiles/sinus_lifting.dir/build.make CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.provides.build
.PHONY : CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.provides

CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.provides.build: CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o


# Object files for target sinus_lifting
sinus_lifting_OBJECTS = \
"CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o"

# External object files for target sinus_lifting
sinus_lifting_EXTERNAL_OBJECTS =

sinus_lifting: CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o
sinus_lifting: CMakeFiles/sinus_lifting.dir/build.make
sinus_lifting: /usr/lib/x86_64-linux-gnu/libpython2.7.so
sinus_lifting: CMakeFiles/sinus_lifting.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tsuruharakota/SnakeRobotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sinus_lifting"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sinus_lifting.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sinus_lifting.dir/build: sinus_lifting

.PHONY : CMakeFiles/sinus_lifting.dir/build

CMakeFiles/sinus_lifting.dir/requires: CMakeFiles/sinus_lifting.dir/sinus_lifting.cpp.o.requires

.PHONY : CMakeFiles/sinus_lifting.dir/requires

CMakeFiles/sinus_lifting.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sinus_lifting.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sinus_lifting.dir/clean

CMakeFiles/sinus_lifting.dir/depend:
	cd /home/tsuruharakota/SnakeRobotics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tsuruharakota/SnakeRobotics /home/tsuruharakota/SnakeRobotics /home/tsuruharakota/SnakeRobotics/build /home/tsuruharakota/SnakeRobotics/build /home/tsuruharakota/SnakeRobotics/build/CMakeFiles/sinus_lifting.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sinus_lifting.dir/depend

