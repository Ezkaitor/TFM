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
CMAKE_SOURCE_DIR = /home/ezkaitor/catkin_ws/src/drone-test/include/plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build

# Include any dependencies generated for this target.
include CMakeFiles/drone_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/drone_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/drone_plugin.dir/flags.make

CMakeFiles/drone_plugin.dir/drone_move.cc.o: CMakeFiles/drone_plugin.dir/flags.make
CMakeFiles/drone_plugin.dir/drone_move.cc.o: ../drone_move.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/drone_plugin.dir/drone_move.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drone_plugin.dir/drone_move.cc.o -c /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/drone_move.cc

CMakeFiles/drone_plugin.dir/drone_move.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_plugin.dir/drone_move.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/drone_move.cc > CMakeFiles/drone_plugin.dir/drone_move.cc.i

CMakeFiles/drone_plugin.dir/drone_move.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_plugin.dir/drone_move.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/drone_move.cc -o CMakeFiles/drone_plugin.dir/drone_move.cc.s

# Object files for target drone_plugin
drone_plugin_OBJECTS = \
"CMakeFiles/drone_plugin.dir/drone_move.cc.o"

# External object files for target drone_plugin
drone_plugin_EXTERNAL_OBJECTS =

libdrone_plugin.so: CMakeFiles/drone_plugin.dir/drone_move.cc.o
libdrone_plugin.so: CMakeFiles/drone_plugin.dir/build.make
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.5.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.13.1
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libccd.so
libdrone_plugin.so: /usr/local/lib/libfcl.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.2.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.3.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.7.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.8.0
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.13.1
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libdrone_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libdrone_plugin.so: CMakeFiles/drone_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libdrone_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drone_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/drone_plugin.dir/build: libdrone_plugin.so

.PHONY : CMakeFiles/drone_plugin.dir/build

CMakeFiles/drone_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/drone_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/drone_plugin.dir/clean

CMakeFiles/drone_plugin.dir/depend:
	cd /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ezkaitor/catkin_ws/src/drone-test/include/plugins /home/ezkaitor/catkin_ws/src/drone-test/include/plugins /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build /home/ezkaitor/catkin_ws/src/drone-test/include/plugins/build/CMakeFiles/drone_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/drone_plugin.dir/depend

