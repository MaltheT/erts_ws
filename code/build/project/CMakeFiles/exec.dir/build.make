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
CMAKE_SOURCE_DIR = /home/malthet/erts_ws/code/src/project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/malthet/erts_ws/code/build/project

# Include any dependencies generated for this target.
include CMakeFiles/exec.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/exec.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exec.dir/flags.make

CMakeFiles/exec.dir/controller.cpp.o: CMakeFiles/exec.dir/flags.make
CMakeFiles/exec.dir/controller.cpp.o: /home/malthet/erts_ws/code/src/project/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/malthet/erts_ws/code/build/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exec.dir/controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exec.dir/controller.cpp.o -c /home/malthet/erts_ws/code/src/project/controller.cpp

CMakeFiles/exec.dir/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exec.dir/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/malthet/erts_ws/code/src/project/controller.cpp > CMakeFiles/exec.dir/controller.cpp.i

CMakeFiles/exec.dir/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exec.dir/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/malthet/erts_ws/code/src/project/controller.cpp -o CMakeFiles/exec.dir/controller.cpp.s

CMakeFiles/exec.dir/robot_arm.cpp.o: CMakeFiles/exec.dir/flags.make
CMakeFiles/exec.dir/robot_arm.cpp.o: /home/malthet/erts_ws/code/src/project/robot_arm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/malthet/erts_ws/code/build/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/exec.dir/robot_arm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exec.dir/robot_arm.cpp.o -c /home/malthet/erts_ws/code/src/project/robot_arm.cpp

CMakeFiles/exec.dir/robot_arm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exec.dir/robot_arm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/malthet/erts_ws/code/src/project/robot_arm.cpp > CMakeFiles/exec.dir/robot_arm.cpp.i

CMakeFiles/exec.dir/robot_arm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exec.dir/robot_arm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/malthet/erts_ws/code/src/project/robot_arm.cpp -o CMakeFiles/exec.dir/robot_arm.cpp.s

CMakeFiles/exec.dir/testbench.cpp.o: CMakeFiles/exec.dir/flags.make
CMakeFiles/exec.dir/testbench.cpp.o: /home/malthet/erts_ws/code/src/project/testbench.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/malthet/erts_ws/code/build/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/exec.dir/testbench.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exec.dir/testbench.cpp.o -c /home/malthet/erts_ws/code/src/project/testbench.cpp

CMakeFiles/exec.dir/testbench.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exec.dir/testbench.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/malthet/erts_ws/code/src/project/testbench.cpp > CMakeFiles/exec.dir/testbench.cpp.i

CMakeFiles/exec.dir/testbench.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exec.dir/testbench.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/malthet/erts_ws/code/src/project/testbench.cpp -o CMakeFiles/exec.dir/testbench.cpp.s

CMakeFiles/exec.dir/controller_driver.cpp.o: CMakeFiles/exec.dir/flags.make
CMakeFiles/exec.dir/controller_driver.cpp.o: /home/malthet/erts_ws/code/src/project/controller_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/malthet/erts_ws/code/build/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/exec.dir/controller_driver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exec.dir/controller_driver.cpp.o -c /home/malthet/erts_ws/code/src/project/controller_driver.cpp

CMakeFiles/exec.dir/controller_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exec.dir/controller_driver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/malthet/erts_ws/code/src/project/controller_driver.cpp > CMakeFiles/exec.dir/controller_driver.cpp.i

CMakeFiles/exec.dir/controller_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exec.dir/controller_driver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/malthet/erts_ws/code/src/project/controller_driver.cpp -o CMakeFiles/exec.dir/controller_driver.cpp.s

# Object files for target exec
exec_OBJECTS = \
"CMakeFiles/exec.dir/controller.cpp.o" \
"CMakeFiles/exec.dir/robot_arm.cpp.o" \
"CMakeFiles/exec.dir/testbench.cpp.o" \
"CMakeFiles/exec.dir/controller_driver.cpp.o"

# External object files for target exec
exec_EXTERNAL_OBJECTS =

exec: CMakeFiles/exec.dir/controller.cpp.o
exec: CMakeFiles/exec.dir/robot_arm.cpp.o
exec: CMakeFiles/exec.dir/testbench.cpp.o
exec: CMakeFiles/exec.dir/controller_driver.cpp.o
exec: CMakeFiles/exec.dir/build.make
exec: /home/malthet/systemc-2.3.3-install/lib/libsystemc-d.so.2.3.3
exec: CMakeFiles/exec.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/malthet/erts_ws/code/build/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable exec"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exec.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exec.dir/build: exec

.PHONY : CMakeFiles/exec.dir/build

CMakeFiles/exec.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exec.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exec.dir/clean

CMakeFiles/exec.dir/depend:
	cd /home/malthet/erts_ws/code/build/project && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/malthet/erts_ws/code/src/project /home/malthet/erts_ws/code/src/project /home/malthet/erts_ws/code/build/project /home/malthet/erts_ws/code/build/project /home/malthet/erts_ws/code/build/project/CMakeFiles/exec.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exec.dir/depend

