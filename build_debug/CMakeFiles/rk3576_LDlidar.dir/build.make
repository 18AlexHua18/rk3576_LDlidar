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
CMAKE_SOURCE_DIR = /home/alex/Project/rk3576_LDlidar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alex/Project/rk3576_LDlidar/build_debug

# Include any dependencies generated for this target.
include CMakeFiles/rk3576_LDlidar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rk3576_LDlidar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rk3576_LDlidar.dir/flags.make

CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.o: CMakeFiles/rk3576_LDlidar.dir/flags.make
CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.o: ../src/config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.o"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.o -c /home/alex/Project/rk3576_LDlidar/src/config.cpp

CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.i"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Project/rk3576_LDlidar/src/config.cpp > CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.i

CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.s"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Project/rk3576_LDlidar/src/config.cpp -o CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.s

CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.o: CMakeFiles/rk3576_LDlidar.dir/flags.make
CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.o: ../src/logger.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.o"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.o -c /home/alex/Project/rk3576_LDlidar/src/logger.cpp

CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.i"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Project/rk3576_LDlidar/src/logger.cpp > CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.i

CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.s"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Project/rk3576_LDlidar/src/logger.cpp -o CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.s

CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.o: CMakeFiles/rk3576_LDlidar.dir/flags.make
CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.o"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.o -c /home/alex/Project/rk3576_LDlidar/src/main.cpp

CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.i"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Project/rk3576_LDlidar/src/main.cpp > CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.i

CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.s"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Project/rk3576_LDlidar/src/main.cpp -o CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.s

CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.o: CMakeFiles/rk3576_LDlidar.dir/flags.make
CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.o: ../src/packet_parser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.o"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.o -c /home/alex/Project/rk3576_LDlidar/src/packet_parser.cpp

CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.i"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Project/rk3576_LDlidar/src/packet_parser.cpp > CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.i

CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.s"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Project/rk3576_LDlidar/src/packet_parser.cpp -o CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.s

CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.o: CMakeFiles/rk3576_LDlidar.dir/flags.make
CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.o: ../src/point_cloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.o"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.o -c /home/alex/Project/rk3576_LDlidar/src/point_cloud.cpp

CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.i"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Project/rk3576_LDlidar/src/point_cloud.cpp > CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.i

CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.s"
	/home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Project/rk3576_LDlidar/src/point_cloud.cpp -o CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.s

# Object files for target rk3576_LDlidar
rk3576_LDlidar_OBJECTS = \
"CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.o" \
"CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.o" \
"CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.o" \
"CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.o" \
"CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.o"

# External object files for target rk3576_LDlidar
rk3576_LDlidar_EXTERNAL_OBJECTS =

bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/src/config.cpp.o
bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/src/logger.cpp.o
bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/src/main.cpp.o
bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/src/packet_parser.cpp.o
bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/src/point_cloud.cpp.o
bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/build.make
bin/rk3576_LDlidar: CMakeFiles/rk3576_LDlidar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable bin/rk3576_LDlidar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rk3576_LDlidar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rk3576_LDlidar.dir/build: bin/rk3576_LDlidar

.PHONY : CMakeFiles/rk3576_LDlidar.dir/build

CMakeFiles/rk3576_LDlidar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rk3576_LDlidar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rk3576_LDlidar.dir/clean

CMakeFiles/rk3576_LDlidar.dir/depend:
	cd /home/alex/Project/rk3576_LDlidar/build_debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Project/rk3576_LDlidar /home/alex/Project/rk3576_LDlidar /home/alex/Project/rk3576_LDlidar/build_debug /home/alex/Project/rk3576_LDlidar/build_debug /home/alex/Project/rk3576_LDlidar/build_debug/CMakeFiles/rk3576_LDlidar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rk3576_LDlidar.dir/depend

