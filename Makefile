# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/step305/SLAM_NANO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/step305/SLAM_NANO

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/step305/SLAM_NANO/CMakeFiles /home/step305/SLAM_NANO/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/step305/SLAM_NANO/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named SLAM_NANO

# Build rule for target.
SLAM_NANO: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 SLAM_NANO
.PHONY : SLAM_NANO

# fast build rule for target.
SLAM_NANO/fast:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/build
.PHONY : SLAM_NANO/fast

Algo/SLAM_algo_beauty.o: Algo/SLAM_algo_beauty.cpp.o

.PHONY : Algo/SLAM_algo_beauty.o

# target to build an object file
Algo/SLAM_algo_beauty.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/SLAM_algo_beauty.cpp.o
.PHONY : Algo/SLAM_algo_beauty.cpp.o

Algo/SLAM_algo_beauty.i: Algo/SLAM_algo_beauty.cpp.i

.PHONY : Algo/SLAM_algo_beauty.i

# target to preprocess a source file
Algo/SLAM_algo_beauty.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/SLAM_algo_beauty.cpp.i
.PHONY : Algo/SLAM_algo_beauty.cpp.i

Algo/SLAM_algo_beauty.s: Algo/SLAM_algo_beauty.cpp.s

.PHONY : Algo/SLAM_algo_beauty.s

# target to generate assembly for a file
Algo/SLAM_algo_beauty.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/SLAM_algo_beauty.cpp.s
.PHONY : Algo/SLAM_algo_beauty.cpp.s

Algo/inv.o: Algo/inv.cpp.o

.PHONY : Algo/inv.o

# target to build an object file
Algo/inv.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/inv.cpp.o
.PHONY : Algo/inv.cpp.o

Algo/inv.i: Algo/inv.cpp.i

.PHONY : Algo/inv.i

# target to preprocess a source file
Algo/inv.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/inv.cpp.i
.PHONY : Algo/inv.cpp.i

Algo/inv.s: Algo/inv.cpp.s

.PHONY : Algo/inv.s

# target to generate assembly for a file
Algo/inv.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/inv.cpp.s
.PHONY : Algo/inv.cpp.s

Algo/match.o: Algo/match.cpp.o

.PHONY : Algo/match.o

# target to build an object file
Algo/match.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/match.cpp.o
.PHONY : Algo/match.cpp.o

Algo/match.i: Algo/match.cpp.i

.PHONY : Algo/match.i

# target to preprocess a source file
Algo/match.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/match.cpp.i
.PHONY : Algo/match.cpp.i

Algo/match.s: Algo/match.cpp.s

.PHONY : Algo/match.s

# target to generate assembly for a file
Algo/match.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/match.cpp.s
.PHONY : Algo/match.cpp.s

Algo/mtimes.o: Algo/mtimes.cpp.o

.PHONY : Algo/mtimes.o

# target to build an object file
Algo/mtimes.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/mtimes.cpp.o
.PHONY : Algo/mtimes.cpp.o

Algo/mtimes.i: Algo/mtimes.cpp.i

.PHONY : Algo/mtimes.i

# target to preprocess a source file
Algo/mtimes.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/mtimes.cpp.i
.PHONY : Algo/mtimes.cpp.i

Algo/mtimes.s: Algo/mtimes.cpp.s

.PHONY : Algo/mtimes.s

# target to generate assembly for a file
Algo/mtimes.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/mtimes.cpp.s
.PHONY : Algo/mtimes.cpp.s

Algo/quat_angle.o: Algo/quat_angle.cpp.o

.PHONY : Algo/quat_angle.o

# target to build an object file
Algo/quat_angle.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle.cpp.o
.PHONY : Algo/quat_angle.cpp.o

Algo/quat_angle.i: Algo/quat_angle.cpp.i

.PHONY : Algo/quat_angle.i

# target to preprocess a source file
Algo/quat_angle.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle.cpp.i
.PHONY : Algo/quat_angle.cpp.i

Algo/quat_angle.s: Algo/quat_angle.cpp.s

.PHONY : Algo/quat_angle.s

# target to generate assembly for a file
Algo/quat_angle.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle.cpp.s
.PHONY : Algo/quat_angle.cpp.s

Algo/quat_angle_data.o: Algo/quat_angle_data.cpp.o

.PHONY : Algo/quat_angle_data.o

# target to build an object file
Algo/quat_angle_data.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle_data.cpp.o
.PHONY : Algo/quat_angle_data.cpp.o

Algo/quat_angle_data.i: Algo/quat_angle_data.cpp.i

.PHONY : Algo/quat_angle_data.i

# target to preprocess a source file
Algo/quat_angle_data.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle_data.cpp.i
.PHONY : Algo/quat_angle_data.cpp.i

Algo/quat_angle_data.s: Algo/quat_angle_data.cpp.s

.PHONY : Algo/quat_angle_data.s

# target to generate assembly for a file
Algo/quat_angle_data.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle_data.cpp.s
.PHONY : Algo/quat_angle_data.cpp.s

Algo/quat_angle_initialize.o: Algo/quat_angle_initialize.cpp.o

.PHONY : Algo/quat_angle_initialize.o

# target to build an object file
Algo/quat_angle_initialize.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle_initialize.cpp.o
.PHONY : Algo/quat_angle_initialize.cpp.o

Algo/quat_angle_initialize.i: Algo/quat_angle_initialize.cpp.i

.PHONY : Algo/quat_angle_initialize.i

# target to preprocess a source file
Algo/quat_angle_initialize.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle_initialize.cpp.i
.PHONY : Algo/quat_angle_initialize.cpp.i

Algo/quat_angle_initialize.s: Algo/quat_angle_initialize.cpp.s

.PHONY : Algo/quat_angle_initialize.s

# target to generate assembly for a file
Algo/quat_angle_initialize.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/quat_angle_initialize.cpp.s
.PHONY : Algo/quat_angle_initialize.cpp.s

Algo/rtGetInf.o: Algo/rtGetInf.cpp.o

.PHONY : Algo/rtGetInf.o

# target to build an object file
Algo/rtGetInf.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rtGetInf.cpp.o
.PHONY : Algo/rtGetInf.cpp.o

Algo/rtGetInf.i: Algo/rtGetInf.cpp.i

.PHONY : Algo/rtGetInf.i

# target to preprocess a source file
Algo/rtGetInf.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rtGetInf.cpp.i
.PHONY : Algo/rtGetInf.cpp.i

Algo/rtGetInf.s: Algo/rtGetInf.cpp.s

.PHONY : Algo/rtGetInf.s

# target to generate assembly for a file
Algo/rtGetInf.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rtGetInf.cpp.s
.PHONY : Algo/rtGetInf.cpp.s

Algo/rtGetNaN.o: Algo/rtGetNaN.cpp.o

.PHONY : Algo/rtGetNaN.o

# target to build an object file
Algo/rtGetNaN.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rtGetNaN.cpp.o
.PHONY : Algo/rtGetNaN.cpp.o

Algo/rtGetNaN.i: Algo/rtGetNaN.cpp.i

.PHONY : Algo/rtGetNaN.i

# target to preprocess a source file
Algo/rtGetNaN.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rtGetNaN.cpp.i
.PHONY : Algo/rtGetNaN.cpp.i

Algo/rtGetNaN.s: Algo/rtGetNaN.cpp.s

.PHONY : Algo/rtGetNaN.s

# target to generate assembly for a file
Algo/rtGetNaN.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rtGetNaN.cpp.s
.PHONY : Algo/rtGetNaN.cpp.s

Algo/rt_nonfinite.o: Algo/rt_nonfinite.cpp.o

.PHONY : Algo/rt_nonfinite.o

# target to build an object file
Algo/rt_nonfinite.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rt_nonfinite.cpp.o
.PHONY : Algo/rt_nonfinite.cpp.o

Algo/rt_nonfinite.i: Algo/rt_nonfinite.cpp.i

.PHONY : Algo/rt_nonfinite.i

# target to preprocess a source file
Algo/rt_nonfinite.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rt_nonfinite.cpp.i
.PHONY : Algo/rt_nonfinite.cpp.i

Algo/rt_nonfinite.s: Algo/rt_nonfinite.cpp.s

.PHONY : Algo/rt_nonfinite.s

# target to generate assembly for a file
Algo/rt_nonfinite.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/rt_nonfinite.cpp.s
.PHONY : Algo/rt_nonfinite.cpp.s

Algo/vector_slam_gyro_data.o: Algo/vector_slam_gyro_data.cpp.o

.PHONY : Algo/vector_slam_gyro_data.o

# target to build an object file
Algo/vector_slam_gyro_data.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data.cpp.o
.PHONY : Algo/vector_slam_gyro_data.cpp.o

Algo/vector_slam_gyro_data.i: Algo/vector_slam_gyro_data.cpp.i

.PHONY : Algo/vector_slam_gyro_data.i

# target to preprocess a source file
Algo/vector_slam_gyro_data.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data.cpp.i
.PHONY : Algo/vector_slam_gyro_data.cpp.i

Algo/vector_slam_gyro_data.s: Algo/vector_slam_gyro_data.cpp.s

.PHONY : Algo/vector_slam_gyro_data.s

# target to generate assembly for a file
Algo/vector_slam_gyro_data.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data.cpp.s
.PHONY : Algo/vector_slam_gyro_data.cpp.s

Algo/vector_slam_gyro_data_data.o: Algo/vector_slam_gyro_data_data.cpp.o

.PHONY : Algo/vector_slam_gyro_data_data.o

# target to build an object file
Algo/vector_slam_gyro_data_data.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data_data.cpp.o
.PHONY : Algo/vector_slam_gyro_data_data.cpp.o

Algo/vector_slam_gyro_data_data.i: Algo/vector_slam_gyro_data_data.cpp.i

.PHONY : Algo/vector_slam_gyro_data_data.i

# target to preprocess a source file
Algo/vector_slam_gyro_data_data.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data_data.cpp.i
.PHONY : Algo/vector_slam_gyro_data_data.cpp.i

Algo/vector_slam_gyro_data_data.s: Algo/vector_slam_gyro_data_data.cpp.s

.PHONY : Algo/vector_slam_gyro_data_data.s

# target to generate assembly for a file
Algo/vector_slam_gyro_data_data.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data_data.cpp.s
.PHONY : Algo/vector_slam_gyro_data_data.cpp.s

Algo/vector_slam_gyro_data_initialize.o: Algo/vector_slam_gyro_data_initialize.cpp.o

.PHONY : Algo/vector_slam_gyro_data_initialize.o

# target to build an object file
Algo/vector_slam_gyro_data_initialize.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data_initialize.cpp.o
.PHONY : Algo/vector_slam_gyro_data_initialize.cpp.o

Algo/vector_slam_gyro_data_initialize.i: Algo/vector_slam_gyro_data_initialize.cpp.i

.PHONY : Algo/vector_slam_gyro_data_initialize.i

# target to preprocess a source file
Algo/vector_slam_gyro_data_initialize.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data_initialize.cpp.i
.PHONY : Algo/vector_slam_gyro_data_initialize.cpp.i

Algo/vector_slam_gyro_data_initialize.s: Algo/vector_slam_gyro_data_initialize.cpp.s

.PHONY : Algo/vector_slam_gyro_data_initialize.s

# target to generate assembly for a file
Algo/vector_slam_gyro_data_initialize.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/Algo/vector_slam_gyro_data_initialize.cpp.s
.PHONY : Algo/vector_slam_gyro_data_initialize.cpp.s

ORBdetector.o: ORBdetector.cpp.o

.PHONY : ORBdetector.o

# target to build an object file
ORBdetector.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/ORBdetector.cpp.o
.PHONY : ORBdetector.cpp.o

ORBdetector.i: ORBdetector.cpp.i

.PHONY : ORBdetector.i

# target to preprocess a source file
ORBdetector.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/ORBdetector.cpp.i
.PHONY : ORBdetector.cpp.i

ORBdetector.s: ORBdetector.cpp.s

.PHONY : ORBdetector.s

# target to generate assembly for a file
ORBdetector.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/ORBdetector.cpp.s
.PHONY : ORBdetector.cpp.s

RealsenseD455.o: RealsenseD455.cpp.o

.PHONY : RealsenseD455.o

# target to build an object file
RealsenseD455.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/RealsenseD455.cpp.o
.PHONY : RealsenseD455.cpp.o

RealsenseD455.i: RealsenseD455.cpp.i

.PHONY : RealsenseD455.i

# target to preprocess a source file
RealsenseD455.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/RealsenseD455.cpp.i
.PHONY : RealsenseD455.cpp.i

RealsenseD455.s: RealsenseD455.cpp.s

.PHONY : RealsenseD455.s

# target to generate assembly for a file
RealsenseD455.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/RealsenseD455.cpp.s
.PHONY : RealsenseD455.cpp.s

SLAM_thread.o: SLAM_thread.cpp.o

.PHONY : SLAM_thread.o

# target to build an object file
SLAM_thread.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/SLAM_thread.cpp.o
.PHONY : SLAM_thread.cpp.o

SLAM_thread.i: SLAM_thread.cpp.i

.PHONY : SLAM_thread.i

# target to preprocess a source file
SLAM_thread.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/SLAM_thread.cpp.i
.PHONY : SLAM_thread.cpp.i

SLAM_thread.s: SLAM_thread.cpp.s

.PHONY : SLAM_thread.s

# target to generate assembly for a file
SLAM_thread.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/SLAM_thread.cpp.s
.PHONY : SLAM_thread.cpp.s

fifo_thread.o: fifo_thread.cpp.o

.PHONY : fifo_thread.o

# target to build an object file
fifo_thread.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/fifo_thread.cpp.o
.PHONY : fifo_thread.cpp.o

fifo_thread.i: fifo_thread.cpp.i

.PHONY : fifo_thread.i

# target to preprocess a source file
fifo_thread.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/fifo_thread.cpp.i
.PHONY : fifo_thread.cpp.i

fifo_thread.s: fifo_thread.cpp.s

.PHONY : fifo_thread.s

# target to generate assembly for a file
fifo_thread.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/fifo_thread.cpp.s
.PHONY : fifo_thread.cpp.s

main.o: main.cpp.o

.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i

.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s

.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/main.cpp.s
.PHONY : main.cpp.s

serialStream.o: serialStream.cpp.o

.PHONY : serialStream.o

# target to build an object file
serialStream.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/serialStream.cpp.o
.PHONY : serialStream.cpp.o

serialStream.i: serialStream.cpp.i

.PHONY : serialStream.i

# target to preprocess a source file
serialStream.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/serialStream.cpp.i
.PHONY : serialStream.cpp.i

serialStream.s: serialStream.cpp.s

.PHONY : serialStream.s

# target to generate assembly for a file
serialStream.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/serialStream.cpp.s
.PHONY : serialStream.cpp.s

syncThread.o: syncThread.cpp.o

.PHONY : syncThread.o

# target to build an object file
syncThread.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/syncThread.cpp.o
.PHONY : syncThread.cpp.o

syncThread.i: syncThread.cpp.i

.PHONY : syncThread.i

# target to preprocess a source file
syncThread.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/syncThread.cpp.i
.PHONY : syncThread.cpp.i

syncThread.s: syncThread.cpp.s

.PHONY : syncThread.s

# target to generate assembly for a file
syncThread.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/syncThread.cpp.s
.PHONY : syncThread.cpp.s

utils.o: utils.cpp.o

.PHONY : utils.o

# target to build an object file
utils.cpp.o:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/utils.cpp.o
.PHONY : utils.cpp.o

utils.i: utils.cpp.i

.PHONY : utils.i

# target to preprocess a source file
utils.cpp.i:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/utils.cpp.i
.PHONY : utils.cpp.i

utils.s: utils.cpp.s

.PHONY : utils.s

# target to generate assembly for a file
utils.cpp.s:
	$(MAKE) -f CMakeFiles/SLAM_NANO.dir/build.make CMakeFiles/SLAM_NANO.dir/utils.cpp.s
.PHONY : utils.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... SLAM_NANO"
	@echo "... edit_cache"
	@echo "... Algo/SLAM_algo_beauty.o"
	@echo "... Algo/SLAM_algo_beauty.i"
	@echo "... Algo/SLAM_algo_beauty.s"
	@echo "... Algo/inv.o"
	@echo "... Algo/inv.i"
	@echo "... Algo/inv.s"
	@echo "... Algo/match.o"
	@echo "... Algo/match.i"
	@echo "... Algo/match.s"
	@echo "... Algo/mtimes.o"
	@echo "... Algo/mtimes.i"
	@echo "... Algo/mtimes.s"
	@echo "... Algo/quat_angle.o"
	@echo "... Algo/quat_angle.i"
	@echo "... Algo/quat_angle.s"
	@echo "... Algo/quat_angle_data.o"
	@echo "... Algo/quat_angle_data.i"
	@echo "... Algo/quat_angle_data.s"
	@echo "... Algo/quat_angle_initialize.o"
	@echo "... Algo/quat_angle_initialize.i"
	@echo "... Algo/quat_angle_initialize.s"
	@echo "... Algo/rtGetInf.o"
	@echo "... Algo/rtGetInf.i"
	@echo "... Algo/rtGetInf.s"
	@echo "... Algo/rtGetNaN.o"
	@echo "... Algo/rtGetNaN.i"
	@echo "... Algo/rtGetNaN.s"
	@echo "... Algo/rt_nonfinite.o"
	@echo "... Algo/rt_nonfinite.i"
	@echo "... Algo/rt_nonfinite.s"
	@echo "... Algo/vector_slam_gyro_data.o"
	@echo "... Algo/vector_slam_gyro_data.i"
	@echo "... Algo/vector_slam_gyro_data.s"
	@echo "... Algo/vector_slam_gyro_data_data.o"
	@echo "... Algo/vector_slam_gyro_data_data.i"
	@echo "... Algo/vector_slam_gyro_data_data.s"
	@echo "... Algo/vector_slam_gyro_data_initialize.o"
	@echo "... Algo/vector_slam_gyro_data_initialize.i"
	@echo "... Algo/vector_slam_gyro_data_initialize.s"
	@echo "... ORBdetector.o"
	@echo "... ORBdetector.i"
	@echo "... ORBdetector.s"
	@echo "... RealsenseD455.o"
	@echo "... RealsenseD455.i"
	@echo "... RealsenseD455.s"
	@echo "... SLAM_thread.o"
	@echo "... SLAM_thread.i"
	@echo "... SLAM_thread.s"
	@echo "... fifo_thread.o"
	@echo "... fifo_thread.i"
	@echo "... fifo_thread.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... serialStream.o"
	@echo "... serialStream.i"
	@echo "... serialStream.s"
	@echo "... syncThread.o"
	@echo "... syncThread.i"
	@echo "... syncThread.s"
	@echo "... utils.o"
	@echo "... utils.i"
	@echo "... utils.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

