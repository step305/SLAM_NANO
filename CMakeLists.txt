# cmake_minimum_required(VERSION <specify CMake version here>)
project(SLAM_NANO)

set(CMAKE_CXX_STANDARD 14)
enable_language(CUDA)
cmake_minimum_required(VERSION 3.10)

add_executable(
        SLAM_NANO
        main.cpp
        serialStream.cpp
        ORBdetector.cpp
        utils.cpp
        RealsenseD455.cpp
        syncThread.cpp
        SLAM_thread.cpp
        fifo_thread.cpp
        Algo/SLAM_algo_beauty.cpp
        Algo/inv.cpp
        Algo/match.cpp
        Algo/quat_angle.cpp
        Algo/quat_angle_data.cpp
        Algo/quat_angle_initialize.cpp
        Algo/rt_nonfinite.cpp
        Algo/rtGetInf.cpp
        Algo/rtGetNaN.cpp
        Algo/vector_slam_gyro_data.cpp
        Algo/vector_slam_gyro_data_data.cpp
        Algo/vector_slam_gyro_data_initialize.cpp
        Algo/mtimes.cpp
        base64.cpp
        )

find_package(OpenCV REQUIRED)
find_package (Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C HL)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wunused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -flto -funroll-loops -fvect-cost-model=unlimited")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --verbose -O3")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "--verbose -O3" )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/cuda/include")
include_directories(${HDF5_INCLUDE_DIRS})


target_link_libraries(${PROJECT_NAME}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        ${HDF5_LIBRARIES}
        )