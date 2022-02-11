//
// File: vector_slam_gyro_data.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:44:06
//
#ifndef VECTOR_SLAM_GYRO_DATA_H
#define VECTOR_SLAM_GYRO_DATA_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "vector_slam_gyro_data_types.h"

// Function Declarations
namespace SLAMALGO
{
  extern void vector_slam_gyro_data(const frame_struct_type *frame, const coder::array<
    unsigned char, 2U> &frame_descriptors, const coder::array<double, 2U>
    &frame_points, map_struct_type *map, double q[4], coder::array<double, 2U> &P,
    double bw[3], double sw[3], double mw[6], double dt, const caminfo_struct_type *cam,
    double occupancy_map[1296], bool motion_flag);
}

#endif

//
// File trailer for vector_slam_gyro_data.h
//
// [EOF]
//
