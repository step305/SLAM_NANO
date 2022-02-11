//
// File: vector_slam_gyro_data_types.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:12:06
//
#ifndef VECTOR_SLAM_GYRO_DATA_TYPES_H
#define VECTOR_SLAM_GYRO_DATA_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4251)

#endif

// Type Definitions
namespace SLAMALGO
{
  struct frame_struct_type
  {
    bool sync;
    double dthe[3];
  };

  struct feature_struct_type
  {
    double en[3];
    double eb[3];
    double pos[3];
    unsigned char des[32];
    double mat;
    double obs;
    double inv;
    double uv[2];
    double cnt_obs;
    double cnt_mat;
    double ceil_coord[2];
  };

  struct caminfo_struct_type
  {
    unsigned int nRows;
    unsigned int nCols;
    double fc[2];
    double cc[2];
    double kc[5];
    double frame_rate;
  };

  struct map_struct_type
  {
    double size;
    coder::array<feature_struct_type, 1U> data;
  };
}

#ifdef _MSC_VER

#pragma warning(pop)

#endif
#endif

//
// File trailer for vector_slam_gyro_data_types.h
//
// [EOF]
//
