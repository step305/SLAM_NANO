//
// File: mtimes.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:12:06
//
#ifndef MTIMES_H
#define MTIMES_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "vector_slam_gyro_data_types.h"

// Function Declarations
namespace SLAMALGO
{
  extern void mtimes(const double A[225], const coder::array<double, 2U> &B,
                     coder::array<double, 2U> &C);
}

#endif

//
// File trailer for mtimes.h
//
// [EOF]
//
