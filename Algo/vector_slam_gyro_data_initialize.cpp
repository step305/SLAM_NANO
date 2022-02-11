//
// File: vector_slam_gyro_data_initialize.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:44:06
//

// Include Files
#include "vector_slam_gyro_data_initialize.h"
#include "rt_nonfinite.h"
#include "vector_slam_gyro_data.h"
#include "vector_slam_gyro_data_data.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
namespace SLAMALGO
{
  void vector_slam_gyro_data_initialize()
  {
    rt_InitInfAndNaN();
    isInitialized_vector_slam_gyro_data = true;
  }
}

//
// File trailer for vector_slam_gyro_data_initialize.cpp
//
// [EOF]
//
