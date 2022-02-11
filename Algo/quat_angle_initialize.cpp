//
// File: quat_angle_initialize.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 14:31:37
//

// Include Files
#include "quat_angle_initialize.h"
#include "quat_angle.h"
#include "quat_angle_data.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
namespace SLAMALGO
{
  void quat_angle_initialize()
  {
    rt_InitInfAndNaN();
    isInitialized_quat_angle = true;
  }
}

//
// File trailer for quat_angle_initialize.cpp
//
// [EOF]
//
