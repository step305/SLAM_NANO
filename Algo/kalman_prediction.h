//
// File: kalman_prediction.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 07-Feb-2022 16:22:32
//
#ifndef KALMAN_PREDICTION_H
#define KALMAN_PREDICTION_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "kalman_prediction_types.h"

// Function Declarations
namespace SLAMALGO
{
  extern void kalman_prediction(const double bw[3], const double mw[6], const
    double sw[3], double dthe[3], double dt, double q[4], double P_data[], const
    int P_size[2], double Cbn[9], double F[225]);
}

#endif

//
// File trailer for kalman_prediction.h
//
// [EOF]
//
