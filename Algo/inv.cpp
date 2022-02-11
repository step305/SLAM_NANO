//
// File: inv.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:12:06
//

// Include Files
#include "inv.h"
#include "rt_nonfinite.h"
#include "vector_slam_gyro_data.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const double x[9]
//                double y[9]
// Return Type  : void
//
namespace SLAMALGO
{
  void inv(const double x[9], double y[9])
  {
    double b_x[9];
    int p1;
    int p2;
    int p3;
    double absx11;
    double absx21;
    double absx31;
    std::memcpy(&b_x[0], &x[0], 9U * sizeof(double));
    p1 = 0;
    p2 = 3;
    p3 = 6;
    absx11 = std::abs(x[0]);
    absx21 = std::abs(x[1]);
    absx31 = std::abs(x[2]);
    if ((absx21 > absx11) && (absx21 > absx31)) {
      p1 = 3;
      p2 = 0;
      b_x[0] = x[1];
      b_x[1] = x[0];
      b_x[3] = x[4];
      b_x[4] = x[3];
      b_x[6] = x[7];
      b_x[7] = x[6];
    } else {
      if (absx31 > absx11) {
        p1 = 6;
        p3 = 0;
        b_x[0] = x[2];
        b_x[2] = x[0];
        b_x[3] = x[5];
        b_x[5] = x[3];
        b_x[6] = x[8];
        b_x[8] = x[6];
      }
    }

    b_x[1] /= b_x[0];
    b_x[2] /= b_x[0];
    b_x[4] -= b_x[1] * b_x[3];
    b_x[5] -= b_x[2] * b_x[3];
    b_x[7] -= b_x[1] * b_x[6];
    b_x[8] -= b_x[2] * b_x[6];
    if (std::abs(b_x[5]) > std::abs(b_x[4])) {
      int itmp;
      itmp = p2;
      p2 = p3;
      p3 = itmp;
      absx11 = b_x[1];
      b_x[1] = b_x[2];
      b_x[2] = absx11;
      absx11 = b_x[4];
      b_x[4] = b_x[5];
      b_x[5] = absx11;
      absx11 = b_x[7];
      b_x[7] = b_x[8];
      b_x[8] = absx11;
    }

    b_x[5] /= b_x[4];
    b_x[8] -= b_x[5] * b_x[7];
    absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
    absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
    y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
    y[p1 + 1] = absx21;
    y[p1 + 2] = absx11;
    absx11 = -b_x[5] / b_x[8];
    absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
    y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p2 + 1] = absx21;
    y[p2 + 2] = absx11;
    absx11 = 1.0 / b_x[8];
    absx21 = -b_x[7] * absx11 / b_x[4];
    y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p3 + 1] = absx21;
    y[p3 + 2] = absx11;
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
