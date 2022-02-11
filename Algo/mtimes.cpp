//
// File: mtimes.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:12:06
//

// Include Files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "vector_slam_gyro_data.h"

// Function Definitions

//
// Arguments    : const double A[225]
//                const coder::array<double, 2U> *B
//                coder::array<double, 2U> *C
// Return Type  : void
//
namespace SLAMALGO
{
  void mtimes(const double A[225], const coder::array<double, 2U> &B, coder::
              array<double, 2U> &C)
  {
    int n;
    double s;
    n = B.size(1);
    C.set_size(15, B.size(1));
    for (int j = 0; j < n; j++) {
      int coffset;
      int boffset;
      coffset = j * 15;
      boffset = j * 15;
      for (int i = 0; i < 15; i++) {
        s = 0.0;
        for (int k = 0; k < 15; k++) {
          s += A[k * 15 + i] * B[boffset + k];
        }

        C[coffset + i] = s;
      }
    }
  }
}

//
// File trailer for mtimes.cpp
//
// [EOF]
//
