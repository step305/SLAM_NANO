//
// File: quat_angle.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 14:31:37
//

// Include Files
#include "quat_angle.h"
#include "quat_angle_data.h"
#include "quat_angle_initialize.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <math.h>

// Function Declarations
namespace SLAMALGO
{
  static double rt_atan2d_snf(double u0, double u1);
}

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
namespace SLAMALGO
{
  static double rt_atan2d_snf(double u0, double u1)
  {
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
      y = rtNaN;
    } else if (rtIsInf(u0) && rtIsInf(u1)) {
      int b_u0;
      int b_u1;
      if (u0 > 0.0) {
        b_u0 = 1;
      } else {
        b_u0 = -1;
      }

      if (u1 > 0.0) {
        b_u1 = 1;
      } else {
        b_u1 = -1;
      }

      y = atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
    } else if (u1 == 0.0) {
      if (u0 > 0.0) {
        y = RT_PI / 2.0;
      } else if (u0 < 0.0) {
        y = -(RT_PI / 2.0);
      } else {
        y = 0.0;
      }
    } else {
      y = atan2(u0, u1);
    }

    return y;
  }

  //
  // function [rz, ry, rx] = quat_angle(q)
  // [rz, ry, rx] = quat_angle(q)
  //   Transforms quaternion to Euler angles
  //
  //    Input arguments:
  //    q -  Attitude quaternion [1,4]
  //
  //    Output arguments:
  //    rz, ry, rx - Euler angles around Z, Y and X axes correspondingly
  // Arguments    : const double q[4]
  //                double *rz
  //                double *ry
  //                double *rx
  // Return Type  : void
  //
  void quat_angle(const double q[4], double *rz, double *ry, double *rx)
  {
    double Cbn_idx_6;
    double rz_tmp;
    double b_rz_tmp;
    double c_rz_tmp;
    double d_rz_tmp;
    double d;
    if (!isInitialized_quat_angle) {
      quat_angle_initialize();
    }

    // 'quat_angle:11' Cbn = quat_dcm(q);
    //   Cbn = quat_dcm(q)
    //   Transform quaternion to direction cosine matrix
    //
    //    Input arguments:
    //    q -  Attitude quaternion [1,4]
    //
    //    Output arguments:
    //    Cbn - direction cosine matrix
    // 'quat_dcm:11' q0 = q(1);
    // 'quat_dcm:12' q1 = q(2);
    // 'quat_dcm:13' q2 = q(3);
    // 'quat_dcm:14' q3 = q(4);
    // 'quat_dcm:16' Cbn_1_1 = q0^2+q1^2-q2^2-q3^2;
    // 'quat_dcm:17' Cbn_1_2 = 2*(q1*q2+q0*q3);
    // 'quat_dcm:18' Cbn_1_3 = 2*(q1*q3-q0*q2);
    // 'quat_dcm:19' Cbn_2_1 = 2*(q1*q2-q0*q3);
    // 'quat_dcm:20' Cbn_2_2 = q0^2-q1^2+q2^2-q3^2;
    // 'quat_dcm:21' Cbn_2_3 = 2*(q2*q3+q0*q1);
    // 'quat_dcm:22' Cbn_3_1 = 2*(q1*q3+q0*q2);
    // 'quat_dcm:23' Cbn_3_2 = 2*(q2*q3-q0*q1);
    // 'quat_dcm:24' Cbn_3_3 = q0^2-q1^2-q2^2+q3^2;
    // 'quat_dcm:26' Cbn = [
    // 'quat_dcm:27'     Cbn_1_1 Cbn_1_2 Cbn_1_3
    // 'quat_dcm:28'     Cbn_2_1 Cbn_2_2 Cbn_2_3
    // 'quat_dcm:29'     Cbn_3_1 Cbn_3_2 Cbn_3_3
    // 'quat_dcm:30'     ];
    Cbn_idx_6 = 2.0 * (q[1] * q[3] - q[0] * q[2]);

    // 'quat_angle:12' [rz, ry, rx] = dcm_angle(Cbn);
    //   [rz, ry, rx] = dcm_angle(Cbn)
    //   Transforms direction cosine matrix to Euler angles
    //
    //    Input arguments:
    //    Cbn -  Direction cosine matrix [3,3]
    //
    //    Output arguments:
    //    rz, ry, rx - Euler angles around Z, Y and X axes correspondingly
    // 'dcm_angle:11' rz = atan2(Cbn(1,2),Cbn(1,1));
    rz_tmp = q[0] * q[0];
    b_rz_tmp = q[1] * q[1];
    c_rz_tmp = q[2] * q[2];
    d_rz_tmp = q[3] * q[3];
    *rz = rt_atan2d_snf(2.0 * (q[1] * q[2] + q[0] * q[3]), ((rz_tmp + b_rz_tmp)
      - c_rz_tmp) - d_rz_tmp);

    //  To prevent "gimbal lock" difficulties
    //  Or alternatively use ry = asin(-Cbn(1,3)) - but this will return complex 
    //  values for ry in "gimbal lock"
    // 'dcm_angle:16' if (Cbn(1,3)^2 < 1.0)
    d = Cbn_idx_6 * Cbn_idx_6;
    if (d < 1.0) {
      // 'dcm_angle:17' ry = -atan2(Cbn(1,3),sqrt(1-Cbn(1,3)^2));
      *ry = -rt_atan2d_snf(Cbn_idx_6, std::sqrt(1.0 - d));
    } else {
      // 'dcm_angle:18' else
      // 'dcm_angle:19' ry = -atan2(Cbn(1,3),0);
      *ry = -rt_atan2d_snf(Cbn_idx_6, 0.0);
    }

    // 'dcm_angle:22' rx = atan2(Cbn(2,3),Cbn(3,3));
    *rx = rt_atan2d_snf(2.0 * (q[2] * q[3] + q[0] * q[1]), ((rz_tmp - b_rz_tmp)
      - c_rz_tmp) + d_rz_tmp);
  }
}

//
// File trailer for quat_angle.cpp
//
// [EOF]
//
