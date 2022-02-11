//
// File: kalman_prediction.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 07-Feb-2022 16:22:32
//

// Include Files
#include "kalman_prediction.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// function [q, Cbn, P, F] = kalman_prediction(bw, mw, sw, dthe, dt, q, nxv, P)
// Correct attitide increments  with estimated bias and scale factor
// Arguments    : const double bw[3]
//                const double mw[6]
//                const double sw[3]
//                double dthe[3]
//                double dt
//                double q[4]
//                double P_data[]
//                const int P_size[2]
//                double Cbn[9]
//                double F[225]
// Return Type  : void
//
namespace SLAMALGO
{
  void kalman_prediction(const double bw[3], const double mw[6], const double
    sw[3], double dthe[3], double dt, double q[4], double P_data[], const int
    P_size[2], double Cbn[9], double F[225])
  {
    double b_I[9];
    double b_gamma;
    double dthe_idx_1;
    double lambda_idx_0;
    double lambda_idx_1;
    int j;
    double lambda_idx_2;
    double lambda[16];
    double y[4];
    double A[225];
    int coffset;
    double c_I[9];
    int aoffset;
    double b_dthe[18];
    int k;
    double b_F[225];
    double P[225];
    static const double dv[225] = { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001 };

    static const double b[225] = { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001 };

    // 'kalman_prediction:5' E = [sw(1) mw(1) mw(2)
    // 'kalman_prediction:6'         mw(3) sw(2) mw(4)
    // 'kalman_prediction:7'         mw(5) mw(6) sw(3)];
    // 'kalman_prediction:8' dthe = (eye(3) + E) * (dthe + bw*dt);
    b_I[0] = sw[0] + 1.0;
    b_I[1] = mw[2];
    b_I[2] = mw[4];
    b_gamma = dthe[0] + bw[0] * dt;
    b_I[3] = mw[0];
    b_I[4] = sw[1] + 1.0;
    b_I[5] = mw[5];
    dthe_idx_1 = dthe[1] + bw[1] * dt;
    b_I[6] = mw[1];
    b_I[7] = mw[3];
    b_I[8] = sw[2] + 1.0;
    lambda_idx_0 = dthe[2] + bw[2] * dt;

    //     %% Attitude Quaternion Mechanization
    // Quaternion from k+1 to k body axes
    // 'kalman_prediction:12' gamma1 = dthe(1);
    // 'kalman_prediction:13' gamma2 = dthe(2);
    // 'kalman_prediction:14' gamma3 = dthe(3);
    // 'kalman_prediction:15' gamma  = sqrt(dthe'*dthe);
    lambda_idx_1 = 0.0;
    for (j = 0; j < 3; j++) {
      lambda_idx_2 = (b_I[j] * b_gamma + b_I[j + 3] * dthe_idx_1) + b_I[j + 6] *
        lambda_idx_0;
      dthe[j] = lambda_idx_2;
      lambda_idx_1 += lambda_idx_2 * lambda_idx_2;
    }

    b_gamma = std::sqrt(lambda_idx_1);

    // 'kalman_prediction:16' lambda0 = cos(gamma/2);
    // 'kalman_prediction:17' lambda1 = -gamma1*sin(gamma/2)/gamma;
    // 'kalman_prediction:18' lambda2 = -gamma2*sin(gamma/2)/gamma;
    // 'kalman_prediction:19' lambda3 = -gamma3*sin(gamma/2)/gamma;
    // 'kalman_prediction:20' if (gamma > 1e-16)
    if (b_gamma > 1.0E-16) {
      // 'kalman_prediction:21' lambda = [lambda0, lambda1, lambda2, lambda3];
      lambda_idx_0 = std::cos(b_gamma / 2.0);
      dthe_idx_1 = std::sin(b_gamma / 2.0);
      lambda_idx_1 = -dthe[0] * dthe_idx_1 / b_gamma;
      lambda_idx_2 = -dthe[1] * dthe_idx_1 / b_gamma;
      b_gamma = -dthe[2] * dthe_idx_1 / b_gamma;
    } else {
      // 'kalman_prediction:22' else
      // 'kalman_prediction:23' lambda = [1, 0, 0, 0];
      lambda_idx_0 = 1.0;
      lambda_idx_1 = 0.0;
      lambda_idx_2 = 0.0;
      b_gamma = 0.0;
    }

    // Update q for body motion
    // 'kalman_prediction:26' q = quat_mult(lambda,q);
    //   q = quat_mult(lam, mu)
    //   Multiplies quaternions  q = lam*mu
    //
    //    Input arguments:
    //    lam -  Attitude quaternion [1,4]
    //    mu -  Attitude quaternion [1,4]
    //
    //    Output arguments:
    //    q -   Attitude quaternion [1,4]
    // 'quat_mult:12' l0 = lam(1);
    // 'quat_mult:13' l1 = lam(2);
    // 'quat_mult:14' l2 = lam(3);
    // 'quat_mult:15' l3 = lam(4);
    // 'quat_mult:17' m = [
    // 'quat_mult:18'     l0 -l1 -l2 -l3
    // 'quat_mult:19'     l1  l0 -l3  l2
    // 'quat_mult:20'     l2  l3  l0 -l1
    // 'quat_mult:21'     l3 -l2  l1  l0
    // 'quat_mult:22'     ];
    // 'quat_mult:23' q = (m*mu')';
    lambda[0] = lambda_idx_0;
    lambda[4] = -lambda_idx_1;
    lambda[8] = -lambda_idx_2;
    lambda[12] = -b_gamma;
    lambda[1] = lambda_idx_1;
    lambda[5] = lambda_idx_0;
    lambda[9] = -b_gamma;
    lambda[13] = lambda_idx_2;
    lambda[2] = lambda_idx_2;
    lambda[6] = b_gamma;
    lambda[10] = lambda_idx_0;
    lambda[14] = -lambda_idx_1;
    lambda[3] = b_gamma;
    lambda[7] = -lambda_idx_2;
    lambda[11] = lambda_idx_1;
    lambda[15] = lambda_idx_0;

    // Normalize quaternion
    // 'kalman_prediction:28' q = q/sqrt(q*q');
    b_gamma = 0.0;
    for (j = 0; j < 4; j++) {
      lambda_idx_2 = ((lambda[j] * q[0] + lambda[j + 4] * q[1]) + lambda[j + 8] *
                      q[2]) + lambda[j + 12] * q[3];
      y[j] = lambda_idx_2;
      b_gamma += lambda_idx_2 * lambda_idx_2;
    }

    double Cbn_tmp;
    double b_Cbn_tmp;
    double c_Cbn_tmp;
    b_gamma = std::sqrt(b_gamma);
    q[0] = y[0] / b_gamma;
    q[1] = y[1] / b_gamma;
    q[2] = y[2] / b_gamma;
    q[3] = y[3] / b_gamma;

    //  DCM
    // 'kalman_prediction:30' Cbn = quat_dcm(q);
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
    dthe_idx_1 = q[0] * q[0];
    b_gamma = q[1] * q[1];
    lambda_idx_0 = q[2] * q[2];
    lambda_idx_1 = q[3] * q[3];
    Cbn[0] = ((dthe_idx_1 + b_gamma) - lambda_idx_0) - lambda_idx_1;
    lambda_idx_2 = q[1] * q[2];
    Cbn_tmp = q[0] * q[3];
    Cbn[3] = 2.0 * (lambda_idx_2 + Cbn_tmp);
    b_Cbn_tmp = q[1] * q[3];
    c_Cbn_tmp = q[0] * q[2];
    Cbn[6] = 2.0 * (b_Cbn_tmp - c_Cbn_tmp);
    Cbn[1] = 2.0 * (lambda_idx_2 - Cbn_tmp);
    dthe_idx_1 -= b_gamma;
    Cbn[4] = (dthe_idx_1 + lambda_idx_0) - lambda_idx_1;
    b_gamma = q[2] * q[3];
    lambda_idx_2 = q[0] * q[1];
    Cbn[7] = 2.0 * (b_gamma + lambda_idx_2);
    Cbn[2] = 2.0 * (b_Cbn_tmp + c_Cbn_tmp);
    Cbn[5] = 2.0 * (b_gamma - lambda_idx_2);
    Cbn[8] = (dthe_idx_1 - lambda_idx_0) + lambda_idx_1;

    //     %% Kalman predict
    // System dynamics matrix F and system noise covariance matrix Q
    // Continuous-time system matrix
    // 'kalman_prediction:35' A = zeros(nxv,nxv);
    std::memset(&A[0], 0, 225U * sizeof(double));

    // 'kalman_prediction:36' A(1:3,4:6) = -Cbn;
    for (j = 0; j < 3; j++) {
      coffset = 15 * (j + 3);
      A[coffset] = -Cbn[3 * j];
      A[coffset + 1] = -Cbn[3 * j + 1];
      A[coffset + 2] = -Cbn[3 * j + 2];
    }

    // 'kalman_prediction:37' A(1:3,7:9) = -Cbn*diag(dthe);
    std::memset(&c_I[0], 0, 9U * sizeof(double));
    c_I[0] = dthe[0];
    c_I[4] = dthe[1];
    c_I[8] = dthe[2];
    for (j = 0; j < 9; j++) {
      b_I[j] = -Cbn[j];
    }

    for (j = 0; j < 3; j++) {
      lambda_idx_2 = b_I[j + 3];
      b_gamma = b_I[j + 6];
      for (aoffset = 0; aoffset < 3; aoffset++) {
        A[j + 15 * (aoffset + 6)] = (b_I[j] * c_I[3 * aoffset] + lambda_idx_2 *
          c_I[3 * aoffset + 1]) + b_gamma * c_I[3 * aoffset + 2];
      }
    }

    // 'kalman_prediction:38' G = [dthe(2), dthe(3), 0, 0, 0, 0
    // 'kalman_prediction:39'         0, 0, dthe(1), dthe(3), 0, 0
    // 'kalman_prediction:40'         0, 0, 0, 0, dthe(1), dthe(2)];
    // 'kalman_prediction:41' A(1:3,10:15) = -Cbn*G;
    for (j = 0; j < 9; j++) {
      b_I[j] = -Cbn[j];
    }

    b_dthe[0] = dthe[1];
    b_dthe[3] = dthe[2];
    b_dthe[6] = 0.0;
    b_dthe[9] = 0.0;
    b_dthe[12] = 0.0;
    b_dthe[15] = 0.0;
    b_dthe[1] = 0.0;
    b_dthe[4] = 0.0;
    b_dthe[7] = dthe[0];
    b_dthe[10] = dthe[2];
    b_dthe[13] = 0.0;
    b_dthe[16] = 0.0;
    b_dthe[2] = 0.0;
    b_dthe[5] = 0.0;
    b_dthe[8] = 0.0;
    b_dthe[11] = 0.0;
    b_dthe[14] = dthe[0];
    b_dthe[17] = dthe[1];
    for (j = 0; j < 3; j++) {
      lambda_idx_2 = b_I[j + 3];
      b_gamma = b_I[j + 6];
      for (aoffset = 0; aoffset < 6; aoffset++) {
        A[j + 15 * (aoffset + 9)] = (b_I[j] * b_dthe[3 * aoffset] + lambda_idx_2
          * b_dthe[3 * aoffset + 1]) + b_gamma * b_dthe[3 * aoffset + 2];
      }
    }

    // State transition matrix
    // 'kalman_prediction:43' F = eye(nxv)+A*dt;
    std::memset(&F[0], 0, 225U * sizeof(double));
    for (k = 0; k < 15; k++) {
      F[k + 15 * k] = 1.0;
    }

    for (j = 0; j < 225; j++) {
      F[j] += A[j] * dt;
    }

    // Attitude errors noise
    // 'kalman_prediction:45' na = 1e-4;
    // Bias noise
    // 'kalman_prediction:47' nr = 1e-4;
    // Scale noise
    // 'kalman_prediction:49' ns = 1e-7;
    // Misalignment noise
    // 'kalman_prediction:51' nm = 1e-3;
    // System noise
    // 'kalman_prediction:53' Qn = diag([
    // 'kalman_prediction:54'         na, na, na,...
    // 'kalman_prediction:55'         nr, nr, nr,...
    // 'kalman_prediction:56'         ns, ns, ns,...
    // 'kalman_prediction:57'         nm, nm, nm, nm, nm, nm]);
    // Trapezioidal integration
    // 'kalman_prediction:59' Q = dt/2*(F*Qn+Qn'*F');
    dthe_idx_1 = dt / 2.0;

    // Covariance predict:
    // Vehicle
    // 'kalman_prediction:62' P(1:nxv,1:nxv) = F*P(1:nxv,1:nxv)*F'+Q;
    for (j = 0; j < 15; j++) {
      coffset = j * 15;
      for (int i = 0; i < 15; i++) {
        aoffset = i * 15;
        b_gamma = 0.0;
        for (k = 0; k < 15; k++) {
          b_gamma += dv[aoffset + k] * F[k * 15 + j];
        }

        A[coffset + i] = b_gamma;
        P[i + 15 * j] = P_data[i + P_size[0] * j];
      }
    }

    for (j = 0; j < 15; j++) {
      for (aoffset = 0; aoffset < 15; aoffset++) {
        lambda_idx_2 = 0.0;
        for (k = 0; k < 15; k++) {
          lambda_idx_2 += F[j + 15 * k] * P[k + 15 * aoffset];
        }

        b_F[j + 15 * aoffset] = lambda_idx_2;
      }
    }

    for (j = 0; j < 15; j++) {
      for (aoffset = 0; aoffset < 15; aoffset++) {
        lambda_idx_2 = 0.0;
        b_gamma = 0.0;
        for (k = 0; k < 15; k++) {
          coffset = j + 15 * k;
          lambda_idx_2 += F[coffset] * b[k + 15 * aoffset];
          b_gamma += b_F[coffset] * F[aoffset + 15 * k];
        }

        P_data[j + P_size[0] * aoffset] = b_gamma + dthe_idx_1 * (lambda_idx_2 +
          A[j + 15 * aoffset]);
      }
    }
  }
}

//
// File trailer for kalman_prediction.cpp
//
// [EOF]
//
