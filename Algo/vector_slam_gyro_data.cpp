//
// File: vector_slam_gyro_data.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:44:06
//

// Include Files
#include "vector_slam_gyro_data.h"
#include "inv.h"
#include "match.h"
#include "mtimes.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "vector_slam_gyro_data_data.h"
#include "vector_slam_gyro_data_initialize.h"
#include <cmath>
#include <cstring>
#include <math.h>

#define R00     0.03
#define na00    1e-4
#define nr00    1e-4
#define ns00    1e-5
#define nm00    1e-3

// Type Definitions
namespace SLAMALGO
{
  struct struct_T
  {
    double size;
    coder::array<feature_struct_type, 1U> data;
  };
}

// Function Declarations
namespace SLAMALGO
{
  static void augment(struct_T *map, coder::array<double, 2U> &P, const double
                      points[200], const unsigned char descriptors[3200], double
                      npoints, const double Cbn[9], double occupancy_map[1296]);
  static void cholesky_update(coder::array<double, 1U> &x, const coder::array<
    double, 2U> &P, const double v[3], const double R[9], const coder::array<
    double, 2U> &H, coder::array<double, 2U> &Pnew, bool *updated);
  static double rt_atan2d_snf(double u0, double u1);
  static double rt_powd_snf(double u0, double u1);
}

// Function Definitions

//
// function [map, P, occupancy_map] = augment(map, P, points, descriptors, npoints, Cbn, nxv, occupancy_map)
// Arguments    : SLAMALGO::struct_T *map
//                coder::array<double, 2U> &P
//                const double points[200]
//                const unsigned char descriptors[3200]
//                double npoints
//                const double Cbn[9]
//                double occupancy_map[1296]
// Return Type  : void
//
namespace SLAMALGO
{
  static void augment(struct_T *map, coder::array<double, 2U> &P, const double
                      points[200], const unsigned char descriptors[3200], double
                      npoints, const double Cbn[9], double occupancy_map[1296])
  {
    double nadd;
    coder::array<double, 2U> PPP;
    double Hv[45];
    coder::array<feature_struct_type, 1U> map_data_new;
    static const feature_struct_type r = { { 0.0, 0.0, 0.0 },// en
      { 0.0, 0.0, 0.0 },               // eb
      { 0.0, 0.0, 0.0 },               // pos
      { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },// des
      0.0,                             // mat
      0.0,                             // obs
      0.0,                             // inv
      { 0.0, 0.0 },                    // uv
      0.0,                             // cnt_obs
      0.0,                             // cnt_mat
      { 0.0, 0.0 }                     // ceil_coord
    };

    coder::array<int, 2U> r1;
    double pos_f[3];
    double en_meas[3];
    double b_Hv[9];
    double b_Cbn[9];
    double c_Cbn[9];
    static const double b[9] = { 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 };

    double c_Hv[45];

    //  Augment State
    // 'vector_slam_gyro_data:251' MAX_MAP_SIZE = 600;
    // 'vector_slam_gyro_data:252' R0= 0.1;
    // 'vector_slam_gyro_data:253' OCCUPANCY_GRID_STEP = 10;
    // 'vector_slam_gyro_data:254' if map.size >= MAX_MAP_SIZE
    if (!(map->size >= 600.0)) {
      int loop_ub;
      int i;
      int b_loop_ub;
      int i1;
      int b_i;
      bool exitg1;

      // 'vector_slam_gyro_data:257' nadd = MAX_MAP_SIZE - map.size;
      nadd = 600.0 - map->size;

      // 'vector_slam_gyro_data:258' if nadd > npoints
      if (600.0 - map->size > npoints) {
        // 'vector_slam_gyro_data:259' nadd = npoints;
        nadd = npoints;
      }

      // 'vector_slam_gyro_data:261' Psize = size(P,1);
      // 'vector_slam_gyro_data:262' PPP = zeros(Psize+3*nadd, Psize+3*nadd);
      PPP.set_size((static_cast<int>(static_cast<double>(P.size(0)) + 3.0 * nadd)),
                   (static_cast<int>(static_cast<double>(P.size(0)) + 3.0 * nadd)));
      loop_ub = static_cast<int>(static_cast<double>(P.size(0)) + 3.0 * nadd) *
        static_cast<int>(static_cast<double>(P.size(0)) + 3.0 * nadd);
      for (i = 0; i < loop_ub; i++) {
        PPP[i] = 0.0;
      }

      // 'vector_slam_gyro_data:263' PPP(1:Psize, 1:Psize) = P;
      loop_ub = P.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = P.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          PPP[i1 + PPP.size(0) * i] = P[i1 + P.size(0) * i];
        }
      }

      // 'vector_slam_gyro_data:264' P = PPP;
      P.set_size(PPP.size(0), PPP.size(1));
      loop_ub = PPP.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = PPP.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          P[i1 + P.size(0) * i] = PPP[i1 + PPP.size(0) * i];
        }
      }

      // 'vector_slam_gyro_data:265' Hv = zeros(3,nxv);
      std::memset(&Hv[0], 0, 45U * sizeof(double));

      // 'vector_slam_gyro_data:267' feature.en = zeros(3,1);
      // unit vector in nav frame
      // 'vector_slam_gyro_data:268' feature.eb = zeros(3,1);
      // unit vector in body frame
      // 'vector_slam_gyro_data:269' feature.pos = zeros(1,3);
      // feature position in sate vector
      // 'vector_slam_gyro_data:270' feature.des = uint8(zeros(1,32));
      // feature ORB descriptor
      // 'vector_slam_gyro_data:271' feature.mat = 0;
      // match flag
      // 'vector_slam_gyro_data:272' feature.obs = 0;
      // observe flag
      // 'vector_slam_gyro_data:273' feature.inv = 0;
      // invisible flag
      // 'vector_slam_gyro_data:274' feature.uv = zeros(2,1);
      // 'vector_slam_gyro_data:275' feature.cnt_obs = 0;
      // 'vector_slam_gyro_data:276' feature.cnt_mat = 0;
      // 'vector_slam_gyro_data:277' feature.ceil_coord = zeros(2,1);
      // 'vector_slam_gyro_data:278' map_data_new = repmat(feature,map.size + nadd,1); 
      loop_ub = static_cast<int>(map->size + nadd);
      map_data_new.set_size(loop_ub);
      for (i = 0; i < loop_ub; i++) {
        map_data_new[i] = r;
      }

      // 'vector_slam_gyro_data:279' map_data_new(1:map.size) = map.data;
      if (1.0 > map->size) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(map->size);
      }

      r1.set_size(1, loop_ub);
      for (i = 0; i < loop_ub; i++) {
        r1[i] = i;
      }

      loop_ub = r1.size(0) * r1.size(1);
      for (i = 0; i < loop_ub; i++) {
        map_data_new[r1[i]] = map->data[i];
      }

      // 'vector_slam_gyro_data:280' map.data = map_data_new;
      map->data.set_size(map_data_new.size(0));
      loop_ub = map_data_new.size(0);
      for (i = 0; i < loop_ub; i++) {
        map->data[i] = map_data_new[i];
      }

      //     if motion_flag == 1
      //         return
      //     end
      // 'vector_slam_gyro_data:286' R = diag([R0, R0, R0]);
      // 'vector_slam_gyro_data:287' pos_f = [1,2,3]+nxv;
      pos_f[0] = 16.0;
      pos_f[1] = 17.0;
      pos_f[2] = 18.0;

      // 'vector_slam_gyro_data:288' for i=1:map.size
      i = static_cast<int>(map->size);
      for (b_i = 0; b_i < i; b_i++) {
        // 'vector_slam_gyro_data:289' pos_f = pos_f+3;
        pos_f[0] += 3.0;
        pos_f[1] += 3.0;
        pos_f[2] += 3.0;
      }

      // 'vector_slam_gyro_data:291' for i=1:npoints
      b_i = 0;
      exitg1 = false;
      while ((!exitg1) && (b_i <= static_cast<int>(npoints) - 1)) {
        double eb_meas_idx_0;
        double ceil_coord_x;
        double ceil_coord_y;

        // Augment map
        // 'vector_slam_gyro_data:293' eb_meas  = [
        // 'vector_slam_gyro_data:294'             sqrt(1-(points(i,1)^2+points(i,2)^2)) 
        // 'vector_slam_gyro_data:295'             points(i,1)
        // 'vector_slam_gyro_data:296'             points(i,2)
        // 'vector_slam_gyro_data:297'             ];
        nadd = points[b_i + 100];
        eb_meas_idx_0 = std::sqrt(1.0 - (points[b_i] * points[b_i] + nadd * nadd));

        // 'vector_slam_gyro_data:298' en_meas = Cbn*eb_meas;
        for (i = 0; i < 3; i++) {
          en_meas[i] = (Cbn[i] * eb_meas_idx_0 + Cbn[i + 3] * points[b_i]) +
            Cbn[i + 6] * nadd;
        }

        // 'vector_slam_gyro_data:299' xx = en_meas(1);
        // 'vector_slam_gyro_data:300' yy = en_meas(2);
        // 'vector_slam_gyro_data:301' zz = en_meas(3);
        // 'vector_slam_gyro_data:302' ceil_coord_x = ceil((pi + atan2(zz, xx)) * 180/ pi / OCCUPANCY_GRID_STEP); 
        ceil_coord_x = std::ceil((rt_atan2d_snf(en_meas[2], en_meas[0]) +
          3.1415926535897931) * 180.0 / 3.1415926535897931 / 10.0);

        // 'vector_slam_gyro_data:303' ceil_coord_y = ceil((pi + atan2(yy, xx)) * 180 / pi / OCCUPANCY_GRID_STEP); 
        ceil_coord_y = std::ceil((rt_atan2d_snf(en_meas[1], en_meas[0]) +
          3.1415926535897931) * 180.0 / 3.1415926535897931 / 10.0);

        // 'vector_slam_gyro_data:304' if occupancy_map(ceil_coord_x, ceil_coord_y) <1 
        i = (static_cast<int>(ceil_coord_x) + 36 * (static_cast<int>
              (ceil_coord_y) - 1)) - 1;
        if (occupancy_map[i] < 1.0) {
          int i2;

          //         if 2 > 1
          // 'vector_slam_gyro_data:306' map.size = map.size + 1;
          map->size++;

          // 'vector_slam_gyro_data:307' map.data(map.size, 1).ceil_coord(1) = ceil_coord_x; 
          map->data[static_cast<int>(map->size) - 1].ceil_coord[0] =
            ceil_coord_x;

          // 'vector_slam_gyro_data:308' map.data(map.size, 1).ceil_coord(2) = ceil_coord_y; 
          map->data[static_cast<int>(map->size) - 1].ceil_coord[1] =
            ceil_coord_y;

          // 'vector_slam_gyro_data:309' occupancy_map(ceil_coord_x, ceil_coord_y) = 1; 
          occupancy_map[i] = 1.0;

          // 'vector_slam_gyro_data:310' map.data(map.size,1).pos = pos_f;
          map->data[static_cast<int>(map->size) - 1].pos[0] = pos_f[0];
          map->data[static_cast<int>(map->size) - 1].pos[1] = pos_f[1];
          map->data[static_cast<int>(map->size) - 1].pos[2] = pos_f[2];

          // 'vector_slam_gyro_data:311' map.data(map.size,1).en = en_meas;
          map->data[static_cast<int>(map->size) - 1].en[0] = en_meas[0];
          map->data[static_cast<int>(map->size) - 1].en[1] = en_meas[1];
          map->data[static_cast<int>(map->size) - 1].en[2] = en_meas[2];

          // 'vector_slam_gyro_data:312' map.data(map.size,1).eb = eb_meas;
          map->data[static_cast<int>(map->size) - 1].eb[0] = eb_meas_idx_0;
          map->data[static_cast<int>(map->size) - 1].eb[1] = points[b_i];
          map->data[static_cast<int>(map->size) - 1].eb[2] = nadd;

          // 'vector_slam_gyro_data:313' map.data(map.size,1).des = descriptors(i,:); 
          for (i = 0; i < 32; i++) {
            map->data[static_cast<int>(map->size) - 1].des[i] = descriptors[b_i
              + 100 * i];
          }

          // 'vector_slam_gyro_data:314' map.data(map.size,1).mat = 0;
          map->data[static_cast<int>(map->size) - 1].mat = 0.0;

          // 'vector_slam_gyro_data:315' map.data(map.size,1).obs = 0;
          map->data[static_cast<int>(map->size) - 1].obs = 0.0;

          // 'vector_slam_gyro_data:316' map.data(map.size,1).inv = 0;
          map->data[static_cast<int>(map->size) - 1].inv = 0.0;

          // 'vector_slam_gyro_data:317' map.data(map.size,1).cnt_obs = 0;
          map->data[static_cast<int>(map->size) - 1].cnt_obs = 0.0;

          // 'vector_slam_gyro_data:318' map.data(map.size,1).cnt_mat = 0;
          map->data[static_cast<int>(map->size) - 1].cnt_mat = 0.0;

          // Augment covariance
          // 'vector_slam_gyro_data:320' Hv(1:3,1:3) = skew(Cbn*eb_meas);
          // 'skew:2' y=[
          // 'skew:3'      0   -x(3)  x(2)
          // 'skew:4'     x(3)   0   -x(1)
          // 'skew:5'    -x(2)  x(1)    0];
          Hv[0] = 0.0;
          Hv[3] = -en_meas[2];
          Hv[6] = en_meas[1];
          Hv[1] = en_meas[2];
          Hv[4] = 0.0;
          Hv[7] = -en_meas[0];
          Hv[2] = -en_meas[1];
          Hv[5] = en_meas[0];
          Hv[8] = 0.0;

          // 'vector_slam_gyro_data:321' Hz = Cbn;
          // Feature covariance
          // Psize = size(P,1);
          // PPP = zeros(Psize+3, Psize+3);
          // PPP(1:Psize, 1:Psize) = P;
          // P = PPP;
          // 'vector_slam_gyro_data:327' P(pos_f,pos_f) = Hv*P(1:nxv,1:nxv)*Hv' + Hz*R*Hz'; 
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 15; i1++) {
              nadd = 0.0;
              for (i2 = 0; i2 < 15; i2++) {
                nadd += Hv[i + 3 * i2] * P[i2 + P.size(0) * i1];
              }

              c_Hv[i + 3 * i1] = nadd;
            }

            for (i1 = 0; i1 < 3; i1++) {
              loop_ub = i + 3 * i1;
              c_Cbn[loop_ub] = (Cbn[i] * b[3 * i1] + Cbn[i + 3] * b[3 * i1 + 1])
                + Cbn[i + 6] * b[3 * i1 + 2];
              nadd = 0.0;
              for (i2 = 0; i2 < 15; i2++) {
                nadd += c_Hv[i + 3 * i2] * Hv[i1 + 3 * i2];
              }

              b_Hv[loop_ub] = nadd;
            }

            nadd = c_Cbn[i + 3];
            eb_meas_idx_0 = c_Cbn[i + 6];
            for (i1 = 0; i1 < 3; i1++) {
              b_Cbn[i + 3 * i1] = (c_Cbn[i] * Cbn[i1] + nadd * Cbn[i1 + 3]) +
                eb_meas_idx_0 * Cbn[i1 + 6];
            }
          }

          for (i = 0; i < 3; i++) {
            i1 = static_cast<int>(pos_f[i]) - 1;
            P[(static_cast<int>(pos_f[0]) + P.size(0) * i1) - 1] = b_Hv[3 * i] +
              b_Cbn[3 * i];
            i2 = 3 * i + 1;
            P[(static_cast<int>(pos_f[1]) + P.size(0) * i1) - 1] = b_Hv[i2] +
              b_Cbn[i2];
            i2 = 3 * i + 2;
            P[(static_cast<int>(pos_f[2]) + P.size(0) * i1) - 1] = b_Hv[i2] +
              b_Cbn[i2];
          }

          // Vehicle to feature cross-covariance
          // 'vector_slam_gyro_data:329' P(pos_f,1:nxv) = Hv*P(1:nxv,1:nxv);
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 15; i1++) {
              nadd = 0.0;
              for (i2 = 0; i2 < 15; i2++) {
                nadd += Hv[i + 3 * i2] * P[i2 + P.size(0) * i1];
              }

              c_Hv[i + 3 * i1] = nadd;
            }
          }

          for (i = 0; i < 15; i++) {
            P[(static_cast<int>(pos_f[0]) + P.size(0) * i) - 1] = c_Hv[3 * i];
            P[(static_cast<int>(pos_f[1]) + P.size(0) * i) - 1] = c_Hv[3 * i + 1];
            P[(static_cast<int>(pos_f[2]) + P.size(0) * i) - 1] = c_Hv[3 * i + 2];
          }

          // 'vector_slam_gyro_data:330' P(1:nxv,pos_f) = P(pos_f,1:nxv)';
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 15; i1++) {
              c_Hv[i1 + 15 * i] = P[(static_cast<int>(pos_f[i]) + P.size(0) * i1)
                - 1];
            }
          }

          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 15; i1++) {
              P[i1 + P.size(0) * (static_cast<int>(pos_f[i]) - 1)] = c_Hv[i1 +
                15 * i];
            }
          }

          // Feature to feature cross-covariance
          // 'vector_slam_gyro_data:332' for j=1:map.size-1
          i = static_cast<int>(map->size - 1.0);
          for (int j = 0; j < i; j++) {
            // 'vector_slam_gyro_data:333' idx_c = map.data(j,1).pos;
            // 'vector_slam_gyro_data:334' P(pos_f,idx_c) = Hv*P(1:nxv,idx_c);
            for (i1 = 0; i1 < 3; i1++) {
              for (i2 = 0; i2 < 15; i2++) {
                c_Hv[i2 + 15 * i1] = P[i2 + P.size(0) * (static_cast<int>
                  (map->data[j].pos[i1]) - 1)];
              }
            }

            for (i1 = 0; i1 < 3; i1++) {
              i2 = static_cast<int>(pos_f[i1]) - 1;
              for (b_loop_ub = 0; b_loop_ub < 3; b_loop_ub++) {
                loop_ub = static_cast<int>(map->data[j].pos[b_loop_ub]) - 1;
                P[i2 + P.size(0) * loop_ub] = 0.0;
                for (int i3 = 0; i3 < 15; i3++) {
                  P[i2 + P.size(0) * loop_ub] = P[i2 + P.size(0) * loop_ub] +
                    Hv[i1 + 3 * i3] * c_Hv[i3 + 15 * b_loop_ub];
                }
              }
            }

            // 'vector_slam_gyro_data:335' P(idx_c,pos_f) = P(pos_f,idx_c)';
            for (i1 = 0; i1 < 3; i1++) {
              loop_ub = static_cast<int>(pos_f[i1]) - 1;
              c_Cbn[3 * i1] = P[loop_ub + P.size(0) * (static_cast<int>
                (map->data[j].pos[0]) - 1)];
              c_Cbn[3 * i1 + 1] = P[loop_ub + P.size(0) * (static_cast<int>
                (map->data[j].pos[1]) - 1)];
              c_Cbn[3 * i1 + 2] = P[loop_ub + P.size(0) * (static_cast<int>
                (map->data[j].pos[2]) - 1)];
            }

            for (i1 = 0; i1 < 3; i1++) {
              i2 = static_cast<int>(pos_f[i1]) - 1;
              P[(static_cast<int>(map->data[j].pos[0]) + P.size(0) * i2) - 1] =
                c_Cbn[3 * i1];
              P[(static_cast<int>(map->data[j].pos[1]) + P.size(0) * i2) - 1] =
                c_Cbn[3 * i1 + 1];
              P[(static_cast<int>(map->data[j].pos[2]) + P.size(0) * i2) - 1] =
                c_Cbn[3 * i1 + 2];
            }
          }

          // 'vector_slam_gyro_data:337' pos_f = pos_f+3;
          pos_f[0] += 3.0;
          pos_f[1] += 3.0;
          pos_f[2] += 3.0;
        }

        // 'vector_slam_gyro_data:339' if map.size == MAX_MAP_SIZE
        if (map->size == 600.0) {
          exitg1 = true;
        } else {
          b_i++;
        }
      }

      // 'vector_slam_gyro_data:343' P = P(1:(nxv + 3*map.size), 1:(nxv + 3*map.size)); 
      nadd = 3.0 * map->size + 15.0;
      if (1.0 > nadd) {
        loop_ub = 0;
        b_loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(nadd);
        b_loop_ub = static_cast<int>(nadd);
      }

      PPP.set_size(loop_ub, b_loop_ub);
      for (i = 0; i < b_loop_ub; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          PPP[i1 + PPP.size(0) * i] = P[i1 + P.size(0) * i];
        }
      }

      P.set_size(PPP.size(0), PPP.size(1));
      loop_ub = PPP.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = PPP.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          P[i1 + P.size(0) * i] = PPP[i1 + PPP.size(0) * i];
        }
      }

      // 'vector_slam_gyro_data:344' map.data = map.data(1:map.size);
      if (1.0 > map->size) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(map->size);
      }

      map_data_new.set_size(loop_ub);
      for (i = 0; i < loop_ub; i++) {
        map_data_new[i] = map->data[i];
      }

      map->data.set_size(map_data_new.size(0));
      loop_ub = map_data_new.size(0);
      for (i = 0; i < loop_ub; i++) {
        map->data[i] = map_data_new[i];
      }
    }
  }

  //
  // function [x,Pnew, updated] = cholesky_update(x, P, v, R, H)
  // Arguments    : coder::array<double, 1U> &x
  //                const coder::array<double, 2U> &P
  //                const double v[3]
  //                const double R[9]
  //                const coder::array<double, 2U> &H
  //                coder::array<double, 2U> &Pnew
  //                bool *updated
  // Return Type  : void
  //
  static void cholesky_update(coder::array<double, 1U> &x, const coder::array<
    double, 2U> &P, const double v[3], const double R[9], const coder::array<
    double, 2U> &H, coder::array<double, 2U> &Pnew, bool *updated)
  {
    int boffset;
    int r1;
    coder::array<double, 2U> PHt;
    int j;
    int coffset;
    int idxAjj;
    int k;
    int i;
    int aoffset;
    double S[9];
    double bkj;
    double b_S[9];
    double a21;
    double Y[3];
    int iy;
    coder::array<double, 2U> W1;
    coder::array<double, 1U> r;

    //  Kalman - Cholesky Update
    // 'vector_slam_gyro_data:349' updated = false;
    *updated = false;

    // 'vector_slam_gyro_data:350' Psize = size(P);
    // 'vector_slam_gyro_data:351' PHt = P*H';
    boffset = P.size(0);
    r1 = P.size(1);
    PHt.set_size(P.size(0), 3);
    for (j = 0; j < 3; j++) {
      coffset = j * boffset;
      for (idxAjj = 0; idxAjj < boffset; idxAjj++) {
        PHt[coffset + idxAjj] = 0.0;
      }

      for (k = 0; k < r1; k++) {
        aoffset = k * P.size(0);
        bkj = H[k * 3 + j];
        for (idxAjj = 0; idxAjj < boffset; idxAjj++) {
          i = coffset + idxAjj;
          PHt[i] = PHt[i] + P[aoffset + idxAjj] * bkj;
        }
      }
    }

    // 'vector_slam_gyro_data:352' S = H*PHt + R;
    r1 = H.size(1);
    for (j = 0; j < 3; j++) {
      coffset = j * 3;
      boffset = j * PHt.size(0);
      S[coffset] = 0.0;
      S[coffset + 1] = 0.0;
      S[coffset + 2] = 0.0;
      for (k = 0; k < r1; k++) {
        aoffset = k * 3;
        bkj = PHt[boffset + k];
        S[coffset] += H[aoffset] * bkj;
        S[coffset + 1] += H[aoffset + 1] * bkj;
        S[coffset + 2] += H[aoffset + 2] * bkj;
      }
    }

    for (i = 0; i < 9; i++) {
      S[i] += R[i];
    }

    // 'vector_slam_gyro_data:353' S = (S + S')*0.5;
    for (i = 0; i < 3; i++) {
      b_S[3 * i] = (S[3 * i] + S[i]) * 0.5;
      boffset = 3 * i + 1;
      b_S[boffset] = (S[boffset] + S[i + 3]) * 0.5;
      boffset = 3 * i + 2;
      b_S[boffset] = (S[boffset] + S[i + 6]) * 0.5;
    }

    // Sinv = inv(S);
    // 'vector_slam_gyro_data:355' chi2 = (v'/S)*v;
    std::memcpy(&S[0], &b_S[0], 9U * sizeof(double));
    r1 = 0;
    aoffset = 1;
    coffset = 2;
    bkj = std::abs(S[0]);
    a21 = std::abs(S[1]);
    if (a21 > bkj) {
      bkj = a21;
      r1 = 1;
      aoffset = 0;
    }

    if (std::abs(S[2]) > bkj) {
      r1 = 2;
      aoffset = 1;
      coffset = 0;
    }

    b_S[aoffset] = S[aoffset] / S[r1];
    b_S[coffset] /= b_S[r1];
    b_S[aoffset + 3] -= b_S[aoffset] * b_S[r1 + 3];
    b_S[coffset + 3] -= b_S[coffset] * b_S[r1 + 3];
    b_S[aoffset + 6] -= b_S[aoffset] * b_S[r1 + 6];
    b_S[coffset + 6] -= b_S[coffset] * b_S[r1 + 6];
    if (std::abs(b_S[coffset + 3]) > std::abs(b_S[aoffset + 3])) {
      boffset = aoffset;
      aoffset = coffset;
      coffset = boffset;
    }

    b_S[coffset + 3] /= b_S[aoffset + 3];
    b_S[coffset + 6] -= b_S[coffset + 3] * b_S[aoffset + 6];
    Y[r1] = v[0] / b_S[r1];
    Y[aoffset] = v[1] - Y[r1] * b_S[r1 + 3];
    Y[coffset] = v[2] - Y[r1] * b_S[r1 + 6];
    Y[aoffset] /= b_S[aoffset + 3];
    Y[coffset] -= Y[aoffset] * b_S[aoffset + 6];
    Y[coffset] /= b_S[coffset + 6];
    Y[aoffset] -= Y[coffset] * b_S[coffset + 3];
    Y[r1] -= Y[coffset] * b_S[coffset];
    Y[r1] -= Y[aoffset] * b_S[aoffset];

    // chi2 = v'*Sinv*v;
    // 'vector_slam_gyro_data:357' Pnew = [];
    Pnew.set_size(0, 0);

    // 'vector_slam_gyro_data:358' if (chi2 < 6.25)
    if ((Y[0] * v[0] + Y[1] * v[1]) + Y[2] * v[2] < 6.25) {
      bool exitg1;

      // 'vector_slam_gyro_data:359' SChol = chol(S);
      aoffset = -2;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 3)) {
        int ix;
        coffset = j * 3;
        idxAjj = coffset + j;
        bkj = 0.0;
        if (j >= 1) {
          ix = coffset;
          iy = coffset;
          for (k = 0; k < j; k++) {
            bkj += S[ix] * S[iy];
            ix++;
            iy++;
          }
        }

        bkj = S[idxAjj] - bkj;
        if (bkj > 0.0) {
          bkj = std::sqrt(bkj);
          S[idxAjj] = bkj;
          if (j + 1 < 3) {
            int idxAjjp1;
            boffset = coffset + 4;
            idxAjjp1 = idxAjj + 4;
            if (j != 0) {
              iy = idxAjj + 3;
              i = (coffset + 3 * (1 - j)) + 4;
              for (k = boffset; k <= i; k += 3) {
                ix = coffset;
                a21 = 0.0;
                r1 = (k + j) - 1;
                for (int ia = k; ia <= r1; ia++) {
                  a21 += S[ia - 1] * S[ix];
                  ix++;
                }

                S[iy] += -a21;
                iy += 3;
              }
            }

            bkj = 1.0 / bkj;
            i = (idxAjj + 3 * (1 - j)) + 4;
            for (k = idxAjjp1; k <= i; k += 3) {
              S[k - 1] *= bkj;
            }
          }

          j++;
        } else {
          S[idxAjj] = bkj;
          aoffset = j - 1;
          exitg1 = true;
        }
      }

      if (aoffset + 2 == 0) {
        boffset = 2;
      } else {
        boffset = aoffset;
      }

      for (j = 0; j <= boffset; j++) {
        i = j + 2;
        if (i <= boffset + 1) {
          std::memset(&S[(j * 3 + i) + -1], 0, ((boffset - i) + 2) * sizeof
                      (double));
        }
      }

      // 'vector_slam_gyro_data:360' W1 = PHt / SChol;
      if (PHt.size(0) == 0) {
        W1.set_size(0, 3);
      } else {
        std::memcpy(&b_S[0], &S[0], 9U * sizeof(double));
        r1 = 0;
        aoffset = 1;
        coffset = 2;
        bkj = std::abs(S[0]);
        a21 = std::abs(S[1]);
        if (a21 > bkj) {
          bkj = a21;
          r1 = 1;
          aoffset = 0;
        }

        if (std::abs(S[2]) > bkj) {
          r1 = 2;
          aoffset = 1;
          coffset = 0;
        }

        b_S[aoffset] = S[aoffset] / S[r1];
        b_S[coffset] /= b_S[r1];
        b_S[aoffset + 3] -= b_S[aoffset] * b_S[r1 + 3];
        b_S[coffset + 3] -= b_S[coffset] * b_S[r1 + 3];
        b_S[aoffset + 6] -= b_S[aoffset] * b_S[r1 + 6];
        b_S[coffset + 6] -= b_S[coffset] * b_S[r1 + 6];
        if (std::abs(b_S[coffset + 3]) > std::abs(b_S[aoffset + 3])) {
          boffset = aoffset;
          aoffset = coffset;
          coffset = boffset;
        }

        b_S[coffset + 3] /= b_S[aoffset + 3];
        b_S[coffset + 6] -= b_S[coffset + 3] * b_S[aoffset + 6];
        boffset = PHt.size(0);
        W1.set_size(PHt.size(0), 3);
        for (k = 0; k < boffset; k++) {
          W1[k + W1.size(0) * r1] = PHt[k] / b_S[r1];
          W1[k + W1.size(0) * aoffset] = PHt[k + PHt.size(0)] - W1[k + W1.size(0)
            * r1] * b_S[r1 + 3];
          W1[k + W1.size(0) * coffset] = PHt[k + PHt.size(0) * 2] - W1[k +
            W1.size(0) * r1] * b_S[r1 + 6];
          W1[k + W1.size(0) * aoffset] = W1[k + W1.size(0) * aoffset] /
            b_S[aoffset + 3];
          W1[k + W1.size(0) * coffset] = W1[k + W1.size(0) * coffset] - W1[k +
            W1.size(0) * aoffset] * b_S[aoffset + 6];
          W1[k + W1.size(0) * coffset] = W1[k + W1.size(0) * coffset] /
            b_S[coffset + 6];
          W1[k + W1.size(0) * aoffset] = W1[k + W1.size(0) * aoffset] - W1[k +
            W1.size(0) * coffset] * b_S[coffset + 3];
          W1[k + W1.size(0) * r1] = W1[k + W1.size(0) * r1] - W1[k + W1.size(0) *
            coffset] * b_S[coffset];
          W1[k + W1.size(0) * r1] = W1[k + W1.size(0) * r1] - W1[k + W1.size(0) *
            aoffset] * b_S[aoffset];
        }
      }

      // 'vector_slam_gyro_data:361' W = W1 * inv(SChol)';
      inv(S, b_S);
      boffset = W1.size(0);
      PHt.set_size(W1.size(0), 3);
      for (j = 0; j < 3; j++) {
        coffset = j * boffset;
        for (idxAjj = 0; idxAjj < boffset; idxAjj++) {
          PHt[coffset + idxAjj] = (W1[idxAjj] * b_S[j] + W1[W1.size(0) + idxAjj]
            * b_S[j + 3]) + W1[(W1.size(0) << 1) + idxAjj] * b_S[j + 6];
        }
      }

      // 'vector_slam_gyro_data:362' x = x + W*v;
      boffset = PHt.size(0);
      r.set_size(PHt.size(0));
      for (idxAjj = 0; idxAjj < boffset; idxAjj++) {
        r[idxAjj] = (PHt[idxAjj] * v[0] + PHt[PHt.size(0) + idxAjj] * v[1]) +
          PHt[(PHt.size(0) << 1) + idxAjj] * v[2];
      }

      boffset = x.size(0);
      for (i = 0; i < boffset; i++) {
        x[i] = x[i] + r[i];
      }

      // 'vector_slam_gyro_data:363' AA = W1*W1';
      boffset = W1.size(0);
      r1 = W1.size(0);
      Pnew.set_size(W1.size(0), W1.size(0));
      for (j = 0; j < r1; j++) {
        coffset = j * boffset;
        for (idxAjj = 0; idxAjj < boffset; idxAjj++) {
          Pnew[coffset + idxAjj] = (W1[idxAjj] * W1[j] + W1[W1.size(0) + idxAjj]
            * W1[W1.size(0) + j]) + W1[(W1.size(0) << 1) + idxAjj] * W1[(W1.size
            (0) << 1) + j];
        }
      }

      // 398sec
      // 'vector_slam_gyro_data:365' Pnew = P - AA;
      boffset = P.size(0) * P.size(1);
      Pnew.set_size(P.size(0), P.size(1));
      for (i = 0; i < boffset; i++) {
        Pnew[i] = P[i] - Pnew[i];
      }

      //  was 22
      // 135
      // K = PHt * Sinv;
      // KH = -K*H + eye(Psize);
      // P = KH * P * KH' + K * R * K';
      // P = (P + P')*0.5;
      // x = x + K * v;
      // 'vector_slam_gyro_data:372' updated = true;
      *updated = true;
    }
  }

  //
  // Arguments    : double u0
  //                double u1
  // Return Type  : double
  //
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
  // Arguments    : double u0
  //                double u1
  // Return Type  : double
  //
  static double rt_powd_snf(double u0, double u1)
  {
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
      y = rtNaN;
    } else {
      double d;
      double d1;
      d = std::abs(u0);
      d1 = std::abs(u1);
      if (rtIsInf(u1)) {
        if (d == 1.0) {
          y = 1.0;
        } else if (d > 1.0) {
          if (u1 > 0.0) {
            y = rtInf;
          } else {
            y = 0.0;
          }
        } else if (u1 > 0.0) {
          y = 0.0;
        } else {
          y = rtInf;
        }
      } else if (d1 == 0.0) {
        y = 1.0;
      } else if (d1 == 1.0) {
        if (u1 > 0.0) {
          y = u0;
        } else {
          y = 1.0 / u0;
        }
      } else if (u1 == 2.0) {
        y = u0 * u0;
      } else if ((u1 == 0.5) && (u0 >= 0.0)) {
        y = std::sqrt(u0);
      } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
        y = rtNaN;
      } else {
        y = pow(u0, u1);
      }
    }

    return y;
  }

  //
  // function [q, P, bw, sw, mw, map, occupancy_map] = ...
  //     vector_slam_gyro_data(frame, frame_descriptors, frame_points, map, q, P, bw, sw, mw, dt, cam, occupancy_map, motion_flag)
  // Arguments    : const frame_struct_type *frame
  //                const coder::array<unsigned char, 2U> *frame_descriptors
  //                const coder::array<double, 2U> *frame_points
  //                map_struct_type *map
  //                double q[4]
  //                coder::array<double, 2U> *P
  //                double bw[3]
  //                double sw[3]
  //                double mw[6]
  //                double dt
  //                const caminfo_struct_type *cam
  //                double occupancy_map[1296]
  //                bool motion_flag
  // Return Type  : void
  //
  void vector_slam_gyro_data(const frame_struct_type *frame, const coder::array<unsigned
    char, 2U> &frame_descriptors, const coder::array<double, 2U> &frame_points,
    map_struct_type *map, double q[4], coder::array<double, 2U> &P, double bw[3],
    double sw[3], double mw[6], double dt, const caminfo_struct_type *cam, double
    occupancy_map[1296], bool motion_flag)
  {
    double Cnb[9];
    double scale;
    double uv_idx_0;
    double t;
    double d;
    double pos_f[3];
    double Cbn[9];
    double b_Cbn_tmp;
    double b_Cnb[9];
    unsigned char des_augm[3200];
    double pts_augm[200];
    double lambda[16];
    coder::array<double, 1U> x;
    double b_lambda[4];
    double en[3];
    double en_mes[3];
    unsigned char b_frame_descriptors[32];
    double A[225];
    coder::array<bool, 1U> delete_index;
    coder::array<double, 2U> H;
    int aoffset;
    double dv[9];
    double b_pos_f[18];
    coder::array<double, 2U> index_keep;
    struct_T b_map;
    static const double dv1[9] = { R00, 0.0, 0.0, 0.0, R00, 0.0, 0.0, 0.0, R00 };

    bool updated;
    double F[225];
    double b_F[225];
    static const double b[225] = { na00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, na00, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, na00, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nr00, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      nr00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, nr00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, ns00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ns00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ns00, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nm00, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nm00, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      nm00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, nm00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, nm00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nm00 };

    if (!isInitialized_vector_slam_gyro_data) {
      vector_slam_gyro_data_initialize();
    }

    // 'vector_slam_gyro_data:3' assert(map.size <= 600);
    //  State sizes
    // Vehicle State Size
    // 'vector_slam_gyro_data:6' nxv = 15;
    // Feature State Size
    // 'vector_slam_gyro_data:8' nxf = 3;
    // 'vector_slam_gyro_data:9' meas_noise = 3e-1;
    // 'vector_slam_gyro_data:11' if (~frame.sync)
    if (!frame->sync) {
      double absxk;
      int i;
      double lambda_idx_2;
      int vlen;
      int i1;
      int m;
      int j;

      // 'vector_slam_gyro_data:12' dthe = frame.dthe;
      //     %% Correct attitide increments  with estimated bias and scale factor 
      // 'vector_slam_gyro_data:14' E = [sw(1) mw(1) mw(2)
      // 'vector_slam_gyro_data:15'         mw(3) sw(2) mw(4)
      // 'vector_slam_gyro_data:16'         mw(5) mw(6) sw(3)];
      // 'vector_slam_gyro_data:17' dthe = (eye(3) + E) * (dthe + bw*dt);
      Cnb[0] = sw[0] + 1.0;
      Cnb[1] = mw[2];
      Cnb[2] = mw[4];
      absxk = frame->dthe[0] + bw[0] * dt;
      Cnb[3] = mw[0];
      Cnb[4] = sw[1] + 1.0;
      Cnb[5] = mw[5];
      scale = frame->dthe[1] + bw[1] * dt;
      Cnb[6] = mw[1];
      Cnb[7] = mw[3];
      Cnb[8] = sw[2] + 1.0;
      uv_idx_0 = frame->dthe[2] + bw[2] * dt;

      //     %% Attitude Quaternion Mechanization
      // Quaternion from k+1 to k body axes
      // 'vector_slam_gyro_data:21' gamma1 = dthe(1);
      // 'vector_slam_gyro_data:22' gamma2 = dthe(2);
      // 'vector_slam_gyro_data:23' gamma3 = dthe(3);
      // 'vector_slam_gyro_data:24' gamma  = sqrt(dthe'*dthe);
      t = 0.0;
      for (i = 0; i < 3; i++) {
        d = (Cnb[i] * absxk + Cnb[i + 3] * scale) + Cnb[i + 6] * uv_idx_0;
        pos_f[i] = d;
        t += d * d;
      }

      absxk = std::sqrt(t);

      // 'vector_slam_gyro_data:25' lambda0 = cos(gamma/2);
      // 'vector_slam_gyro_data:26' lambda1 = -gamma1*sin(gamma/2)/gamma;
      scale = std::sin(absxk / 2.0);

      // 'vector_slam_gyro_data:27' lambda2 = -gamma2*sin(gamma/2)/gamma;
      // 'vector_slam_gyro_data:28' lambda3 = -gamma3*sin(gamma/2)/gamma;
      // 'vector_slam_gyro_data:29' if (gamma > 1e-16)
      if (absxk > 1.0E-16) {
        // 'vector_slam_gyro_data:30' lambda = [lambda0, lambda1, lambda2, lambda3]; 
        t = std::cos(absxk / 2.0);
        uv_idx_0 = -pos_f[0] * scale / absxk;
        lambda_idx_2 = -pos_f[1] * scale / absxk;
        absxk = -pos_f[2] * scale / absxk;
      } else {
        // 'vector_slam_gyro_data:31' else
        // 'vector_slam_gyro_data:32' lambda = [1, 0, 0, 0];
        t = 1.0;
        uv_idx_0 = 0.0;
        lambda_idx_2 = 0.0;
        absxk = 0.0;
      }

      // Update q for body motion
      // 'vector_slam_gyro_data:35' q = quat_mult(lambda,q);
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
      lambda[0] = t;
      lambda[4] = -uv_idx_0;
      lambda[8] = -lambda_idx_2;
      lambda[12] = -absxk;
      lambda[1] = uv_idx_0;
      lambda[5] = t;
      lambda[9] = -absxk;
      lambda[13] = lambda_idx_2;
      lambda[2] = lambda_idx_2;
      lambda[6] = absxk;
      lambda[10] = t;
      lambda[14] = -uv_idx_0;
      lambda[3] = absxk;
      lambda[7] = -lambda_idx_2;
      lambda[11] = uv_idx_0;
      lambda[15] = t;
      for (i = 0; i < 4; i++) {
        b_lambda[i] = ((lambda[i] * q[0] + lambda[i + 4] * q[1]) + lambda[i + 8]
                       * q[2]) + lambda[i + 12] * q[3];
      }

      double Cbn_tmp;
      double obs_cnt;

      // Normalize quaternion
      // 'vector_slam_gyro_data:37' q = q/sqrt(q*q');
      q[0] = b_lambda[0];
      scale = q[0] * q[0];
      q[1] = b_lambda[1];
      scale += q[1] * q[1];
      q[2] = b_lambda[2];
      scale += q[2] * q[2];
      q[3] = b_lambda[3];
      scale += q[3] * q[3];
      scale = std::sqrt(scale);
      q[0] /= scale;
      q[1] /= scale;
      q[2] /= scale;
      q[3] /= scale;

      //  DCM
      // 'vector_slam_gyro_data:39' Cbn = quat_dcm(q);
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
      uv_idx_0 = q[0] * q[0];
      lambda_idx_2 = q[1] * q[1];
      scale = q[2] * q[2];
      absxk = q[3] * q[3];
      Cbn[0] = ((uv_idx_0 + lambda_idx_2) - scale) - absxk;
      t = q[1] * q[2];
      Cbn_tmp = q[0] * q[3];
      Cbn[3] = 2.0 * (t + Cbn_tmp);
      obs_cnt = q[1] * q[3];
      b_Cbn_tmp = q[0] * q[2];
      Cbn[6] = 2.0 * (obs_cnt - b_Cbn_tmp);
      Cbn[1] = 2.0 * (t - Cbn_tmp);
      uv_idx_0 -= lambda_idx_2;
      Cbn[4] = (uv_idx_0 + scale) - absxk;
      lambda_idx_2 = q[2] * q[3];
      t = q[0] * q[1];
      Cbn[7] = 2.0 * (lambda_idx_2 + t);
      Cbn[2] = 2.0 * (obs_cnt + b_Cbn_tmp);
      Cbn[5] = 2.0 * (lambda_idx_2 - t);
      Cbn[8] = (uv_idx_0 - scale) + absxk;

      //     %% Kalman predict
      // System dynamics matrix F and system noise covariance matrix Q
      // Continuous-time system matrix
      // 'vector_slam_gyro_data:44' A = zeros(nxv,nxv);
      std::memset(&A[0], 0, 225U * sizeof(double));

      // 'vector_slam_gyro_data:45' A(1:3,4:6) = -Cbn;
      for (i = 0; i < 3; i++) {
        vlen = 15 * (i + 3);
        A[vlen] = -Cbn[3 * i];
        A[vlen + 1] = -Cbn[3 * i + 1];
        A[vlen + 2] = -Cbn[3 * i + 2];
      }

      // 'vector_slam_gyro_data:46' A(1:3,7:9) = -Cbn*diag(dthe);
      std::memset(&b_Cnb[0], 0, 9U * sizeof(double));
      b_Cnb[0] = pos_f[0];
      b_Cnb[4] = pos_f[1];
      b_Cnb[8] = pos_f[2];
      for (i = 0; i < 9; i++) {
        Cnb[i] = -Cbn[i];
      }

      for (i = 0; i < 3; i++) {
        d = Cnb[i + 3];
        b_Cbn_tmp = Cnb[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
          A[i + 15 * (i1 + 6)] = (Cnb[i] * b_Cnb[3 * i1] + d * b_Cnb[3 * i1 + 1])
            + b_Cbn_tmp * b_Cnb[3 * i1 + 2];
        }
      }

      // 'vector_slam_gyro_data:47' G = [dthe(2), dthe(3), 0, 0, 0, 0
      // 'vector_slam_gyro_data:48'         0, 0, dthe(1), dthe(3), 0, 0
      // 'vector_slam_gyro_data:49'         0, 0, 0, 0, dthe(1), dthe(2)];
      // 'vector_slam_gyro_data:50' A(1:3,10:15) = -Cbn*G;
      for (i = 0; i < 9; i++) {
        Cbn[i] = -Cbn[i];
      }

      b_pos_f[0] = pos_f[1];
      b_pos_f[3] = pos_f[2];
      b_pos_f[6] = 0.0;
      b_pos_f[9] = 0.0;
      b_pos_f[12] = 0.0;
      b_pos_f[15] = 0.0;
      b_pos_f[1] = 0.0;
      b_pos_f[4] = 0.0;
      b_pos_f[7] = pos_f[0];
      b_pos_f[10] = pos_f[2];
      b_pos_f[13] = 0.0;
      b_pos_f[16] = 0.0;
      b_pos_f[2] = 0.0;
      b_pos_f[5] = 0.0;
      b_pos_f[8] = 0.0;
      b_pos_f[11] = 0.0;
      b_pos_f[14] = pos_f[0];
      b_pos_f[17] = pos_f[1];
      for (i = 0; i < 3; i++) {
        d = Cbn[i + 3];
        b_Cbn_tmp = Cbn[i + 6];
        for (i1 = 0; i1 < 6; i1++) {
          A[i + 15 * (i1 + 9)] = (Cbn[i] * b_pos_f[3 * i1] + d * b_pos_f[3 * i1
            + 1]) + b_Cbn_tmp * b_pos_f[3 * i1 + 2];
        }
      }

      // State transition matrix
      // 'vector_slam_gyro_data:52' F = eye(nxv)+A*dt;
      std::memset(&F[0], 0, 225U * sizeof(double));
      for (m = 0; m < 15; m++) {
        F[m + 15 * m] = 1.0;
      }

      for (i = 0; i < 225; i++) {
        F[i] += A[i] * dt;
      }

      // Attitude errors noise
      // 'vector_slam_gyro_data:54' na = 1e-4;
      // Bias noise
      // 'vector_slam_gyro_data:56' nr = 1e-4;
      // Scale noise
      // 'vector_slam_gyro_data:58' ns = 1e-7;
      // Misalignment noise
      // 'vector_slam_gyro_data:60' nm = 1e-3;
      // System noise
      // 'vector_slam_gyro_data:62' Qn = diag([
      // 'vector_slam_gyro_data:63'         na, na, na,...
      // 'vector_slam_gyro_data:64'         nr, nr, nr,...
      // 'vector_slam_gyro_data:65'         ns, ns, ns,...
      // 'vector_slam_gyro_data:66'         nm, nm, nm, nm, nm, nm]);
      // Trapezioidal integration
      // 'vector_slam_gyro_data:68' Q = dt/2*(F*Qn+Qn'*F');
      uv_idx_0 = dt / 2.0;

      // Covariance predict:
      // Vehicle
      // 'vector_slam_gyro_data:71' P(1:nxv,1:nxv) = F*P(1:nxv,1:nxv)*F'+Q;
      for (j = 0; j < 15; j++) {
        vlen = j * 15;
        for (int b_i = 0; b_i < 15; b_i++) {
          aoffset = b_i * 15;
          scale = 0.0;
          d = 0.0;
          for (m = 0; m < 15; m++) {
            int naugm;
            naugm = m * 15 + j;
            scale += b[aoffset + m] * F[naugm];
            d += F[naugm] * P[m + P.size(0) * b_i];
          }

          b_F[j + 15 * b_i] = d;
          A[vlen + b_i] = scale;
        }
      }

      for (i = 0; i < 15; i++) {
        for (i1 = 0; i1 < 15; i1++) {
          d = 0.0;
          b_Cbn_tmp = 0.0;
          for (j = 0; j < 15; j++) {
            vlen = i + 15 * j;
            d += F[vlen] * b[j + 15 * i1];
            b_Cbn_tmp += b_F[vlen] * F[i1 + 15 * j];
          }

          P[i + P.size(0) * i1] = b_Cbn_tmp + uv_idx_0 * (d + A[i + 15 * i1]);
        }
      }

      // Features
      // 'vector_slam_gyro_data:73' if (map.size > 0)
      if (map->size > 0.0) {
        // 'vector_slam_gyro_data:74' rnm = (nxv + 1):(nxv + map.size * nxf);
        scale = map->size * 3.0 + 15.0;
        if (rtIsNaN(scale)) {
          index_keep.set_size(1, 1);
          index_keep[0] = rtNaN;
        } else if (scale < 16.0) {
          index_keep.set_size(1, 0);
        } else if (rtIsInf(scale) && (16.0 == scale)) {
          index_keep.set_size(1, 1);
          index_keep[0] = rtNaN;
        } else {
          aoffset = static_cast<int>(std::floor(scale - 16.0));
          index_keep.set_size(1, (aoffset + 1));
          for (i = 0; i <= aoffset; i++) {
            index_keep[i] = static_cast<double>(i) + 16.0;
          }
        }

        // 'vector_slam_gyro_data:75' P(1:nxv, rnm) = F * P(1:nxv, rnm);
        x.set_size(index_keep.size(1));
        aoffset = index_keep.size(1);
        for (i = 0; i < aoffset; i++) {
          x[i] = index_keep[i];
        }

        index_keep.set_size(15, x.size(0));
        aoffset = x.size(0);
        for (i = 0; i < aoffset; i++) {
          for (i1 = 0; i1 < 15; i1++) {
            index_keep[i1 + 15 * i] = P[i1 + P.size(0) * (static_cast<int>(x[i])
              - 1)];
          }
        }

        mtimes(F, index_keep, H);
        aoffset = H.size(1);
        for (i = 0; i < aoffset; i++) {
          for (i1 = 0; i1 < 15; i1++) {
            P[i1 + P.size(0) * (static_cast<int>(x[i]) - 1)] = H[i1 + 15 * i];
          }
        }

        // 'vector_slam_gyro_data:76' P(rnm, 1:nxv) = P(1:nxv, rnm)';
        index_keep.set_size(x.size(0), 15);
        aoffset = x.size(0);
        for (i = 0; i < 15; i++) {
          for (i1 = 0; i1 < aoffset; i1++) {
            index_keep[i1 + index_keep.size(0) * i] = P[i + P.size(0) * (
              static_cast<int>(x[i1]) - 1)];
          }
        }

        aoffset = index_keep.size(0);
        for (i = 0; i < 15; i++) {
          for (i1 = 0; i1 < aoffset; i1++) {
            P[(static_cast<int>(x[i1]) + P.size(0) * i) - 1] = index_keep[i1 +
              index_keep.size(0) * i];
          }
        }
      }

      //     %% Measurements
    } else {
      // 'vector_slam_gyro_data:80' else
      // 'vector_slam_gyro_data:81' if motion_flag
      if (!motion_flag) {
        double absxk;
        int i;
        double lambda_idx_2;
        double Cbn_tmp;
        double obs_cnt;
        int naugm;
        int vlen;
        int i1;
        int m;

        // 'vector_slam_gyro_data:85' MAX_OBS_POINTS = 10;
        // 'vector_slam_gyro_data:86' [Cbn, Cnb] = update_init(q);
        // 'update_init:4' Cbn = quat_dcm(q);
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
        uv_idx_0 = q[0] * q[0];
        lambda_idx_2 = q[1] * q[1];
        scale = q[2] * q[2];
        absxk = q[3] * q[3];
        Cbn[0] = ((uv_idx_0 + lambda_idx_2) - scale) - absxk;
        t = q[1] * q[2];
        Cbn_tmp = q[0] * q[3];
        Cbn[3] = 2.0 * (t + Cbn_tmp);
        obs_cnt = q[1] * q[3];
        b_Cbn_tmp = q[0] * q[2];
        Cbn[6] = 2.0 * (obs_cnt - b_Cbn_tmp);
        Cbn[1] = 2.0 * (t - Cbn_tmp);
        uv_idx_0 -= lambda_idx_2;
        Cbn[4] = (uv_idx_0 + scale) - absxk;
        lambda_idx_2 = q[2] * q[3];
        t = q[0] * q[1];
        Cbn[7] = 2.0 * (lambda_idx_2 + t);
        Cbn[2] = 2.0 * (obs_cnt + b_Cbn_tmp);
        Cbn[5] = 2.0 * (lambda_idx_2 - t);
        Cbn[8] = (uv_idx_0 - scale) + absxk;

        // 'update_init:5' Cnb = double(Cbn');
        for (i = 0; i < 3; i++) {
          b_Cnb[3 * i] = Cbn[i];
          b_Cnb[3 * i + 1] = Cbn[i + 3];
          b_Cnb[3 * i + 2] = Cbn[i + 6];
        }

        // 'vector_slam_gyro_data:87' des_augm = uint8(zeros(100,32));
        std::memset(&des_augm[0], 0, 3200U * sizeof(unsigned char));

        // 'vector_slam_gyro_data:88' pts_augm = zeros(100,2);
        std::memset(&pts_augm[0], 0, 200U * sizeof(double));

        // 'vector_slam_gyro_data:89' naugm = 0;
        naugm = 0;

        // 'vector_slam_gyro_data:90' nmatch = 0;
        // 'vector_slam_gyro_data:91' points = frame_points;
        // 'vector_slam_gyro_data:92' descriptors = frame_descriptors;
        // 'vector_slam_gyro_data:93' npoints = size(points,1);
        // 'vector_slam_gyro_data:94' x = zeros(nxv+map.size*nxf,1);
        vlen = static_cast<int>(map->size * 3.0 + 15.0);
        x.set_size(vlen);
        for (i = 0; i < vlen; i++) {
          x[i] = 0.0;
        }

        // 'vector_slam_gyro_data:95' obs_cnt = 0;
        obs_cnt = 0.0;

        // 'vector_slam_gyro_data:96' for i=1:npoints
        i = frame_points.size(0);
        for (int b_i = 0; b_i < i; b_i++) {
          bool initialized;
          bool too_close;
          bool exitg1;

          // Keypoint unit vectors in b- and n-frames
          // 'vector_slam_gyro_data:98' point =double(points(i,:));
          // 'vector_slam_gyro_data:99' [eb_mes, en_mes] = calc_feature_coords(point, Cbn); 
          // 'calc_feature_coords:4' eb_mes  = double([
          // 'calc_feature_coords:5'             sqrt(1 - (point(1)^2 + point(2)^2)) 
          // 'calc_feature_coords:6'             point(1)
          // 'calc_feature_coords:7'             point(2)
          // 'calc_feature_coords:8'         ]);
          d = frame_points[b_i];
          b_Cbn_tmp = frame_points[b_i + frame_points.size(0)];
          en[0] = std::sqrt(1.0 - (d * d + b_Cbn_tmp * b_Cbn_tmp));
          en[1] = d;
          en[2] = b_Cbn_tmp;

          // 'calc_feature_coords:9' en_mes = double(Cbn*eb_mes);
          for (i1 = 0; i1 < 3; i1++) {
            en_mes[i1] = (Cbn[i1] * en[0] + Cbn[i1 + 3] * d) + Cbn[i1 + 6] *
              b_Cbn_tmp;
          }

          // Kypoint descriptor
          // 'vector_slam_gyro_data:101' des_mes = uint8(descriptors(i,:));
          // Look for the matching feature on the map
          // 'vector_slam_gyro_data:103' initialized = false;
          initialized = false;

          // 'vector_slam_gyro_data:104' too_close  = false;
          too_close = false;

          // 'vector_slam_gyro_data:108' for m=1:map.size
          m = 0;
          exitg1 = false;
          while ((!exitg1) && (m <= static_cast<int>(map->size) - 1)) {
            bool guard1 = false;
            unsigned int hamming_norm;

            // Check if the measured unit vector is not too close to the
            // one on the map
            // 'vector_slam_gyro_data:112' en_map = map.data(m,1).en;
            // 'vector_slam_gyro_data:113' too_close = check_closeness(en_map, en_mes); 
            // 'check_closeness:4' prox_threshold = single_const(5e-2);
            // 'check_closeness:5' too_close = false;
            too_close = false;

            // 'check_closeness:6' if (single(norm(en_map-en_mes)) < prox_threshold) 
            scale = 3.3121686421112381E-170;
            absxk = std::abs(map->data[m].en[0] - en_mes[0]);
            if (absxk > 3.3121686421112381E-170) {
              uv_idx_0 = 1.0;
              scale = absxk;
            } else {
              t = absxk / 3.3121686421112381E-170;
              uv_idx_0 = t * t;
            }

            absxk = std::abs(map->data[m].en[1] - en_mes[1]);
            if (absxk > scale) {
              t = scale / absxk;
              uv_idx_0 = uv_idx_0 * t * t + 1.0;
              scale = absxk;
            } else {
              t = absxk / scale;
              uv_idx_0 += t * t;
            }

            absxk = std::abs(map->data[m].en[2] - en_mes[2]);
            if (absxk > scale) {
              t = scale / absxk;
              uv_idx_0 = uv_idx_0 * t * t + 1.0;
              scale = absxk;
            } else {
              t = absxk / scale;
              uv_idx_0 += t * t;
            }

            uv_idx_0 = scale * std::sqrt(uv_idx_0);
            if (static_cast<float>(uv_idx_0) < 0.05) {
              // 'check_closeness:7' too_close(:) = true;
              too_close = true;
            }

            // Memoization
            // 'vector_slam_gyro_data:116' if (~map.data(m,1).obs) && (~map.data(m,1).inv) 
            if ((!(map->data[m].obs != 0.0)) && (!(map->data[m].inv != 0.0))) {
              // Map feature unit vectors in n- and b-frames
              // 'vector_slam_gyro_data:118' map.data(m,1).eb = rotate_en(Cnb, en_map); 
              // 'rotate_en:4' eb = Cnb * en;
              for (i1 = 0; i1 < 3; i1++) {
                map->data[m].eb[i1] = 0.0;
                map->data[m].eb[i1] = map->data[m].eb[i1] + b_Cnb[i1] *
                  map->data[m].en[0];
                map->data[m].eb[i1] = map->data[m].eb[i1] + b_Cnb[i1 + 3] *
                  map->data[m].en[1];
                map->data[m].eb[i1] = map->data[m].eb[i1] + b_Cnb[i1 + 6] *
                  map->data[m].en[2];
              }

              // 'vector_slam_gyro_data:119' eb_map = map.data(m,1).eb;
              //  !!!!
              // 'vector_slam_gyro_data:121' if (eb_map(1) > 0.75)
              if (map->data[m].eb[0] > 0.75) {
                // %%0.75 !!!!! 0.9
                // Project map feature to the camera matrix to check if
                // it fits to camera frame
                // 'vector_slam_gyro_data:124' uv = project_point([eb_map(2,1),eb_map(3,1)],cam); 
                lambda_idx_2 = map->data[m].eb[1];
                scale = map->data[m].eb[2];

                //  Project points to camera matrix
                // 'vector_slam_gyro_data:378' x_ = point(1);
                // 'vector_slam_gyro_data:379' y_ = point(2);
                // 'vector_slam_gyro_data:380' r2 = x_^2+y_^2;
                uv_idx_0 = lambda_idx_2 * lambda_idx_2;
                absxk = scale * scale;
                t = uv_idx_0 + absxk;

                // 'vector_slam_gyro_data:381' r4 = r2^2;
                // 'vector_slam_gyro_data:382' r6 = r2^3;
                // 'vector_slam_gyro_data:383' k1 = cam.kc(1);
                // 'vector_slam_gyro_data:384' k2 = cam.kc(2);
                // 'vector_slam_gyro_data:385' p1 = cam.kc(3);
                // 'vector_slam_gyro_data:386' p2 = cam.kc(4);
                // 'vector_slam_gyro_data:387' k3 = cam.kc(5);
                // 'vector_slam_gyro_data:388' x__ = x_*(1+k1*r2+k2*r4+k3*r6) + 2*p1*x_*y_+p2*(r2+2*x_^2); 
                // 'vector_slam_gyro_data:389' y__ = y_*(1+k1*r2+k2*r4+k3*r6) + p1*(r2+2*y_^2)+2*p2*x_*y_; 
                // 'vector_slam_gyro_data:390' uv = [cam.fc(1)*x__+cam.cc(1); cam.fc(2)*y__+cam.cc(2)]; 
                Cbn_tmp = ((cam->kc[0] * t + 1.0) + cam->kc[1] * (t * t)) +
                  cam->kc[4] * rt_powd_snf(t, 3.0);
                uv_idx_0 = cam->fc[0] * ((lambda_idx_2 * Cbn_tmp + 2.0 * cam->
                  kc[2] * lambda_idx_2 * scale) + cam->kc[3] * (t + 2.0 *
                  uv_idx_0)) + cam->cc[0];
                scale = cam->fc[1] * ((scale * Cbn_tmp + cam->kc[2] * (t + 2.0 *
                  absxk)) + 2.0 * cam->kc[3] * lambda_idx_2 * scale) + cam->cc[1];

                // 'vector_slam_gyro_data:125' excluded_band = 10;
                // 'vector_slam_gyro_data:126' if ( uv(1) > 0 + excluded_band && uv(1) < ... 
                // 'vector_slam_gyro_data:127'                             cam.nCols-excluded_band) && ... 
                // 'vector_slam_gyro_data:128'                             (uv(2) > 0+excluded_band && uv(2) < ... 
                // 'vector_slam_gyro_data:129'                             cam.nRows-excluded_band ) 
                if (uv_idx_0 > 10.0) {
                  hamming_norm = cam->nCols - 10U;
                  if (hamming_norm > cam->nCols) {
                    hamming_norm = 0U;
                  }

                  if ((uv_idx_0 < hamming_norm) && (scale > 10.0)) {
                    hamming_norm = cam->nRows - 10U;
                    if (hamming_norm > cam->nRows) {
                      hamming_norm = 0U;
                    }

                    if (scale < hamming_norm) {
                      // 'vector_slam_gyro_data:130' map.data(m,1).obs = 1;
                      map->data[m].obs = 1.0;

                      // 'vector_slam_gyro_data:131' map.data(m,1).uv = uv;
                      map->data[m].uv[0] = uv_idx_0;
                      map->data[m].uv[1] = scale;
                    } else {
                      // 'vector_slam_gyro_data:132' else
                      // 'vector_slam_gyro_data:133' map.data(m,1).inv = 1;
                      map->data[m].inv = 1.0;
                    }
                  } else {
                    // 'vector_slam_gyro_data:132' else
                    // 'vector_slam_gyro_data:133' map.data(m,1).inv = 1;
                    map->data[m].inv = 1.0;
                  }
                } else {
                  // 'vector_slam_gyro_data:132' else
                  // 'vector_slam_gyro_data:133' map.data(m,1).inv = 1;
                  map->data[m].inv = 1.0;
                }
              } else {
                // 'vector_slam_gyro_data:135' else
                // 'vector_slam_gyro_data:136' map.data(m,1).inv = 1;
                map->data[m].inv = 1.0;
              }
            }

            // 'vector_slam_gyro_data:140' if (map.data(m,1).obs == 1)
            guard1 = false;
            if (map->data[m].obs == 1.0) {
              // 'vector_slam_gyro_data:142' eb_map = map.data(m,1).eb;
              // Match ORB descriptors using Hamming norm
              // 'vector_slam_gyro_data:144' des_map = uint8(map.data(m,1).des); 
              // 'vector_slam_gyro_data:145' norm_threshold = 30;
              // 'vector_slam_gyro_data:146' hamming_norm = match(uint8(des_mes), uint8(des_map)); 
              for (i1 = 0; i1 < 32; i1++) {
                b_frame_descriptors[i1] = frame_descriptors[b_i +
                  frame_descriptors.size(0) * i1];
              }

              hamming_norm = match(b_frame_descriptors, map->data[m].des);

              // If descriptors match
              // 'vector_slam_gyro_data:148' if (hamming_norm < norm_threshold)
              if (hamming_norm < 30U) {
                int j;

                // 'vector_slam_gyro_data:149' pos_f = map.data(m,1).pos;
                // 'vector_slam_gyro_data:150' v = eb_mes-eb_map;
                // 'vector_slam_gyro_data:151' H = zeros(nxf, nxv+map.size*nxf); 
                i1 = static_cast<int>(map->size * 3.0 + 15.0);
                H.set_size(3, i1);
                aoffset = 3 * static_cast<int>(map->size * 3.0 + 15.0);
                for (i1 = 0; i1 < aoffset; i1++) {
                  H[i1] = 0.0;
                }

                // 'vector_slam_gyro_data:152' H(1:nxf,1:3)   = -Cnb*skew(Cbn*eb_mes); 
                for (i1 = 0; i1 < 3; i1++) {
                  pos_f[i1] = (Cbn[i1] * en[0] + Cbn[i1 + 3] * d) + Cbn[i1 + 6] *
                    b_Cbn_tmp;
                }

                // 'skew:2' y=[
                // 'skew:3'      0   -x(3)  x(2)
                // 'skew:4'     x(3)   0   -x(1)
                // 'skew:5'    -x(2)  x(1)    0];
                for (i1 = 0; i1 < 9; i1++) {
                  Cnb[i1] = -b_Cnb[i1];
                }

                dv[0] = 0.0;
                dv[3] = -pos_f[2];
                dv[6] = pos_f[1];
                dv[1] = pos_f[2];
                dv[4] = 0.0;
                dv[7] = -pos_f[0];
                dv[2] = -pos_f[1];
                dv[5] = pos_f[0];
                dv[8] = 0.0;
                for (i1 = 0; i1 < 3; i1++) {
                  absxk = Cnb[i1 + 3];
                  scale = Cnb[i1 + 6];
                  for (j = 0; j < 3; j++) {
                    H[i1 + 3 * j] = (Cnb[i1] * dv[3 * j] + absxk * dv[3 * j + 1])
                      + scale * dv[3 * j + 2];
                  }
                }

                // 'vector_slam_gyro_data:153' H(1:nxf,pos_f) =  Cnb;
                // Measurement noise covariance matrix
                // 'vector_slam_gyro_data:155' R = meas_noise*eye(nxf);
                // Kalman Update with Chi-squared check
                // 'vector_slam_gyro_data:157' [x, P, updated] = cholesky_update(x, P, v, R, H); 
                for (i1 = 0; i1 < 3; i1++) {
                  vlen = static_cast<int>(map->data[m].pos[i1]) - 1;
                  H[3 * vlen] = b_Cnb[3 * i1];
                  H[3 * vlen + 1] = b_Cnb[3 * i1 + 1];
                  H[3 * vlen + 2] = b_Cnb[3 * i1 + 2];
                  pos_f[i1] = en[i1] - map->data[m].eb[i1];
                }

                cholesky_update(x, P, pos_f, dv1, H, index_keep, &updated);
                P.set_size(index_keep.size(0), index_keep.size(1));
                aoffset = index_keep.size(1);
                for (i1 = 0; i1 < aoffset; i1++) {
                  vlen = index_keep.size(0);
                  for (j = 0; j < vlen; j++) {
                    P[j + P.size(0) * i1] = index_keep[j + index_keep.size(0) *
                      i1];
                  }
                }

                // 'vector_slam_gyro_data:158' if (updated)
                if (updated) {
                  // matching feature found
                  // 'vector_slam_gyro_data:160' nmatch = nmatch+1;
                  // 'vector_slam_gyro_data:161' initialized = true;
                  initialized = true;

                  // 'vector_slam_gyro_data:162' map.data(m,1).mat = 1;
                  map->data[m].mat = 1.0;

                  // 'vector_slam_gyro_data:163' obs_cnt = obs_cnt + 1;
                  obs_cnt++;
                  exitg1 = true;
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              // 'vector_slam_gyro_data:169' if obs_cnt > MAX_OBS_POINTS
              if (obs_cnt > 10.0) {
                exitg1 = true;
              } else {
                m++;
              }
            }
          }

          // If feature is not on the map - augment later
          // 'vector_slam_gyro_data:176' if (~initialized) && (~too_close)
          if ((!initialized) && (!too_close)) {
            // 'vector_slam_gyro_data:177' naugm = naugm+1;
            naugm++;

            // 'vector_slam_gyro_data:178' pts_augm(naugm,:) = points(i,:);
            pts_augm[naugm - 1] = frame_points[b_i];
            pts_augm[naugm + 99] = frame_points[b_i + frame_points.size(0)];

            // 'vector_slam_gyro_data:179' des_augm(naugm,:) = des_mes;
            for (i1 = 0; i1 < 32; i1++) {
              des_augm[(naugm + 100 * i1) - 1] = frame_descriptors[b_i +
                frame_descriptors.size(0) * i1];
            }
          }
        }

        //     %% Correct Attitude Quaternion
        // Error quaternion
        // 'vector_slam_gyro_data:184' qe = [sqrt(1-((x(1)/2)^2 + (x(2)/2)^2 + (x(3)/2)^2)),... 
        // 'vector_slam_gyro_data:185'         x(1)/2, x(2)/2, x(3)/2];
        uv_idx_0 = x[0] / 2.0;
        scale = x[1] / 2.0;
        absxk = x[2] / 2.0;

        // Correct esimated attitude quaternion
        // 'vector_slam_gyro_data:187' q = quat_mult(q, qe);
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
        lambda[0] = q[0];
        lambda[4] = -q[1];
        lambda[8] = -q[2];
        lambda[12] = -q[3];
        lambda[1] = q[1];
        lambda[5] = q[0];
        lambda[9] = -q[3];
        lambda[13] = q[2];
        lambda[2] = q[2];
        lambda[6] = q[3];
        lambda[10] = q[0];
        lambda[14] = -q[1];
        lambda[3] = q[3];
        lambda[7] = -q[2];
        lambda[11] = q[1];
        lambda[15] = q[0];
        b_lambda[0] = std::sqrt(1.0 - ((uv_idx_0 * uv_idx_0 + scale * scale) +
          absxk * absxk));
        b_lambda[1] = x[0] / 2.0;
        b_lambda[2] = x[1] / 2.0;
        b_lambda[3] = x[2] / 2.0;
        for (i = 0; i < 4; i++) {
          q[i] = ((lambda[i] * b_lambda[0] + lambda[i + 4] * b_lambda[1]) +
                  lambda[i + 8] * b_lambda[2]) + lambda[i + 12] * b_lambda[3];
        }

        //     %% Update gyro bias estimate
        // 'vector_slam_gyro_data:190' bw = bw+x(4:6,1);
        //     %% Update gyro scale estimate
        // 'vector_slam_gyro_data:193' sw = sw+x(7:9,1);
        bw[0] += x[3];
        sw[0] += x[6];
        bw[1] += x[4];
        sw[1] += x[7];
        bw[2] += x[5];
        sw[2] += x[8];

        //     %% Update gyro misalignment estimate
        // 'vector_slam_gyro_data:196' mw = mw+x(10:15,1);
        for (i = 0; i < 6; i++) {
          mw[i] += x[i + 9];
        }

        //     %% Correct N-frame unit vectors
        // 'vector_slam_gyro_data:199' pos_f = [1,2,3]+nxv;
        pos_f[0] = 16.0;
        pos_f[1] = 17.0;
        pos_f[2] = 18.0;

        // 'vector_slam_gyro_data:200' for m=1:map.size
        i = static_cast<int>(map->size);
        for (m = 0; m < i; m++) {
          // 'vector_slam_gyro_data:201' d_en = x(pos_f,1);
          // 'vector_slam_gyro_data:202' en = map.data(m,1).en;
          // 'vector_slam_gyro_data:203' en = en+d_en;
          // 'vector_slam_gyro_data:204' map.data(m,1).en = en./norm(en);
          scale = 3.3121686421112381E-170;
          d = map->data[m].en[0] + x[static_cast<int>(pos_f[0]) - 1];
          en[0] = d;
          absxk = std::abs(d);
          if (absxk > 3.3121686421112381E-170) {
            uv_idx_0 = 1.0;
            scale = absxk;
          } else {
            t = absxk / 3.3121686421112381E-170;
            uv_idx_0 = t * t;
          }

          d = map->data[m].en[1] + x[static_cast<int>(pos_f[1]) - 1];
          en[1] = d;
          absxk = std::abs(d);
          if (absxk > scale) {
            t = scale / absxk;
            uv_idx_0 = uv_idx_0 * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            uv_idx_0 += t * t;
          }

          d = map->data[m].en[2] + x[static_cast<int>(pos_f[2]) - 1];
          absxk = std::abs(d);
          if (absxk > scale) {
            t = scale / absxk;
            uv_idx_0 = uv_idx_0 * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            uv_idx_0 += t * t;
          }

          uv_idx_0 = scale * std::sqrt(uv_idx_0);

          // 'vector_slam_gyro_data:205' pos_f = pos_f+3;
          map->data[m].en[0] = en[0] / uv_idx_0;
          pos_f[0] += 3.0;
          map->data[m].en[1] = en[1] / uv_idx_0;
          pos_f[1] += 3.0;
          map->data[m].en[2] = d / uv_idx_0;
          pos_f[2] += 3.0;
        }

        //     %% Manage features
        // 'vector_slam_gyro_data:208' delete_index = false(map.size,1);
        i = static_cast<int>(map->size);
        delete_index.set_size(i);
        aoffset = static_cast<int>(map->size);
        for (i = 0; i < aoffset; i++) {
          delete_index[i] = false;
        }

        // 'vector_slam_gyro_data:209' for m=1:map.size
        i = static_cast<int>(map->size);
        for (m = 0; m < i; m++) {
          // 'vector_slam_gyro_data:210' map.data(m,1).cnt_obs = map.data(m,1).cnt_obs + map.data(m,1).obs; 
          map->data[m].cnt_obs = map->data[m].cnt_obs + map->data[m].obs;

          // 'vector_slam_gyro_data:211' map.data(m,1).cnt_mat = map.data(m,1).cnt_mat + map.data(m,1).mat; 
          map->data[m].cnt_mat = map->data[m].cnt_mat + map->data[m].mat;

          // 'vector_slam_gyro_data:212' obs_rate = map.data(m,1).cnt_mat/map.data(m,1).cnt_obs; 
          //  !!!!
          // 'vector_slam_gyro_data:214' obs_thr = 0.05;
          // %% 0.1
          // 'vector_slam_gyro_data:215' if (obs_rate < obs_thr)
          if (map->data[m].cnt_mat / map->data[m].cnt_obs < 0.05) {
            // 'vector_slam_gyro_data:216' delete_index(m,1) = true;
            delete_index[m] = true;

            // 'vector_slam_gyro_data:217' occupancy_map(map.data(m,1).ceil_coord(1), map.data(m,1).ceil_coord(2)) = 0; 
            occupancy_map[(static_cast<int>(map->data[m].ceil_coord[0]) + 36 * (
              static_cast<int>(map->data[m].ceil_coord[1]) - 1)) - 1] = 0.0;
          }
        }

        // 'vector_slam_gyro_data:220' if (sum(delete_index) > 0)
        vlen = delete_index.size(0);
        if (delete_index.size(0) == 0) {
          aoffset = 0;
        } else {
          aoffset = delete_index[0];
          for (m = 2; m <= vlen; m++) {
            aoffset += delete_index[m - 1];
          }
        }

        if (aoffset > 0) {
          // 'vector_slam_gyro_data:221' index_keep = zeros(map.size+5,3);
          i = static_cast<int>(map->size + 5.0);
          index_keep.set_size(i, 3);
          aoffset = i * 3;
          for (i = 0; i < aoffset; i++) {
            index_keep[i] = 0.0;
          }

          // 'vector_slam_gyro_data:222' index_keep(1,:) = [1,2,3];
          // 'vector_slam_gyro_data:223' index_keep(2,:) = [4,5,6];
          // 'vector_slam_gyro_data:224' index_keep(3,:) = [7,8,9];
          // 'vector_slam_gyro_data:225' index_keep(4,:) = [10,11,12];
          // 'vector_slam_gyro_data:226' index_keep(5,:) = [13,14,15];
          // 'vector_slam_gyro_data:227' pos_f = [1,2,3]+nxv;
          index_keep[0] = 1.0;
          index_keep[1] = 4.0;
          index_keep[2] = 7.0;
          index_keep[3] = 10.0;
          index_keep[4] = 13.0;
          pos_f[0] = 16.0;
          index_keep[index_keep.size(0)] = 2.0;
          index_keep[index_keep.size(0) + 1] = 5.0;
          index_keep[index_keep.size(0) + 2] = 8.0;
          index_keep[index_keep.size(0) + 3] = 11.0;
          index_keep[index_keep.size(0) + 4] = 14.0;
          pos_f[1] = 17.0;
          index_keep[index_keep.size(0) * 2] = 3.0;
          index_keep[index_keep.size(0) * 2 + 1] = 6.0;
          index_keep[index_keep.size(0) * 2 + 2] = 9.0;
          index_keep[index_keep.size(0) * 2 + 3] = 12.0;
          index_keep[index_keep.size(0) * 2 + 4] = 15.0;
          pos_f[2] = 18.0;

          // 'vector_slam_gyro_data:228' cnt = 0;
          vlen = -1;

          // 'vector_slam_gyro_data:229' for m=1:map.size
          i = static_cast<int>(map->size);
          for (m = 0; m < i; m++) {
            // 'vector_slam_gyro_data:230' if (~delete_index(m,1))
            if (!delete_index[m]) {
              // 'vector_slam_gyro_data:231' cnt = cnt+1;
              vlen++;

              // 'vector_slam_gyro_data:232' map.data(cnt,1) = map.data(m,1);
              map->data[vlen] = map->data[m];

              // 'vector_slam_gyro_data:233' index_keep(cnt+5,:) = map.data(cnt,1).pos; 
              index_keep[vlen + 5] = map->data[vlen].pos[0];
              index_keep[(vlen + index_keep.size(0)) + 5] = map->data[vlen].pos
                [1];
              index_keep[(vlen + index_keep.size(0) * 2) + 5] = map->data[vlen].
                pos[2];

              // 'vector_slam_gyro_data:234' map.data(cnt,1).pos = pos_f;
              // 'vector_slam_gyro_data:235' pos_f = pos_f+3;
              map->data[vlen].pos[0] = pos_f[0];
              pos_f[0] += 3.0;
              map->data[vlen].pos[1] = pos_f[1];
              pos_f[1] += 3.0;
              map->data[vlen].pos[2] = pos_f[2];
              pos_f[2] += 3.0;
            }
          }

          // 'vector_slam_gyro_data:238' map.size = cnt;
          map->size = vlen + 1;

          // 'vector_slam_gyro_data:239' index_keep = reshape(index_keep(1:(cnt+5),:)',cnt*nxf+nxv,1); 
          aoffset = vlen + 5;
          H.set_size(3, (vlen + 6));
          for (i = 0; i <= aoffset; i++) {
            H[3 * i] = index_keep[i];
            H[3 * i + 1] = index_keep[i + index_keep.size(0)];
            H[3 * i + 2] = index_keep[i + index_keep.size(0) * 2];
          }

          vlen = (vlen + 1) * 3 + 15;

          // 'vector_slam_gyro_data:240' P = P(index_keep,index_keep);
          index_keep.set_size(vlen, vlen);
          for (i = 0; i < vlen; i++) {
            for (i1 = 0; i1 < vlen; i1++) {
              index_keep[i1 + index_keep.size(0) * i] = P[(static_cast<int>(H[i1])
                + P.size(0) * (static_cast<int>(H[i]) - 1)) - 1];
            }
          }

          P.set_size(index_keep.size(0), index_keep.size(1));
          aoffset = index_keep.size(1);
          for (i = 0; i < aoffset; i++) {
            vlen = index_keep.size(0);
            for (i1 = 0; i1 < vlen; i1++) {
              P[i1 + P.size(0) * i] = index_keep[i1 + index_keep.size(0) * i];
            }
          }
        }

        //     %% Augment
        // 'vector_slam_gyro_data:243' if (naugm > 0)
        if (naugm > 0) {
          // 'vector_slam_gyro_data:244' [map, P, occupancy_map] = augment(map, P, pts_augm, des_augm, naugm, Cbn, nxv, occupancy_map); 
          b_map.size = map->size;
          b_map.data.set_size(map->data.size(0));
          aoffset = map->data.size(0);
          for (i = 0; i < aoffset; i++) {
            b_map.data[i] = map->data[i];
          }

          index_keep.set_size(P.size(0), P.size(1));
          aoffset = P.size(0) * P.size(1);
          for (i = 0; i < aoffset; i++) {
            index_keep[i] = P[i];
          }

          augment(&b_map, index_keep, pts_augm, des_augm, static_cast<double>
                  (naugm), Cbn, occupancy_map);
          map->size = b_map.size;
          map->data.set_size(b_map.data.size(0));
          aoffset = b_map.data.size(0);
          for (i = 0; i < aoffset; i++) {
            map->data[i] = b_map.data[i];
          }

          P.set_size(index_keep.size(0), index_keep.size(1));
          aoffset = index_keep.size(1);
          for (i = 0; i < aoffset; i++) {
            vlen = index_keep.size(0);
            for (i1 = 0; i1 < vlen; i1++) {
              P[i1 + P.size(0) * i] = index_keep[i1 + index_keep.size(0) * i];
            }
          }
        }
      }
    }
  }
}

//
// File trailer for vector_slam_gyro_data.cpp
//
// [EOF]
//
