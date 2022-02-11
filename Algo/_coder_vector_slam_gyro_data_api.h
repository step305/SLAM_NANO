/*
 * File: _coder_vector_slam_gyro_data_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 09-Feb-2022 13:12:06
 */

#ifndef _CODER_VECTOR_SLAM_GYRO_DATA_API_H
#define _CODER_VECTOR_SLAM_GYRO_DATA_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef typedef_struct2_T
#define typedef_struct2_T

typedef struct {
  real_T en[3];
  real_T eb[3];
  real_T pos[3];
  uint8_T des[32];
  real_T mat;
  real_T obs;
  real_T inv;
  real_T uv[2];
  real_T cnt_obs;
  real_T cnt_mat;
  real_T ceil_coord[2];
} struct2_T;

#endif                                 /*typedef_struct2_T*/

#ifndef typedef_emxArray_struct2_T
#define typedef_emxArray_struct2_T

typedef struct {
  struct2_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct2_T;

#endif                                 /*typedef_emxArray_struct2_T*/

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T

struct emxArray_uint8_T
{
  uint8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_uint8_T*/

#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T

typedef struct emxArray_uint8_T emxArray_uint8_T;

#endif                                 /*typedef_emxArray_uint8_T*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  boolean_T sync;
  real_T dthe[3];
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_struct3_T
#define typedef_struct3_T

typedef struct {
  uint32_T nRows;
  uint32_T nCols;
  real_T fc[2];
  real_T cc[2];
  real_T kc[5];
  real_T frame_rate;
} struct3_T;

#endif                                 /*typedef_struct3_T*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  real_T size;
  emxArray_struct2_T *data;
} struct1_T;

#endif                                 /*typedef_struct1_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void vector_slam_gyro_data(struct0_T *frame, emxArray_uint8_T
  *frame_descriptors, emxArray_real_T *frame_points, struct1_T *map, real_T q[4],
  emxArray_real_T *P, real_T bw[3], real_T sw[3], real_T mw[6], real_T dt,
  struct3_T *cam, real_T occupancy_map[8100], boolean_T motion_flag);
extern void vector_slam_gyro_data_api(const mxArray * const prhs[13], int32_T
  nlhs, const mxArray *plhs[7]);
extern void vector_slam_gyro_data_atexit(void);
extern void vector_slam_gyro_data_initialize(void);
extern void vector_slam_gyro_data_terminate(void);
extern void vector_slam_gyro_data_xil_shutdown(void);
extern void vector_slam_gyro_data_xil_terminate(void);

#endif

/*
 * File trailer for _coder_vector_slam_gyro_data_api.h
 *
 * [EOF]
 */
