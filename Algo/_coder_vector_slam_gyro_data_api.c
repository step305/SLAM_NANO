/*
 * File: _coder_vector_slam_gyro_data_api.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 09-Feb-2022 13:12:06
 */

/* Include Files */
#include "_coder_vector_slam_gyro_data_api.h"
#include "_coder_vector_slam_gyro_data_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131594U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "vector_slam_gyro_data",             /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct3_T *y);
static struct0_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static void b_emlrt_marshallOut(const emxArray_real_T *u, const mxArray *y);
static uint32_T bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static boolean_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static void c_emlrt_marshallOut(const real_T u[3], const mxArray *y);
static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[2]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3]);
static void d_emlrt_marshallOut(const real_T u[6], const mxArray *y);
static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[5]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *frame_descriptors, const char_T *identifier, emxArray_uint8_T *y);
static const mxArray *e_emlrt_marshallOut(const struct1_T u);
static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *occupancy_map, const char_T *identifier))[8100];
static struct0_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *frame,
  const char_T *identifier);
static void emlrt_marshallOut(const real_T u[4], const mxArray *y);
static void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel);
static void emxEnsureCapacity_struct2_T(emxArray_struct2_T *emxArray, int32_T
  oldNumel);
static void emxEnsureCapacity_uint8_T(emxArray_uint8_T *emxArray, int32_T
  oldNumel);
static void emxFreeStruct_struct1_T(struct1_T *pStruct);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxFree_struct2_T(emxArray_struct2_T **pEmxArray);
static void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);
static void emxInitStruct_struct1_T(const emlrtStack *sp, struct1_T *pStruct,
  boolean_T doPush);
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush);
static void emxInit_struct2_T(const emlrtStack *sp, emxArray_struct2_T
  **pEmxArray, int32_T numDimensions, boolean_T doPush);
static void emxInit_uint8_T(const emlrtStack *sp, emxArray_uint8_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_uint8_T *y);
static void f_emlrt_marshallOut(const real_T u[8100], const mxArray *y);
static real_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[8100];
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *frame_points,
  const char_T *identifier, emxArray_real_T *y);
static boolean_T gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *motion_flag, const char_T *identifier);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static boolean_T hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *map, const
  char_T *identifier, struct1_T *y);
static void ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y);
static void jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_uint8_T *ret);
static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_struct2_T *y);
static real_T lb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3]);
static void mb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3]);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, uint8_T y[32]);
static void nb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, uint8_T ret[32]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[2]);
static void ob_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[2]);
static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char_T *identifier))[4];
static real_T (*pb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static void qb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P, const
  char_T *identifier, emxArray_real_T *y);
static real_T (*rb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static real_T (*sb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];
static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *bw,
  const char_T *identifier))[3];
static uint32_T tb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static void ub_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[2]);
static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *mw,
  const char_T *identifier))[6];
static void vb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[5]);
static real_T (*w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static real_T (*wb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8100];
static real_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char_T *identifier);
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cam, const
  char_T *identifier, struct3_T *y);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct3_T *y
 * Return Type  : void
 */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct3_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[6] = { "nRows", "nCols", "fc", "cc", "kc",
    "frame_rate" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 6, fieldNames, 0U, &dims);
  thisId.fIdentifier = "nRows";
  y->nRows = bb_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "nRows")), &thisId);
  thisId.fIdentifier = "nCols";
  y->nCols = bb_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "nCols")), &thisId);
  thisId.fIdentifier = "fc";
  cb_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "fc")),
                      &thisId, y->fc);
  thisId.fIdentifier = "cc";
  cb_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "cc")),
                      &thisId, y->cc);
  thisId.fIdentifier = "kc";
  db_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4, "kc")),
                      &thisId, y->kc);
  thisId.fIdentifier = "frame_rate";
  y->frame_rate = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    5, "frame_rate")), &thisId);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : struct0_T
 */
static struct0_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  struct0_T y;
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[2] = { "sync", "dthe" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 2, fieldNames, 0U, &dims);
  thisId.fIdentifier = "sync";
  y.sync = c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "sync")), &thisId);
  thisId.fIdentifier = "dthe";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "dthe")),
                     &thisId, y.dthe);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emxArray_real_T *u
 *                const mxArray *y
 * Return Type  : void
 */
static void b_emlrt_marshallOut(const emxArray_real_T *u, const mxArray *y)
{
  emlrtMxSetData((mxArray *)y, &u->data[0]);
  emlrtSetDimensions((mxArray *)y, u->size, 2);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint32_T
 */
static uint32_T bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  uint32_T y;
  y = tb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : boolean_T
 */
static boolean_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = hb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u[3]
 *                const mxArray *y
 * Return Type  : void
 */
static void c_emlrt_marshallOut(const real_T u[3], const mxArray *y)
{
  static const int32_T iv[1] = { 3 };

  emlrtMxSetData((mxArray *)y, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)y, *(int32_T (*)[1])&iv[0], 1);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[2]
 * Return Type  : void
 */
static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[2])
{
  ub_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[3]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3])
{
  ib_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[6]
 *                const mxArray *y
 * Return Type  : void
 */
static void d_emlrt_marshallOut(const real_T u[6], const mxArray *y)
{
  static const int32_T iv[1] = { 6 };

  emlrtMxSetData((mxArray *)y, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)y, *(int32_T (*)[1])&iv[0], 1);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[5]
 * Return Type  : void
 */
static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[5])
{
  vb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *frame_descriptors
 *                const char_T *identifier
 *                emxArray_uint8_T *y
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *frame_descriptors, const char_T *identifier, emxArray_uint8_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(frame_descriptors), &thisId, y);
  emlrtDestroyArray(&frame_descriptors);
}

/*
 * Arguments    : const struct1_T u
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const struct1_T u)
{
  const mxArray *y;
  static const char * sv[2] = { "size", "data" };

  const mxArray *b_y;
  const mxArray *m;
  const emxArray_struct2_T *b_u;
  int32_T iv[1];
  static const char * sv1[11] = { "en", "eb", "pos", "des", "mat", "obs", "inv",
    "uv", "cnt_obs", "cnt_mat", "ceil_coord" };

  int32_T i;
  int32_T b_j0;
  const mxArray *c_y;
  static const int32_T iv1[1] = { 3 };

  real_T *pData;
  static const int32_T iv2[1] = { 3 };

  static const int32_T iv3[2] = { 1, 3 };

  static const int32_T iv4[2] = { 1, 32 };

  uint8_T *b_pData;
  int32_T b_i;
  int32_T c_i;
  static const int32_T iv5[1] = { 2 };

  static const int32_T iv6[1] = { 2 };

  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 2, sv));
  b_y = NULL;
  m = emlrtCreateDoubleScalar(u.size);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "size", b_y, 0);
  b_u = u.data;
  b_y = NULL;
  iv[0] = u.data->size[0];
  emlrtAssign(&b_y, emlrtCreateStructArray(1, iv, 11, sv1));
  emlrtCreateField(b_y, "en");
  emlrtCreateField(b_y, "eb");
  emlrtCreateField(b_y, "pos");
  emlrtCreateField(b_y, "des");
  emlrtCreateField(b_y, "mat");
  emlrtCreateField(b_y, "obs");
  emlrtCreateField(b_y, "inv");
  emlrtCreateField(b_y, "uv");
  emlrtCreateField(b_y, "cnt_obs");
  emlrtCreateField(b_y, "cnt_mat");
  emlrtCreateField(b_y, "ceil_coord");
  i = 0;
  for (b_j0 = 0; b_j0 < b_u->size[0U]; b_j0++) {
    c_y = NULL;
    m = emlrtCreateNumericArray(1, &iv1[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = b_u->data[b_j0].en[0];
    pData[1] = b_u->data[b_j0].en[1];
    pData[2] = b_u->data[b_j0].en[2];
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "en", c_y, 0);
    c_y = NULL;
    m = emlrtCreateNumericArray(1, &iv2[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = b_u->data[b_j0].eb[0];
    pData[1] = b_u->data[b_j0].eb[1];
    pData[2] = b_u->data[b_j0].eb[2];
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "eb", c_y, 1);
    c_y = NULL;
    m = emlrtCreateNumericArray(2, &iv3[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = b_u->data[b_j0].pos[0];
    pData[1] = b_u->data[b_j0].pos[1];
    pData[2] = b_u->data[b_j0].pos[2];
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "pos", c_y, 2);
    c_y = NULL;
    m = emlrtCreateNumericArray(2, &iv4[0], mxUINT8_CLASS, mxREAL);
    b_pData = (uint8_T *)emlrtMxGetData(m);
    b_i = 0;
    for (c_i = 0; c_i < 32; c_i++) {
      b_pData[b_i] = b_u->data[b_j0].des[c_i];
      b_i++;
    }

    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "des", c_y, 3);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(b_u->data[b_j0].mat);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "mat", c_y, 4);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(b_u->data[b_j0].obs);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "obs", c_y, 5);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(b_u->data[b_j0].inv);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "inv", c_y, 6);
    c_y = NULL;
    m = emlrtCreateNumericArray(1, &iv5[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = b_u->data[b_j0].uv[0];
    pData[1] = b_u->data[b_j0].uv[1];
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "uv", c_y, 7);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(b_u->data[b_j0].cnt_obs);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "cnt_obs", c_y, 8);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(b_u->data[b_j0].cnt_mat);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "cnt_mat", c_y, 9);
    c_y = NULL;
    m = emlrtCreateNumericArray(1, &iv6[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = b_u->data[b_j0].ceil_coord[0];
    pData[1] = b_u->data[b_j0].ceil_coord[1];
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(b_y, i, "ceil_coord", c_y, 10);
    i++;
  }

  emlrtSetFieldR2017b(y, 0, "data", b_y, 1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *occupancy_map
 *                const char_T *identifier
 * Return Type  : real_T (*)[8100]
 */
static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *occupancy_map, const char_T *identifier))[8100]
{
  real_T (*y)[8100];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = fb_emlrt_marshallIn(sp, emlrtAlias(occupancy_map), &thisId);
  emlrtDestroyArray(&occupancy_map);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *frame
 *                const char_T *identifier
 * Return Type  : struct0_T
 */
  static struct0_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *frame,
  const char_T *identifier)
{
  struct0_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(frame), &thisId);
  emlrtDestroyArray(&frame);
  return y;
}

/*
 * Arguments    : const real_T u[4]
 *                const mxArray *y
 * Return Type  : void
 */
static void emlrt_marshallOut(const real_T u[4], const mxArray *y)
{
  static const int32_T iv[2] = { 1, 4 };

  emlrtMxSetData((mxArray *)y, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)y, *(int32_T (*)[2])&iv[0], 2);
}

/*
 * Arguments    : emxArray_real_T *emxArray
 *                int32_T oldNumel
 * Return Type  : void
 */
static void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_struct2_T *emxArray
 *                int32_T oldNumel
 * Return Type  : void
 */
static void emxEnsureCapacity_struct2_T(emxArray_struct2_T *emxArray, int32_T
  oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(struct2_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(struct2_T) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (struct2_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_uint8_T *emxArray
 *                int32_T oldNumel
 * Return Type  : void
 */
static void emxEnsureCapacity_uint8_T(emxArray_uint8_T *emxArray, int32_T
  oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(uint8_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(uint8_T) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (uint8_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : struct1_T *pStruct
 * Return Type  : void
 */
static void emxFreeStruct_struct1_T(struct1_T *pStruct)
{
  emxFree_struct2_T(&pStruct->data);
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_struct2_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_struct2_T(emxArray_struct2_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_struct2_T *)NULL) {
    if (((*pEmxArray)->data != (struct2_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_struct2_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_uint8_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_uint8_T(emxArray_uint8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint8_T *)NULL) {
    if (((*pEmxArray)->data != (uint8_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_uint8_T *)NULL;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                struct1_T *pStruct
 *                boolean_T doPush
 * Return Type  : void
 */
static void emxInitStruct_struct1_T(const emlrtStack *sp, struct1_T *pStruct,
  boolean_T doPush)
{
  emxInit_struct2_T(sp, &pStruct->data, 1, doPush);
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_real_T **pEmxArray
 *                int32_T numDimensions
 *                boolean_T doPush
 * Return Type  : void
 */
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocMex(sizeof(emxArray_real_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void *)
      &emxFree_real_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_struct2_T **pEmxArray
 *                int32_T numDimensions
 *                boolean_T doPush
 * Return Type  : void
 */
static void emxInit_struct2_T(const emlrtStack *sp, emxArray_struct2_T
  **pEmxArray, int32_T numDimensions, boolean_T doPush)
{
  emxArray_struct2_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_struct2_T *)emlrtMallocMex(sizeof(emxArray_struct2_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void *)
      &emxFree_struct2_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (struct2_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_uint8_T **pEmxArray
 *                int32_T numDimensions
 *                boolean_T doPush
 * Return Type  : void
 */
static void emxInit_uint8_T(const emlrtStack *sp, emxArray_uint8_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush)
{
  emxArray_uint8_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_uint8_T *)emlrtMallocMex(sizeof(emxArray_uint8_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void *)
      &emxFree_uint8_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (uint8_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                emxArray_uint8_T *y
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_uint8_T *y)
{
  jb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[8100]
 *                const mxArray *y
 * Return Type  : void
 */
static void f_emlrt_marshallOut(const real_T u[8100], const mxArray *y)
{
  static const int32_T iv[2] = { 90, 90 };

  emlrtMxSetData((mxArray *)y, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)y, *(int32_T (*)[2])&iv[0], 2);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[8100]
 */
static real_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[8100]
{
  real_T (*y)[8100];
  y = wb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *frame_points
 *                const char_T *identifier
 *                emxArray_real_T *y
 * Return Type  : void
 */
  static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *frame_points, const char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  h_emlrt_marshallIn(sp, emlrtAlias(frame_points), &thisId, y);
  emlrtDestroyArray(&frame_points);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *motion_flag
 *                const char_T *identifier
 * Return Type  : boolean_T
 */
static boolean_T gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *motion_flag, const char_T *identifier)
{
  boolean_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(motion_flag), &thisId);
  emlrtDestroyArray(&motion_flag);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  kb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : boolean_T
 */
static boolean_T hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "logical", false, 0U, &dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *map
 *                const char_T *identifier
 *                struct1_T *y
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *map, const
  char_T *identifier, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  j_emlrt_marshallIn(sp, emlrtAlias(map), &thisId, y);
  emlrtDestroyArray(&map);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[3]
 * Return Type  : void
 */
static void ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  real_T (*r)[3];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct1_T *y
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[2] = { "size", "data" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 2, fieldNames, 0U, &dims);
  thisId.fIdentifier = "size";
  y->size = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "size")), &thisId);
  thisId.fIdentifier = "data";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "data")),
                     &thisId, y->data);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                emxArray_uint8_T *ret
 * Return Type  : void
 */
static void jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_uint8_T *ret)
{
  static const int32_T dims[2] = { 200, 32 };

  const boolean_T bv[2] = { true, false };

  int32_T iv[2];
  int32_T i;
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "uint8", false, 2U, dims, &bv[0], iv);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_uint8_T(ret, i);
  ret->data = (uint8_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = lb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                emxArray_real_T *ret
 * Return Type  : void
 */
static void kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  static const int32_T dims[2] = { 200, 2 };

  const boolean_T bv[2] = { true, false };

  int32_T iv[2];
  int32_T i;
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims, &bv[0],
    iv);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                emxArray_struct2_T *y
 * Return Type  : void
 */
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_struct2_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[11] = { "en", "eb", "pos", "des", "mat", "obs",
    "inv", "uv", "cnt_obs", "cnt_mat", "ceil_coord" };

  static const int32_T dims[1] = { 600 };

  const boolean_T bv[1] = { true };

  int32_T sizes[1];
  int32_T i;
  int32_T n;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckVsStructR2012b(sp, parentId, u, 11, fieldNames, 1U, dims, &bv[0],
    sizes);
  i = y->size[0];
  y->size[0] = sizes[0];
  emxEnsureCapacity_struct2_T(y, i);
  n = sizes[0];
  for (i = 0; i < n; i++) {
    thisId.fIdentifier = "en";
    d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 0, "en")),
                       &thisId, y->data[i].en);
    thisId.fIdentifier = "eb";
    d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 1, "eb")),
                       &thisId, y->data[i].eb);
    thisId.fIdentifier = "pos";
    m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 2, "pos")),
                       &thisId, y->data[i].pos);
    thisId.fIdentifier = "des";
    n_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 3, "des")),
                       &thisId, y->data[i].des);
    thisId.fIdentifier = "mat";
    y->data[i].mat = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 4, "mat")), &thisId);
    thisId.fIdentifier = "obs";
    y->data[i].obs = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 5, "obs")), &thisId);
    thisId.fIdentifier = "inv";
    y->data[i].inv = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 6, "inv")), &thisId);
    thisId.fIdentifier = "uv";
    o_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 7, "uv")),
                       &thisId, y->data[i].uv);
    thisId.fIdentifier = "cnt_obs";
    y->data[i].cnt_obs = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
      (sp, u, i, 8, "cnt_obs")), &thisId);
    thisId.fIdentifier = "cnt_mat";
    y->data[i].cnt_mat = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
      (sp, u, i, 9, "cnt_mat")), &thisId);
    thisId.fIdentifier = "ceil_coord";
    o_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 10,
      "ceil_coord")), &thisId, y->data[i].ceil_coord);
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T lb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[3]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3])
{
  mb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[3]
 * Return Type  : void
 */
static void mb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[2] = { 1, 3 };

  real_T (*r)[3];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                uint8_T y[32]
 * Return Type  : void
 */
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, uint8_T y[32])
{
  nb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                uint8_T ret[32]
 * Return Type  : void
 */
static void nb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, uint8_T ret[32])
{
  static const int32_T dims[2] = { 1, 32 };

  uint8_T (*r)[32];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 2U, dims);
  r = (uint8_T (*)[32])emlrtMxGetData(src);
  for (i = 0; i < 32; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[2]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[2])
{
  ob_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[2]
 * Return Type  : void
 */
static void ob_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[2])
{
  static const int32_T dims[1] = { 2 };

  real_T (*r)[2];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[2])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *q
 *                const char_T *identifier
 * Return Type  : real_T (*)[4]
 */
static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char_T *identifier))[4]
{
  real_T (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = q_emlrt_marshallIn(sp, emlrtAlias(q), &thisId);
  emlrtDestroyArray(&q);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[4]
 */
  static real_T (*pb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  real_T (*ret)[4];
  static const int32_T dims[2] = { 1, 4 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[4]
 */
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4]
{
  real_T (*y)[4];
  y = pb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                emxArray_real_T *ret
 * Return Type  : void
 */
  static void qb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  static const int32_T dims[2] = { 1815, 1815 };

  const boolean_T bv[2] = { true, true };

  int32_T iv[2];
  int32_T i;
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims, &bv[0],
    iv);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *P
 *                const char_T *identifier
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P, const
  char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  s_emlrt_marshallIn(sp, emlrtAlias(P), &thisId, y);
  emlrtDestroyArray(&P);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
static real_T (*rb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  real_T (*ret)[3];
  static const int32_T dims[1] = { 3 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                emxArray_real_T *y
 * Return Type  : void
 */
  static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  qb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[6]
 */
static real_T (*sb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  static const int32_T dims[1] = { 6 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *bw
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
  static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *bw,
  const char_T *identifier))[3]
{
  real_T (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = u_emlrt_marshallIn(sp, emlrtAlias(bw), &thisId);
  emlrtDestroyArray(&bw);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint32_T
 */
static uint32_T tb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  uint32_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint32", false, 0U, &dims);
  ret = *(uint32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = rb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[2]
 * Return Type  : void
 */
  static void ub_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId, real_T ret[2])
{
  static const int32_T dims[2] = { 1, 2 };

  real_T (*r)[2];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[2])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *mw
 *                const char_T *identifier
 * Return Type  : real_T (*)[6]
 */
static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *mw,
  const char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = w_emlrt_marshallIn(sp, emlrtAlias(mw), &thisId);
  emlrtDestroyArray(&mw);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[5]
 * Return Type  : void
 */
  static void vb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId, real_T ret[5])
{
  static const int32_T dims[2] = { 1, 5 };

  real_T (*r)[5];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[5])emlrtMxGetData(src);
  for (i = 0; i < 5; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[6]
 */
static real_T (*w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = sb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[8100]
 */
  static real_T (*wb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8100]
{
  real_T (*ret)[8100];
  static const int32_T dims[2] = { 90, 90 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[8100])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *dt
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = k_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *cam
 *                const char_T *identifier
 *                struct3_T *y
 * Return Type  : void
 */
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cam, const
  char_T *identifier, struct3_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  ab_emlrt_marshallIn(sp, emlrtAlias(cam), &thisId, y);
  emlrtDestroyArray(&cam);
}

/*
 * Arguments    : const mxArray * const prhs[13]
 *                int32_T nlhs
 *                const mxArray *plhs[7]
 * Return Type  : void
 */
void vector_slam_gyro_data_api(const mxArray * const prhs[13], int32_T nlhs,
  const mxArray *plhs[7])
{
  emxArray_uint8_T *frame_descriptors;
  emxArray_real_T *frame_points;
  struct1_T map;
  emxArray_real_T *P;
  const mxArray *prhs_copy_idx_0;
  const mxArray *prhs_copy_idx_1;
  const mxArray *prhs_copy_idx_2;
  const mxArray *prhs_copy_idx_3;
  const mxArray *prhs_copy_idx_4;
  const mxArray *prhs_copy_idx_5;
  const mxArray *prhs_copy_idx_6;
  const mxArray *prhs_copy_idx_7;
  const mxArray *prhs_copy_idx_8;
  const mxArray *prhs_copy_idx_9;
  const mxArray *prhs_copy_idx_10;
  const mxArray *prhs_copy_idx_11;
  struct0_T frame;
  real_T (*q)[4];
  real_T (*bw)[3];
  real_T (*sw)[3];
  real_T (*mw)[6];
  real_T dt;
  struct3_T cam;
  real_T (*occupancy_map)[8100];
  boolean_T motion_flag;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_uint8_T(&st, &frame_descriptors, 2, true);
  emxInit_real_T(&st, &frame_points, 2, true);
  emxInitStruct_struct1_T(&st, &map, true);
  emxInit_real_T(&st, &P, 2, true);
  prhs_copy_idx_0 = prhs[0];
  prhs_copy_idx_1 = prhs[1];
  prhs_copy_idx_2 = prhs[2];
  prhs_copy_idx_3 = prhs[3];
  prhs_copy_idx_4 = emlrtProtectR2012b(prhs[4], 4, true, -1);
  prhs_copy_idx_5 = emlrtProtectR2012b(prhs[5], 5, true, -1);
  prhs_copy_idx_6 = emlrtProtectR2012b(prhs[6], 6, true, -1);
  prhs_copy_idx_7 = emlrtProtectR2012b(prhs[7], 7, true, -1);
  prhs_copy_idx_8 = emlrtProtectR2012b(prhs[8], 8, true, -1);
  prhs_copy_idx_9 = prhs[9];
  prhs_copy_idx_10 = prhs[10];
  prhs_copy_idx_11 = emlrtProtectR2012b(prhs[11], 11, true, -1);

  /* Marshall function inputs */
  frame = emlrt_marshallIn(&st, emlrtAliasP(prhs_copy_idx_0), "frame");
  frame_descriptors->canFreeData = false;
  e_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_1), "frame_descriptors",
                     frame_descriptors);
  frame_points->canFreeData = false;
  g_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_2), "frame_points",
                     frame_points);
  i_emlrt_marshallIn(&st, emlrtAliasP(prhs_copy_idx_3), "map", &map);
  q = p_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_4), "q");
  P->canFreeData = false;
  r_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_5), "P", P);
  bw = t_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_6), "bw");
  sw = t_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_7), "sw");
  mw = v_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_8), "mw");
  dt = x_emlrt_marshallIn(&st, emlrtAliasP(prhs_copy_idx_9), "dt");
  y_emlrt_marshallIn(&st, emlrtAliasP(prhs_copy_idx_10), "cam", &cam);
  occupancy_map = eb_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_11),
    "occupancy_map");
  motion_flag = gb_emlrt_marshallIn(&st, emlrtAliasP(prhs[12]), "motion_flag");

  /* Invoke the target function */
  vector_slam_gyro_data(&frame, frame_descriptors, frame_points, &map, *q, P,
                        *bw, *sw, *mw, dt, &cam, *occupancy_map, motion_flag);

  /* Marshall function outputs */
  emlrt_marshallOut(*q, prhs_copy_idx_4);
  plhs[0] = prhs_copy_idx_4;
  emxFree_real_T(&frame_points);
  emxFree_uint8_T(&frame_descriptors);
  if (nlhs > 1) {
    P->canFreeData = false;
    b_emlrt_marshallOut(P, prhs_copy_idx_5);
    plhs[1] = prhs_copy_idx_5;
  }

  emxFree_real_T(&P);
  if (nlhs > 2) {
    c_emlrt_marshallOut(*bw, prhs_copy_idx_6);
    plhs[2] = prhs_copy_idx_6;
  }

  if (nlhs > 3) {
    c_emlrt_marshallOut(*sw, prhs_copy_idx_7);
    plhs[3] = prhs_copy_idx_7;
  }

  if (nlhs > 4) {
    d_emlrt_marshallOut(*mw, prhs_copy_idx_8);
    plhs[4] = prhs_copy_idx_8;
  }

  if (nlhs > 5) {
    plhs[5] = e_emlrt_marshallOut(map);
  }

  emxFreeStruct_struct1_T(&map);
  if (nlhs > 6) {
    f_emlrt_marshallOut(*occupancy_map, prhs_copy_idx_11);
    plhs[6] = prhs_copy_idx_11;
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vector_slam_gyro_data_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  vector_slam_gyro_data_xil_terminate();
  vector_slam_gyro_data_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vector_slam_gyro_data_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vector_slam_gyro_data_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_vector_slam_gyro_data_api.c
 *
 * [EOF]
 */
