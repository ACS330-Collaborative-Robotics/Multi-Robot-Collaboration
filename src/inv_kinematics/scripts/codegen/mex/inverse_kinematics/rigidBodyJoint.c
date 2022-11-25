/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rigidBodyJoint.c
 *
 * Code generation for function 'rigidBodyJoint'
 *
 */

/* Include files */
#include "rigidBodyJoint.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "normalizeRows.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo
    yj_emlrtRSI =
        {
            399,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo ak_emlrtRSI = {
    38,    /* lineNo */
    "cat", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/cat.m" /* pathName
                                                                       */
};

static emlrtRSInfo bk_emlrtRSI = {
    110,        /* lineNo */
    "cat_impl", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/cat.m" /* pathName
                                                                       */
};

static emlrtRSInfo ck_emlrtRSI =
    {
        22,           /* lineNo */
        "axang2rotm", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/"
        "axang2rotm.m" /* pathName */
};

static emlrtRSInfo dk_emlrtRSI = {
    21,                      /* lineNo */
    "validateNumericMatrix", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+internal/+validation/validateNumericMatrix.m" /* pathName */
};

static emlrtRTEInfo ib_emlrtRTEI = {
    285,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/cat.m" /* pName
                                                                       */
};

static emlrtRTEInfo lb_emlrtRTEI = {
    18,              /* lineNo */
    23,              /* colNo */
    "validatencols", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatencols.m" /* pName */
};

static emlrtRTEInfo
    lk_emlrtRTEI =
        {
            525,              /* lineNo */
            30,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    mk_emlrtRTEI =
        {
            525,              /* lineNo */
            35,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    nk_emlrtRTEI =
        {
            525,              /* lineNo */
            40,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    ok_emlrtRTEI =
        {
            529,              /* lineNo */
            30,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

/* Function Definitions */
void b_minus(const emlrtStack *sp, emxArray_real_T *in1,
             const emxArray_real_T *in2)
{
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  real_T *b_in2_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in2, 1, &ok_emlrtRTEI);
  i = b_in2->size[0];
  if (in1->size[0] == 1) {
    b_in2->size[0] = in2->size[0];
  } else {
    b_in2->size[0] = in1->size[0];
  }
  emxEnsureCapacity_real_T(sp, b_in2, i, &ok_emlrtRTEI);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  if (in1->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in1->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] = in2_data[i * stride_0_0] - in1_data[i * stride_1_0];
  }
  i = in1->size[0];
  in1->size[0] = b_in2->size[0];
  emxEnsureCapacity_real_T(sp, in1, i, &ok_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in2->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in2_data[i];
  }
  emxFree_real_T(sp, &b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_rigidBodyJoint_transformBodyT(const emlrtStack *sp,
                                     const rigidBodyJoint *obj,
                                     const emxArray_real_T *q, real_T T[16])
{
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T b[16];
  real_T b_obj[16];
  real_T tempR[9];
  const real_T *q_data;
  real_T b_tempR_tmp;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  int32_T exitg1;
  int32_T i;
  int32_T i1;
  int32_T kstr;
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  q_data = q->data;
  st.site = &jj_emlrtRSI;
  b_bool = false;
  if (obj->Type->size[1] == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (obj->Type->data[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    kstr = 0;
  } else {
    b_bool = false;
    if (obj->Type->size[1] == 8) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 8) {
          if (obj->Type->data[kstr] != b_cv1[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      kstr = 1;
    } else {
      kstr = -1;
    }
  }
  switch (kstr) {
  case 0:
    memset(&b[0], 0, 16U * sizeof(real_T));
    b[0] = 1.0;
    b[5] = 1.0;
    b[10] = 1.0;
    b[15] = 1.0;
    break;
  case 1: {
    real_T R[9];
    real_T result_data[4];
    real_T v[3];
    real_T c_tempR_tmp;
    real_T d_tempR_tmp;
    real_T e_tempR_tmp;
    real_T f_tempR_tmp;
    real_T g_tempR_tmp;
    real_T h_tempR_tmp;
    real_T i_tempR_tmp;
    real_T j_tempR_tmp;
    int8_T input_sizes_idx_1;
    b_st.site = &kj_emlrtRSI;
    rigidBodyJoint_get_JointAxis(obj, v);
    b_st.site = &lj_emlrtRSI;
    c_st.site = &ak_emlrtRSI;
    d_st.site = &bk_emlrtRSI;
    if ((q->size[0] != 1) && (q->size[0] != 0)) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &ib_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    input_sizes_idx_1 = (int8_T)(q->size[0] != 0);
    result_data[0] = v[0];
    result_data[1] = v[1];
    result_data[2] = v[2];
    kstr = input_sizes_idx_1;
    if (kstr - 1 >= 0) {
      result_data[3] = q_data[0];
    }
    b_st.site = &lj_emlrtRSI;
    c_st.site = &nj_emlrtRSI;
    d_st.site = &ck_emlrtRSI;
    e_st.site = &dk_emlrtRSI;
    f_st.site = &y_emlrtRSI;
    if (input_sizes_idx_1 + 3 != 4) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &lb_emlrtRTEI,
          "Coder:toolbox:ValidateattributesincorrectNumcols",
          "MATLAB:axang2rotm:incorrectNumcols", 5, 4, 5, "axang", 6, 4.0);
    }
    d_st.site = &oj_emlrtRSI;
    normalizeRows(&d_st, &result_data[0], v);
    cth = muDoubleScalarCos(result_data[3]);
    sth = muDoubleScalarSin(result_data[3]);
    c_tempR_tmp = v[0] * v[0] * (1.0 - cth) + cth;
    tempR[0] = c_tempR_tmp;
    d_tempR_tmp = v[0] * v[1] * (1.0 - cth);
    e_tempR_tmp = v[2] * sth;
    f_tempR_tmp = d_tempR_tmp - e_tempR_tmp;
    tempR[1] = f_tempR_tmp;
    g_tempR_tmp = v[0] * v[2] * (1.0 - cth);
    h_tempR_tmp = v[1] * sth;
    i_tempR_tmp = g_tempR_tmp + h_tempR_tmp;
    tempR[2] = i_tempR_tmp;
    d_tempR_tmp += e_tempR_tmp;
    tempR[3] = d_tempR_tmp;
    e_tempR_tmp = v[1] * v[1] * (1.0 - cth) + cth;
    tempR[4] = e_tempR_tmp;
    j_tempR_tmp = v[1] * v[2] * (1.0 - cth);
    tempR_tmp = v[0] * sth;
    b_tempR_tmp = j_tempR_tmp - tempR_tmp;
    tempR[5] = b_tempR_tmp;
    g_tempR_tmp -= h_tempR_tmp;
    tempR[6] = g_tempR_tmp;
    h_tempR_tmp = j_tempR_tmp + tempR_tmp;
    tempR[7] = h_tempR_tmp;
    j_tempR_tmp = v[2] * v[2] * (1.0 - cth) + cth;
    tempR[8] = j_tempR_tmp;
    R[0] = c_tempR_tmp;
    R[1] = f_tempR_tmp;
    R[2] = i_tempR_tmp;
    R[3] = d_tempR_tmp;
    R[4] = e_tempR_tmp;
    R[5] = b_tempR_tmp;
    R[6] = g_tempR_tmp;
    R[7] = h_tempR_tmp;
    R[8] = j_tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[kstr] = tempR[3 * kstr];
      R[kstr + 3] = tempR[3 * kstr + 1];
      R[kstr + 6] = tempR[3 * kstr + 2];
    }
    memset(&b[0], 0, 16U * sizeof(real_T));
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = R[3 * i];
      b[kstr + 1] = R[3 * i + 1];
      b[kstr + 2] = R[3 * i + 2];
    }
    b[15] = 1.0;
  } break;
  default: {
    real_T v[3];
    b_st.site = &mj_emlrtRSI;
    rigidBodyJoint_get_JointAxis(obj, v);
    b_st.site = &yj_emlrtRSI;
    c_st.site = &ek_emlrtRSI;
    if (q->size[0] != 1) {
      if (q->size[0] == 1) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &kb_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &jb_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    memset(&tempR[0], 0, 9U * sizeof(real_T));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = tempR[3 * i];
      b[kstr + 1] = tempR[3 * i + 1];
      b[kstr + 2] = tempR[3 * i + 2];
      b[i + 12] = v[i] * q_data[0];
    }
    b[3] = 0.0;
    b[7] = 0.0;
    b[11] = 0.0;
    b[15] = 1.0;
  } break;
  }
  for (i = 0; i < 4; i++) {
    sth = obj->JointToParentTransform[i];
    tempR_tmp = obj->JointToParentTransform[i + 4];
    b_tempR_tmp = obj->JointToParentTransform[i + 8];
    cth = obj->JointToParentTransform[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      kstr = i1 << 2;
      b_obj[i + kstr] = ((sth * b[kstr] + tempR_tmp * b[kstr + 1]) +
                         b_tempR_tmp * b[kstr + 2]) +
                        cth * b[kstr + 3];
    }
    sth = b_obj[i];
    tempR_tmp = b_obj[i + 4];
    b_tempR_tmp = b_obj[i + 8];
    cth = b_obj[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      kstr = i1 << 2;
      T[i + kstr] = ((sth * obj->ChildToJointTransform[kstr] +
                      tempR_tmp * obj->ChildToJointTransform[kstr + 1]) +
                     b_tempR_tmp * obj->ChildToJointTransform[kstr + 2]) +
                    cth * obj->ChildToJointTransform[kstr + 3];
    }
  }
}

void minus(const emlrtStack *sp, emxArray_real_T *in1,
           const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 1, &nk_emlrtRTEI);
  i = b_in1->size[0];
  if (in2->size[0] == 1) {
    b_in1->size[0] = in1->size[0];
  } else {
    b_in1->size[0] = in2->size[0];
  }
  emxEnsureCapacity_real_T(sp, b_in1, i, &nk_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] - in2_data[i * stride_1_0];
  }
  i = in1->size[0];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_real_T(sp, in1, i, &nk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void plus(const emlrtStack *sp, emxArray_real_T *in1,
          const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 1, &lk_emlrtRTEI);
  i = b_in1->size[0];
  if (in2->size[0] == 1) {
    b_in1->size[0] = in1->size[0];
  } else {
    b_in1->size[0] = in2->size[0];
  }
  emxEnsureCapacity_real_T(sp, b_in1, i, &lk_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] + in2_data[i * stride_1_0];
  }
  i = in1->size[0];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_real_T(sp, in1, i, &lk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void rigidBodyJoint_get_JointAxis(const rigidBodyJoint *obj, real_T ax[3])
{
  static const char_T b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int32_T exitg1;
  int32_T kstr;
  boolean_T b_bool;
  boolean_T guard1 = false;
  b_bool = false;
  if (obj->Type->size[1] == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (obj->Type->data[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    b_bool = false;
    if (obj->Type->size[1] == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (obj->Type->data[kstr] != cv[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = rtNaN;
      ax[1] = rtNaN;
      ax[2] = rtNaN;
    }
  }
  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

rigidBodyJoint *rigidBodyJoint_rigidBodyJoint(const emlrtStack *sp,
                                              rigidBodyJoint *obj,
                                              const emxArray_char_T *jname)
{
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  emlrtStack b_st;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  rigidBodyJoint *b_obj;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  const char_T *jname_data;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  jname_data = jname->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  b_obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_obj->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->ChildToJointTransform[i] = iv[i];
  }
  st.site = &x_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  if (jname->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:rigidBodyJoint:expectedNonempty", 3, 4, 5, "jname");
  }
  st.site = &q_emlrtRSI;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = jname->size[1];
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &jc_emlrtRTEI);
  loop_ub = jname->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_obj->NameInternal->data[i] = jname_data[i];
  }
  i = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 5;
  emxEnsureCapacity_char_T(sp, b_obj->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    b_obj->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(sp, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(sp, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = b_obj->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = b_obj->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(sp, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }
  i = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = b_obj->PositionLimitsInternal->size[0] *
      b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b_obj->PositionLimitsInternal, i, &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    b_obj->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->HomePositionInternal, i, &oc_emlrtRTEI);
  b_obj->HomePositionInternal->data[0] = 0.0;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

void times(const emlrtStack *sp, emxArray_real_T *in1,
           const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 1, &mk_emlrtRTEI);
  i = b_in1->size[0];
  if (in2->size[0] == 1) {
    b_in1->size[0] = in1->size[0];
  } else {
    b_in1->size[0] = in2->size[0];
  }
  emxEnsureCapacity_real_T(sp, b_in1, i, &mk_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] * in2_data[i * stride_1_0];
  }
  i = in1->size[0];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_real_T(sp, in1, i, &mk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (rigidBodyJoint.c) */
