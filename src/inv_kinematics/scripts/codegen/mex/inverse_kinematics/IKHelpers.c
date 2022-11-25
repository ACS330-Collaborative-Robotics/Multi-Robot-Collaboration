/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * IKHelpers.c
 *
 * Code generation for function 'IKHelpers'
 *
 */

/* Include files */
#include "IKHelpers.h"
#include "RigidBodyTree.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "normalizeRows.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "xzsvdc.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo vi_emlrtRSI = {
    22,                      /* lineNo */
    "IKHelpers/computeCost", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pathName */
};

static emlrtRSInfo wi_emlrtRSI = {
    27,                      /* lineNo */
    "IKHelpers/computeCost", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pathName */
};

static emlrtRSInfo xi_emlrtRSI = {
    30,                      /* lineNo */
    "IKHelpers/computeCost", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pathName */
};

static emlrtRSInfo ik_emlrtRSI = {
    64,                    /* lineNo */
    "IKHelpers/poseError", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pathName */
};

static emlrtRSInfo jk_emlrtRSI =
    {
        53,           /* lineNo */
        "rotm2axang", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/"
        "rotm2axang.m" /* pathName */
};

static emlrtRSInfo kk_emlrtRSI =
    {
        44,           /* lineNo */
        "rotm2axang", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/"
        "rotm2axang.m" /* pathName */
};

static emlrtRSInfo lk_emlrtRSI = {
    14,                                                            /* lineNo */
    "svd",                                                         /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/svd.m" /* pathName
                                                                    */
};

static emlrtRSInfo mk_emlrtRSI = {
    36,                                                            /* lineNo */
    "svd",                                                         /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/svd.m" /* pathName
                                                                    */
};

static emlrtRSInfo nk_emlrtRSI = {
    42,                                                            /* lineNo */
    "svd",                                                         /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/svd.m" /* pathName
                                                                    */
};

static emlrtRSInfo ok_emlrtRSI =
    {
        29,             /* lineNo */
        "anyNonFinite", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
        "anyNonFinite.m" /* pathName */
};

static emlrtRSInfo
    pk_emlrtRSI =
        {
            44,          /* lineNo */
            "vAllOrAny", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
            "vAllOrAny.m" /* pathName */
};

static emlrtRSInfo rk_emlrtRSI = {
    52,    /* lineNo */
    "svd", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo sk_emlrtRSI = {
    107,          /* lineNo */
    "callLAPACK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo tk_emlrtRSI =
    {
        34,       /* lineNo */
        "xgesvd", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgesvd.m" /* pathName */
};

static emlrtRSInfo dm_emlrtRSI = {
    42,                           /* lineNo */
    "IKHelpers/evaluateSolution", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pathName */
};

static emlrtBCInfo ie_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    61,                    /* lineNo */
    23,                    /* colNo */
    "",                    /* aName */
    "IKHelpers/poseError", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m", /* pName */
    0                        /* checkKind */
};

static emlrtECInfo tb_emlrtECI =
    {
        -1,           /* nDims */
        47,           /* lineNo */
        5,            /* colNo */
        "rotm2axang", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/"
        "rotm2axang.m" /* pName */
};

static emlrtBCInfo je_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    59,                    /* lineNo */
    23,                    /* colNo */
    "",                    /* aName */
    "IKHelpers/poseError", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo ke_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    59,                    /* lineNo */
    19,                    /* colNo */
    "",                    /* aName */
    "IKHelpers/poseError", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m", /* pName */
    0                        /* checkKind */
};

static emlrtRTEInfo li_emlrtRTEI = {
    15,          /* lineNo */
    13,          /* colNo */
    "IKHelpers", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pName */
};

static emlrtRTEInfo mi_emlrtRTEI = {
    26,          /* lineNo */
    13,          /* colNo */
    "IKHelpers", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pName */
};

static emlrtRTEInfo ni_emlrtRTEI = {
    28,          /* lineNo */
    13,          /* colNo */
    "IKHelpers", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pName */
};

static emlrtRTEInfo oi_emlrtRTEI = {
    30,          /* lineNo */
    13,          /* colNo */
    "IKHelpers", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pName */
};

static emlrtRTEInfo qi_emlrtRTEI = {
    30,          /* lineNo */
    30,          /* colNo */
    "IKHelpers", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pName */
};

/* Function Declarations */
static void IKHelpers_poseError(const emlrtStack *sp, const real_T Td[16],
                                const real_T T_data[], const int32_T T_size[2],
                                real_T errorvec[6]);

/* Function Definitions */
static void IKHelpers_poseError(const emlrtStack *sp, const real_T Td[16],
                                const real_T T_data[], const int32_T T_size[2],
                                real_T errorvec[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  creal_T u;
  creal_T v;
  real_T b_I[9];
  real_T y[9];
  real_T S[3];
  real_T b_v[3];
  real_T br_tmp_tmp;
  real_T q;
  int32_T b_iv[3];
  int32_T vspecial_size[2];
  int32_T i;
  int32_T i1;
  int32_T k;
  boolean_T exitg1;
  boolean_T rEQ0;
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
  if (T_size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, T_size[0], &ke_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (T_size[0] < 2) {
    emlrtDynamicBoundsCheckR2012b(2, 1, 1, &ke_emlrtBCI, (emlrtConstCTX)sp);
  }
  if (T_size[0] < 3) {
    emlrtDynamicBoundsCheckR2012b(3, 1, 2, &ke_emlrtBCI, (emlrtConstCTX)sp);
  }
  if (T_size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, T_size[1], &je_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (T_size[1] < 2) {
    emlrtDynamicBoundsCheckR2012b(2, 1, 1, &je_emlrtBCI, (emlrtConstCTX)sp);
  }
  if (T_size[1] < 3) {
    emlrtDynamicBoundsCheckR2012b(3, 1, 2, &je_emlrtBCI, (emlrtConstCTX)sp);
  }
  if (T_size[1] < 4) {
    emlrtDynamicBoundsCheckR2012b(4, 1, 3, &ie_emlrtBCI, (emlrtConstCTX)sp);
  }
  k = T_size[0];
  for (i = 0; i < 3; i++) {
    real_T d;
    br_tmp_tmp = Td[i];
    q = Td[i + 4];
    d = Td[i + 8];
    for (i1 = 0; i1 < 3; i1++) {
      y[i + 3 * i1] = (br_tmp_tmp * T_data[i1] + q * T_data[i1 + k]) +
                      d * T_data[i1 + k * 2];
    }
  }
  st.site = &ik_emlrtRSI;
  u.re = 0.5 * (((y[0] + y[4]) + y[8]) - 1.0);
  if (!(muDoubleScalarAbs(u.re) > 1.0)) {
    br_tmp_tmp = u.re;
    u.re = muDoubleScalarAcos(br_tmp_tmp);
  } else {
    v.re = u.re + 1.0;
    v.im = 0.0;
    b_sqrt(&v);
    br_tmp_tmp = u.re;
    u.re = 1.0 - br_tmp_tmp;
    u.im = 0.0;
    b_sqrt(&u);
    br_tmp_tmp = u.re;
    u.re = 2.0 * muDoubleScalarAtan2(br_tmp_tmp, v.re);
  }
  br_tmp_tmp = 2.0 * muDoubleScalarSin(u.re);
  b_v[0] = (y[5] - y[7]) / br_tmp_tmp;
  b_v[1] = (y[6] - y[2]) / br_tmp_tmp;
  b_v[2] = (y[1] - y[3]) / br_tmp_tmp;
  if (muDoubleScalarIsNaN(u.re) || muDoubleScalarIsInf(u.re)) {
    br_tmp_tmp = rtNaN;
  } else if (u.re == 0.0) {
    br_tmp_tmp = 0.0;
  } else {
    br_tmp_tmp = muDoubleScalarRem(u.re, 3.1415926535897931);
    rEQ0 = (br_tmp_tmp == 0.0);
    if (!rEQ0) {
      q = muDoubleScalarAbs(u.re / 3.1415926535897931);
      rEQ0 = !(muDoubleScalarAbs(q - muDoubleScalarFloor(q + 0.5)) >
               2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      br_tmp_tmp = 0.0;
    } else if (u.re < 0.0) {
      br_tmp_tmp += 3.1415926535897931;
    }
  }
  rEQ0 = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!(b_v[k] == 0.0)) {
      rEQ0 = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if ((br_tmp_tmp == 0.0) || rEQ0) {
    real_T V[9];
    vspecial_size[0] = 3;
    vspecial_size[1] = 1;
    memset(&b_I[0], 0, 9U * sizeof(real_T));
    b_I[0] = 1.0;
    b_I[4] = 1.0;
    b_I[8] = 1.0;
    b_st.site = &kk_emlrtRSI;
    for (k = 0; k < 9; k++) {
      b_I[k] -= y[k];
    }
    c_st.site = &lk_emlrtRSI;
    d_st.site = &ok_emlrtRSI;
    e_st.site = &pk_emlrtRSI;
    rEQ0 = true;
    for (k = 0; k < 9; k++) {
      if (rEQ0) {
        br_tmp_tmp = b_I[k];
        if (muDoubleScalarIsInf(br_tmp_tmp) ||
            muDoubleScalarIsNaN(br_tmp_tmp)) {
          rEQ0 = false;
        }
      } else {
        rEQ0 = false;
      }
    }
    if (rEQ0) {
      c_st.site = &mk_emlrtRSI;
      d_st.site = &rk_emlrtRSI;
      e_st.site = &sk_emlrtRSI;
      f_st.site = &tk_emlrtRSI;
      xzsvdc(&f_st, b_I, y, S, V);
    } else {
      c_st.site = &nk_emlrtRSI;
      d_st.site = &rk_emlrtRSI;
      e_st.site = &sk_emlrtRSI;
      memset(&b_I[0], 0, 9U * sizeof(real_T));
      f_st.site = &tk_emlrtRSI;
      xzsvdc(&f_st, b_I, y, S, V);
      for (k = 0; k < 9; k++) {
        V[k] = rtNaN;
      }
    }
    b_iv[0] = 1;
    b_iv[1] = 3;
    b_iv[2] = 1;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 3, &vspecial_size[0], 2,
                                  &tb_emlrtECI, &st);
    b_v[0] = V[6];
    b_v[1] = V[7];
    b_v[2] = V[8];
  }
  for (k = 0; k < 3; k++) {
    S[k] = b_v[k];
  }
  b_st.site = &jk_emlrtRSI;
  normalizeRows(&b_st, S, b_v);
  errorvec[0] = u.re * b_v[0];
  errorvec[3] = Td[12] - T_data[T_size[0] * 3];
  errorvec[1] = u.re * b_v[1];
  errorvec[4] = Td[13] - T_data[T_size[0] * 3 + 1];
  errorvec[2] = u.re * b_v[2];
  errorvec[5] = Td[14] - T_data[T_size[0] * 3 + 2];
}

void IKHelpers_computeCost(const emlrtStack *sp, const emxArray_real_T *x,
                           c_robotics_manip_internal_IKExt *args, real_T *cost,
                           real_T W[36], emxArray_real_T *Jac,
                           c_robotics_manip_internal_IKExt **b_args)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  e_robotics_manip_internal_Rigid *treeInternal;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_char_T *bodyName;
  emxArray_real_T *b_y;
  real_T T_data[16];
  real_T Td[16];
  real_T e[6];
  real_T y[6];
  real_T alpha1;
  real_T beta1;
  real_T *Jac_data;
  real_T *y_data;
  int32_T T_size[2];
  int32_T i;
  int32_T loop_ub;
  char_T TRANSA1;
  char_T TRANSB1;
  char_T *bodyName_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  *b_args = args;
  treeInternal = (*b_args)->Robot;
  emxInit_char_T(sp, &bodyName, &li_emlrtRTEI);
  i = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = (*b_args)->BodyName->size[1];
  emxEnsureCapacity_char_T(sp, bodyName, i, &li_emlrtRTEI);
  bodyName_data = bodyName->data;
  loop_ub = (*b_args)->BodyName->size[1];
  for (i = 0; i < loop_ub; i++) {
    bodyName_data[i] = (*b_args)->BodyName->data[i];
  }
  for (i = 0; i < 16; i++) {
    Td[i] = (*b_args)->Tform[i];
  }
  for (i = 0; i < 36; i++) {
    W[i] = (*b_args)->WeightMatrix[i];
  }
  st.site = &vi_emlrtRSI;
  c_RigidBodyTree_efficientFKAndJ(&st, treeInternal, x, bodyName, T_data,
                                  T_size, Jac);
  emxFree_char_T(sp, &bodyName);
  loop_ub = 6 * Jac->size[1];
  i = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  emxEnsureCapacity_real_T(sp, Jac, i, &mi_emlrtRTEI);
  Jac_data = Jac->data;
  for (i = 0; i < loop_ub; i++) {
    Jac_data[i] = -Jac_data[i];
  }
  st.site = &wi_emlrtRSI;
  IKHelpers_poseError(&st, Td, T_data, T_size, e);
  i = (*b_args)->ErrTemp->size[0];
  (*b_args)->ErrTemp->size[0] = 6;
  emxEnsureCapacity_real_T(sp, (*b_args)->ErrTemp, i, &ni_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    (*b_args)->ErrTemp->data[i] = e[i];
  }
  alpha1 = 0.0;
  for (i = 0; i < 6; i++) {
    beta1 = 0.0;
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      beta1 += 0.5 * e[loop_ub] * W[loop_ub + 6 * i];
    }
    alpha1 += beta1 * e[i];
  }
  (*b_args)->CostTemp = alpha1;
  for (i = 0; i < 6; i++) {
    beta1 = 0.0;
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      beta1 += e[loop_ub] * W[loop_ub + 6 * i];
    }
    y[i] = beta1;
  }
  emxInit_real_T(sp, &b_y, 2, &qi_emlrtRTEI);
  y_data = b_y->data;
  st.site = &xi_emlrtRSI;
  b_st.site = &fk_emlrtRSI;
  if (Jac->size[1] == 0) {
    b_y->size[0] = 1;
    b_y->size[1] = 0;
  } else {
    c_st.site = &gk_emlrtRSI;
    d_st.site = &hk_emlrtRSI;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)1;
    n_t = (ptrdiff_t)Jac->size[1];
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)1;
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)1;
    i = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = Jac->size[1];
    emxEnsureCapacity_real_T(&d_st, b_y, i, &pi_emlrtRTEI);
    y_data = b_y->data;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &y[0], &lda_t,
          &Jac_data[0], &ldb_t, &beta1, &y_data[0], &ldc_t);
  }
  i = (*b_args)->GradTemp->size[0];
  (*b_args)->GradTemp->size[0] = b_y->size[1];
  emxEnsureCapacity_real_T(sp, (*b_args)->GradTemp, i, &oi_emlrtRTEI);
  loop_ub = b_y->size[1];
  for (i = 0; i < loop_ub; i++) {
    (*b_args)->GradTemp->data[i] = y_data[i];
  }
  emxFree_real_T(sp, &b_y);
  *cost = (*b_args)->CostTemp;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

real_T IKHelpers_evaluateSolution(const emlrtStack *sp,
                                  const c_robotics_manip_internal_IKExt *args)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T y[6];
  real_T en;
  real_T scale;
  int32_T i;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &dm_emlrtRSI;
  b_st.site = &ek_emlrtRSI;
  if (args->ErrTemp->size[0] != 6) {
    if (args->ErrTemp->size[0] == 1) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &kb_emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &jb_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
  for (k = 0; k < 6; k++) {
    scale = 0.0;
    for (i = 0; i < 6; i++) {
      scale += args->WeightMatrix[k + 6 * i] * args->ErrTemp->data[i];
    }
    y[k] = scale;
  }
  en = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 6; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(y[k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      en = en * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      en += t * t;
    }
  }
  return scale * muDoubleScalarSqrt(en);
}

/* End of code generation (IKHelpers.c) */
