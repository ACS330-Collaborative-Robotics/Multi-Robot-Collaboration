/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_mexutil.h"
#include "inverse_kinematics_types.h"
#include "qrsolve.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo em_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo fm_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo gm_emlrtRSI = {
    44,      /* lineNo */
    "mldiv", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo hm_emlrtRSI = {
    67,        /* lineNo */
    "lusolve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo im_emlrtRSI = {
    112,          /* lineNo */
    "lusolveNxN", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo jm_emlrtRSI = {
    109,          /* lineNo */
    "lusolveNxN", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo km_emlrtRSI = {
    124,          /* lineNo */
    "InvAtimesX", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo lm_emlrtRSI = {
    19,        /* lineNo */
    "xgetrfs", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pathName */
};

static emlrtRSInfo mm_emlrtRSI = {
    108,      /* lineNo */
    "cmldiv", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pathName */
};

static emlrtRSInfo nm_emlrtRSI = {
    70,       /* lineNo */
    "cmldiv", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pathName */
};

static emlrtRSInfo bn_emlrtRSI = {
    90,              /* lineNo */
    "warn_singular", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo cn_emlrtRSI = {
    61,        /* lineNo */
    "qrsolve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo dn_emlrtRSI = {
    72,        /* lineNo */
    "qrsolve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo en_emlrtRSI = {
    85,        /* lineNo */
    "qrsolve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo fn_emlrtRSI =
    {
        63,       /* lineNo */
        "xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo gn_emlrtRSI =
    {
        138,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo hn_emlrtRSI =
    {
        141,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo in_emlrtRSI =
    {
        143,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo jn_emlrtRSI =
    {
        148,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo kn_emlrtRSI =
    {
        151,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo ln_emlrtRSI =
    {
        154,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo mn_emlrtRSI =
    {
        158,            /* lineNo */
        "ceval_xgeqp3", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pathName */
};

static emlrtRSInfo nn_emlrtRSI = {
    173,          /* lineNo */
    "rankFromQR", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo on_emlrtRSI = {
    172,          /* lineNo */
    "rankFromQR", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRTEInfo nb_emlrtRTEI = {
    16,         /* lineNo */
    19,         /* colNo */
    "mldivide", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pName */
};

static emlrtRTEInfo hj_emlrtRTEI = {
    20,         /* lineNo */
    5,          /* colNo */
    "mldivide", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pName */
};

static emlrtRTEInfo ij_emlrtRTEI =
    {
        1,        /* lineNo */
        32,       /* colNo */
        "xgeqp3", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pName */
};

static emlrtRTEInfo jj_emlrtRTEI =
    {
        61,       /* lineNo */
        9,        /* colNo */
        "xgeqp3", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pName */
};

static emlrtRTEInfo kj_emlrtRTEI = {
    48,        /* lineNo */
    37,        /* colNo */
    "xgetrfs", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pName */
};

static emlrtRTEInfo lj_emlrtRTEI =
    {
        92,       /* lineNo */
        22,       /* colNo */
        "xgeqp3", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pName */
};

static emlrtRTEInfo mj_emlrtRTEI = {
    70,        /* lineNo */
    23,        /* colNo */
    "xgetrfs", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pName */
};

static emlrtRTEInfo nj_emlrtRTEI =
    {
        105,      /* lineNo */
        1,        /* colNo */
        "xgeqp3", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgeqp3.m" /* pName */
};

static emlrtRTEInfo oj_emlrtRTEI = {
    44,         /* lineNo */
    34,         /* colNo */
    "mldivide", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pName */
};

static emlrtRTEInfo pj_emlrtRTEI = {
    1,          /* lineNo */
    14,         /* colNo */
    "mldivide", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pName */
};

static emlrtRTEInfo qj_emlrtRTEI = {
    70,        /* lineNo */
    1,         /* colNo */
    "xgetrfs", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pName */
};

/* Function Definitions */
void mldivide(const emlrtStack *sp, const emxArray_real_T *A,
              const emxArray_real_T *B, emxArray_real_T *Y)
{
  static const int32_T b_iv[2] = {1, 6};
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'g', 'e', 'q', 'p', '3'};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  ptrdiff_t *jpvt_t_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_int32_T *jpvt;
  emxArray_ptrdiff_t *IPIV;
  emxArray_ptrdiff_t *jpvt_t;
  emxArray_real_T *b_A;
  emxArray_real_T *b_B;
  emxArray_real_T *tau;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  const real_T *A_data;
  const real_T *B_data;
  real_T *Y_data;
  real_T *b_A_data;
  int32_T b_na;
  int32_T i;
  int32_T minmn;
  int32_T rankA;
  int32_T *jpvt_data;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  B_data = B->data;
  A_data = A->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (B->size[0] != A->size[0]) {
    emlrtErrorWithMessageIdR2018a(sp, &nb_emlrtRTEI, "MATLAB:dimagree",
                                  "MATLAB:dimagree", 0);
  }
  st.site = &em_emlrtRSI;
  emxInit_real_T(&st, &b_A, 2, &pj_emlrtRTEI);
  emxInit_int32_T(&st, &jpvt, 2, &pj_emlrtRTEI);
  emxInit_real_T(&st, &tau, 1, &pj_emlrtRTEI);
  emxInit_ptrdiff_t(&st, &jpvt_t, &nj_emlrtRTEI);
  emxInit_ptrdiff_t(&st, &IPIV, &qj_emlrtRTEI);
  emxInit_real_T(&st, &b_B, 2, &oj_emlrtRTEI);
  if ((A->size[0] == 0) || (A->size[1] == 0) ||
      ((B->size[0] == 0) || (B->size[1] == 0))) {
    i = Y->size[0] * Y->size[1];
    Y->size[0] = A->size[1];
    Y->size[1] = B->size[1];
    emxEnsureCapacity_real_T(&st, Y, i, &hj_emlrtRTEI);
    Y_data = Y->data;
    minmn = A->size[1] * B->size[1];
    for (i = 0; i < minmn; i++) {
      Y_data[i] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    ptrdiff_t INFO;
    ptrdiff_t LDA;
    ptrdiff_t info_t;
    b_st.site = &fm_emlrtRSI;
    c_st.site = &hm_emlrtRSI;
    i = Y->size[0] * Y->size[1];
    Y->size[0] = B->size[0];
    Y->size[1] = B->size[1];
    emxEnsureCapacity_real_T(&c_st, Y, i, &hj_emlrtRTEI);
    Y_data = Y->data;
    minmn = B->size[0] * B->size[1];
    for (i = 0; i < minmn; i++) {
      Y_data[i] = B_data[i];
    }
    int32_T ma;
    int32_T na;
    d_st.site = &jm_emlrtRSI;
    e_st.site = &km_emlrtRSI;
    ma = A->size[0];
    na = A->size[1];
    b_na = B->size[0];
    i = muIntScalarMin_sint32(ma, na);
    b_na = muIntScalarMin_sint32(b_na, i);
    f_st.site = &lm_emlrtRSI;
    i = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(&f_st, b_A, i, &kj_emlrtRTEI);
    b_A_data = b_A->data;
    minmn = A->size[0] * A->size[1];
    for (i = 0; i < minmn; i++) {
      b_A_data[i] = A_data[i];
    }
    g_st.site = &nm_emlrtRSI;
    b_repmat(&g_st, (ptrdiff_t)0.0, b_na, jpvt_t);
    i = IPIV->size[0];
    IPIV->size[0] = jpvt_t->size[0];
    emxEnsureCapacity_ptrdiff_t(&f_st, IPIV, i, &mj_emlrtRTEI);
    jpvt_t_data = IPIV->data;
    info_t = (ptrdiff_t)b_na;
    LDA = (ptrdiff_t)b_A->size[0];
    INFO = LAPACKE_dgetrf_work(102, info_t, info_t, &b_A_data[0], LDA,
                               &jpvt_t_data[0]);
    g_st.site = &mm_emlrtRSI;
    if ((int32_T)INFO < 0) {
      if ((int32_T)INFO == -1010) {
        emlrtErrorWithMessageIdR2018a(&g_st, &ob_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&g_st, &pb_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      19, &cv2[0], 12, (int32_T)INFO);
      }
    }
    LAPACKE_dgetrs_work(102, 'N', info_t, (ptrdiff_t)B->size[1], &b_A_data[0],
                        LDA, &jpvt_t_data[0], &Y_data[0],
                        (ptrdiff_t)B->size[0]);
    if (((A->size[0] != 1) || (A->size[1] != 1)) && ((int32_T)INFO > 0)) {
      d_st.site = &im_emlrtRSI;
      e_st.site = &bn_emlrtRSI;
      b_warning(&e_st);
    }
  } else {
    ptrdiff_t info_t;
    real_T tol;
    int32_T ma;
    int32_T minmana;
    int32_T na;
    boolean_T p;
    b_st.site = &gm_emlrtRSI;
    c_st.site = &cn_emlrtRSI;
    i = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(&c_st, b_A, i, &ij_emlrtRTEI);
    b_A_data = b_A->data;
    minmn = A->size[0] * A->size[1];
    for (i = 0; i < minmn; i++) {
      b_A_data[i] = A_data[i];
    }
    rankA = b_A->size[0];
    na = b_A->size[1] - 1;
    i = jpvt->size[0] * jpvt->size[1];
    jpvt->size[0] = 1;
    jpvt->size[1] = b_A->size[1];
    emxEnsureCapacity_int32_T(&c_st, jpvt, i, &jj_emlrtRTEI);
    jpvt_data = jpvt->data;
    minmn = b_A->size[1];
    for (i = 0; i < minmn; i++) {
      jpvt_data[i] = 0;
    }
    d_st.site = &fn_emlrtRSI;
    ma = b_A->size[0];
    b_na = b_A->size[1];
    minmana = muIntScalarMin_sint32(ma, b_na);
    i = tau->size[0];
    tau->size[0] = minmana;
    emxEnsureCapacity_real_T(&d_st, tau, i, &lj_emlrtRTEI);
    Y_data = tau->data;
    i = jpvt_t->size[0];
    jpvt_t->size[0] = b_A->size[1];
    emxEnsureCapacity_ptrdiff_t(&d_st, jpvt_t, i, &nj_emlrtRTEI);
    jpvt_t_data = jpvt_t->data;
    minmn = b_A->size[1];
    for (i = 0; i < minmn; i++) {
      jpvt_t_data[i] = (ptrdiff_t)0;
    }
    info_t = LAPACKE_dgeqp3(
        102, (ptrdiff_t)b_A->size[0], (ptrdiff_t)b_A->size[1], &b_A_data[0],
        (ptrdiff_t)b_A->size[0], &jpvt_t_data[0], &Y_data[0]);
    e_st.site = &gn_emlrtRSI;
    if ((int32_T)info_t != 0) {
      p = true;
      if ((int32_T)info_t != -4) {
        if ((int32_T)info_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&e_st, &ob_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&e_st, &pb_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &fname[0], 12, (int32_T)info_t);
        }
      }
    } else {
      p = false;
    }
    if (p) {
      e_st.site = &hn_emlrtRSI;
      if (b_A->size[1] > 2147483646) {
        f_st.site = &vc_emlrtRSI;
        check_forloop_overflow_error(&f_st);
      }
      for (b_na = 0; b_na <= na; b_na++) {
        e_st.site = &in_emlrtRSI;
        if (rankA > 2147483646) {
          f_st.site = &vc_emlrtRSI;
          check_forloop_overflow_error(&f_st);
        }
        for (minmn = 0; minmn < rankA; minmn++) {
          b_A_data[b_na * ma + minmn] = rtNaN;
        }
      }
      minmn = muIntScalarMin_sint32(rankA, b_A->size[1]);
      e_st.site = &jn_emlrtRSI;
      for (rankA = 0; rankA < minmn; rankA++) {
        Y_data[rankA] = rtNaN;
      }
      b_na = minmn + 1;
      e_st.site = &kn_emlrtRSI;
      if ((minmn + 1 <= minmana) && (minmana > 2147483646)) {
        f_st.site = &vc_emlrtRSI;
        check_forloop_overflow_error(&f_st);
      }
      for (rankA = b_na; rankA <= minmana; rankA++) {
        Y_data[rankA - 1] = 0.0;
      }
      e_st.site = &ln_emlrtRSI;
      for (rankA = 0; rankA <= na; rankA++) {
        jpvt_data[rankA] = rankA + 1;
      }
    } else {
      e_st.site = &mn_emlrtRSI;
      if (b_A->size[1] > 2147483646) {
        f_st.site = &vc_emlrtRSI;
        check_forloop_overflow_error(&f_st);
      }
      for (rankA = 0; rankA <= na; rankA++) {
        jpvt_data[rankA] = (int32_T)jpvt_t_data[rankA];
      }
    }
    c_st.site = &dn_emlrtRSI;
    rankA = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      b_na = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      b_na = b_A->size[0];
    }
    tol = muDoubleScalarMin(1.4901161193847656E-8,
                            2.2204460492503131E-15 * (real_T)b_na) *
          muDoubleScalarAbs(b_A_data[0]);
    while (
        (rankA < minmn) &&
        (!(muDoubleScalarAbs(b_A_data[rankA + b_A->size[0] * rankA]) <= tol))) {
      rankA++;
    }
    if (rankA < minmn) {
      char_T str[14];
      d_st.site = &nn_emlrtRSI;
      y = NULL;
      m = emlrtCreateCharArray(2, &b_iv[0]);
      emlrtInitCharArrayR2013a(&d_st, 6, m, &rfmt[0]);
      emlrtAssign(&y, m);
      b_y = NULL;
      m = emlrtCreateDoubleScalar(tol);
      emlrtAssign(&b_y, m);
      e_st.site = &mq_emlrtRSI;
      emlrt_marshallIn(&e_st, b_sprintf(&e_st, y, b_y, &d_emlrtMCI),
                       "<output of sprintf>", str);
      d_st.site = &on_emlrtRSI;
      f_warning(&d_st, rankA, str);
    }
    i = b_B->size[0] * b_B->size[1];
    b_B->size[0] = B->size[0];
    b_B->size[1] = B->size[1];
    emxEnsureCapacity_real_T(&b_st, b_B, i, &oj_emlrtRTEI);
    Y_data = b_B->data;
    minmn = B->size[0] * B->size[1] - 1;
    for (i = 0; i <= minmn; i++) {
      Y_data[i] = B_data[i];
    }
    c_st.site = &en_emlrtRSI;
    LSQFromQR(&c_st, b_A, tau, jpvt, b_B, rankA, Y);
  }
  emxFree_real_T(&st, &b_B);
  emxFree_ptrdiff_t(&st, &IPIV);
  emxFree_ptrdiff_t(&st, &jpvt_t);
  emxFree_real_T(&st, &tau);
  emxFree_int32_T(&st, &jpvt);
  emxFree_real_T(&st, &b_A);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (mldivide.c) */
