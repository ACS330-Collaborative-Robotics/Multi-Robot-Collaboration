/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * isPositiveDefinite.c
 *
 * Code generation for function 'isPositiveDefinite'
 *
 */

/* Include files */
#include "isPositiveDefinite.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo dp_emlrtRSI = {
    11,                   /* lineNo */
    "isPositiveDefinite", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/isPositiveDefinite.m" /* pathName */
};

static emlrtRSInfo ep_emlrtRSI = {
    15,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/chol.m" /* pathName
                                                                     */
};

static emlrtRSInfo fp_emlrtRSI = {
    84,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo gp_emlrtRSI = {
    93,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo hp_emlrtRSI = {
    94,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo ip_emlrtRSI =
    {
        79,             /* lineNo */
        "ceval_xpotrf", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xpotrf.m" /* pathName */
};

static emlrtRSInfo jp_emlrtRSI =
    {
        13,       /* lineNo */
        "xpotrf", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xpotrf.m" /* pathName */
};

static emlrtRTEInfo tb_emlrtRTEI = {
    56,     /* lineNo */
    23,     /* colNo */
    "chol", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/chol.m" /* pName
                                                                        */
};

static emlrtRTEInfo ub_emlrtRTEI = {
    16,                                                             /* lineNo */
    5,                                                              /* colNo */
    "chol",                                                         /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/chol.m" /* pName */
};

static emlrtRTEInfo gk_emlrtRTEI = {
    1,      /* lineNo */
    31,     /* colNo */
    "chol", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/chol.m" /* pName
                                                                        */
};

static emlrtRTEInfo hk_emlrtRTEI = {
    1,                    /* lineNo */
    17,                   /* colNo */
    "isPositiveDefinite", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/isPositiveDefinite.m" /* pName */
};

/* Function Definitions */
boolean_T isPositiveDefinite(const emlrtStack *sp, const emxArray_real_T *B)
{
  static const char_T fname[19] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'p', 'o', 't', 'r', 'f',
                                   '_', 'w', 'o', 'r', 'k'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_real_T *A;
  const real_T *B_data;
  real_T *A_data;
  int32_T i;
  int32_T info;
  int32_T mrows;
  int32_T ncols;
  boolean_T flag;
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
  B_data = B->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &dp_emlrtRSI;
  b_st.site = &ep_emlrtRSI;
  emxInit_real_T(&b_st, &A, 2, &hk_emlrtRTEI);
  ncols = A->size[0] * A->size[1];
  A->size[0] = B->size[0];
  A->size[1] = B->size[1];
  emxEnsureCapacity_real_T(&b_st, A, ncols, &gk_emlrtRTEI);
  A_data = A->data;
  mrows = B->size[0] * B->size[1];
  for (ncols = 0; ncols < mrows; ncols++) {
    A_data[ncols] = B_data[ncols];
  }
  mrows = A->size[0];
  ncols = A->size[1];
  if (A->size[0] != A->size[1]) {
    emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI, "MATLAB:square",
                                  "MATLAB:square", 0);
  }
  mrows = muIntScalarMin_sint32(mrows, ncols);
  ncols = 0;
  info = 0;
  if (mrows != 0) {
    ptrdiff_t info_t;
    c_st.site = &fp_emlrtRSI;
    d_st.site = &jp_emlrtRSI;
    info_t = LAPACKE_dpotrf_work(102, 'L', (ptrdiff_t)mrows, &A_data[0],
                                 (ptrdiff_t)A->size[0]);
    e_st.site = &ip_emlrtRSI;
    if ((int32_T)info_t < 0) {
      if ((int32_T)info_t == -1010) {
        emlrtErrorWithMessageIdR2018a(&e_st, &ob_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&e_st, &pb_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      19, &fname[0], 12, (int32_T)info_t);
      }
    }
    info = (int32_T)info_t;
    if ((int32_T)info_t == 0) {
      ncols = mrows;
    } else {
      ncols = (int32_T)info_t - 1;
    }
    c_st.site = &gp_emlrtRSI;
    if (ncols > 2147483646) {
      d_st.site = &vc_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    for (mrows = 2; mrows <= ncols; mrows++) {
      c_st.site = &hp_emlrtRSI;
      for (i = 0; i <= mrows - 2; i++) {
        A_data[i + A->size[0] * (mrows - 1)] = 0.0;
      }
    }
  }
  if ((ncols > A->size[0]) || (ncols > A->size[1])) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  emxFree_real_T(&st, &A);
  flag = (info == 0);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return flag;
}

/* End of code generation (isPositiveDefinite.c) */
