/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qrsolve.c
 *
 * Code generation for function 'qrsolve'
 *
 */

/* Include files */
#include "qrsolve.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo pn_emlrtRSI = {
    119,         /* lineNo */
    "LSQFromQR", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo qn_emlrtRSI = {
    126,         /* lineNo */
    "LSQFromQR", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo rn_emlrtRSI = {
    128,         /* lineNo */
    "LSQFromQR", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo sn_emlrtRSI = {
    138,         /* lineNo */
    "LSQFromQR", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo tn_emlrtRSI = {
    31,         /* lineNo */
    "xunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xunormqr.m" /* pathName */
};

static emlrtRSInfo un_emlrtRSI = {
    102,              /* lineNo */
    "ceval_xunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xunormqr.m" /* pathName */
};

static emlrtRSInfo vn_emlrtRSI = {
    108,              /* lineNo */
    "ceval_xunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xunormqr.m" /* pathName */
};

static emlrtRSInfo wn_emlrtRSI = {
    18,          /* lineNo */
    "xzunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzunormqr.m" /* pathName */
};

static emlrtRSInfo xn_emlrtRSI = {
    21,          /* lineNo */
    "xzunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzunormqr.m" /* pathName */
};

static emlrtRSInfo yn_emlrtRSI = {
    23,          /* lineNo */
    "xzunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzunormqr.m" /* pathName */
};

static emlrtRSInfo ao_emlrtRSI = {
    29,          /* lineNo */
    "xzunormqr", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzunormqr.m" /* pathName */
};

static emlrtRTEInfo rj_emlrtRTEI = {
    109,       /* lineNo */
    1,         /* colNo */
    "qrsolve", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pName
                                                                           */
};

static emlrtRTEInfo sj_emlrtRTEI = {
    119,       /* lineNo */
    5,         /* colNo */
    "qrsolve", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/qrsolve.m" /* pName
                                                                           */
};

/* Function Definitions */
void LSQFromQR(const emlrtStack *sp, const emxArray_real_T *A,
               const emxArray_real_T *tau, const emxArray_int32_T *jpvt,
               emxArray_real_T *B, int32_T rankA, emxArray_real_T *Y)
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'o', 'r', 'm', 'q', 'r'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  const real_T *A_data;
  const real_T *tau_data;
  real_T *B_data;
  real_T *Y_data;
  const int32_T *jpvt_data;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T m;
  int32_T mn;
  int32_T nb;
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
  jpvt_data = jpvt->data;
  tau_data = tau->data;
  A_data = A->data;
  nb = B->size[1];
  mn = Y->size[0] * Y->size[1];
  Y->size[0] = A->size[1];
  Y->size[1] = B->size[1];
  emxEnsureCapacity_real_T(sp, Y, mn, &rj_emlrtRTEI);
  Y_data = Y->data;
  m = A->size[1] * B->size[1];
  for (mn = 0; mn < m; mn++) {
    Y_data[mn] = 0.0;
  }
  st.site = &pn_emlrtRSI;
  b_st.site = &tn_emlrtRSI;
  if ((A->size[0] != 0) && (A->size[1] != 0) &&
      ((B->size[0] != 0) && (B->size[1] != 0))) {
    ptrdiff_t nrc_t;
    boolean_T overflow;
    nrc_t = (ptrdiff_t)B->size[0];
    nrc_t =
        LAPACKE_dormqr(102, 'L', 'T', nrc_t, (ptrdiff_t)B->size[1],
                       (ptrdiff_t)muIntScalarMin_sint32(A->size[0], A->size[1]),
                       (real_T *)&A_data[0], (ptrdiff_t)A->size[0],
                       (real_T *)&tau_data[0], &B_data[0], nrc_t);
    c_st.site = &un_emlrtRSI;
    if ((int32_T)nrc_t != 0) {
      boolean_T p;
      overflow = true;
      p = false;
      if ((int32_T)nrc_t == -7) {
        p = true;
      } else if ((int32_T)nrc_t == -9) {
        p = true;
      } else if ((int32_T)nrc_t == -10) {
        p = true;
      }
      if (!p) {
        if ((int32_T)nrc_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&c_st, &ob_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&c_st, &pb_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &fname[0], 12, (int32_T)nrc_t);
        }
      }
    } else {
      overflow = false;
    }
    if (overflow) {
      if (((int32_T)nrc_t == -10) && (B->size[1] > 1)) {
        int32_T b_nb;
        c_st.site = &vn_emlrtRSI;
        m = A->size[0];
        b_nb = B->size[1];
        mn = muIntScalarMin_sint32(A->size[0], A->size[1]);
        d_st.site = &wn_emlrtRSI;
        if (mn > 2147483646) {
          e_st.site = &vc_emlrtRSI;
          check_forloop_overflow_error(&e_st);
        }
        for (j = 0; j < mn; j++) {
          if (tau_data[j] != 0.0) {
            int32_T a_tmp;
            d_st.site = &xn_emlrtRSI;
            if (b_nb > 2147483646) {
              e_st.site = &vc_emlrtRSI;
              check_forloop_overflow_error(&e_st);
            }
            a_tmp = j + 2;
            overflow = ((j + 2 <= m) && (m > 2147483646));
            for (k = 0; k < b_nb; k++) {
              real_T wj;
              wj = B_data[j + B->size[0] * k];
              d_st.site = &yn_emlrtRSI;
              if (overflow) {
                e_st.site = &vc_emlrtRSI;
                check_forloop_overflow_error(&e_st);
              }
              for (i = a_tmp; i <= m; i++) {
                wj += A_data[(i + A->size[0] * j) - 1] *
                      B_data[(i + B->size[0] * k) - 1];
              }
              wj *= tau_data[j];
              if (wj != 0.0) {
                B_data[j + B->size[0] * k] -= wj;
                d_st.site = &ao_emlrtRSI;
                if ((j + 2 <= m) && (m > 2147483646)) {
                  e_st.site = &vc_emlrtRSI;
                  check_forloop_overflow_error(&e_st);
                }
                for (i = a_tmp; i <= m; i++) {
                  B_data[(i + B->size[0] * k) - 1] -=
                      A_data[(i + A->size[0] * j) - 1] * wj;
                }
              }
            }
          }
        }
      } else {
        int32_T b_nb;
        m = B->size[0];
        b_nb = B->size[1];
        mn = B->size[0] * B->size[1];
        B->size[0] = m;
        B->size[1] = b_nb;
        emxEnsureCapacity_real_T(&b_st, B, mn, &sj_emlrtRTEI);
        B_data = B->data;
        m *= b_nb;
        for (mn = 0; mn < m; mn++) {
          B_data[mn] = rtNaN;
        }
      }
    }
  }
  st.site = &qn_emlrtRSI;
  if (nb > 2147483646) {
    b_st.site = &vc_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (k = 0; k < nb; k++) {
    st.site = &rn_emlrtRSI;
    if (rankA > 2147483646) {
      b_st.site = &vc_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (i = 0; i < rankA; i++) {
      Y_data[(jpvt_data[i] + Y->size[0] * k) - 1] = B_data[i + B->size[0] * k];
    }
    for (j = rankA; j >= 1; j--) {
      mn = jpvt_data[j - 1];
      Y_data[(mn + Y->size[0] * k) - 1] /=
          A_data[(j + A->size[0] * (j - 1)) - 1];
      st.site = &sn_emlrtRSI;
      for (i = 0; i <= j - 2; i++) {
        Y_data[(jpvt_data[i] + Y->size[0] * k) - 1] -=
            Y_data[(jpvt_data[j - 1] + Y->size[0] * k) - 1] *
            A_data[i + A->size[0] * (j - 1)];
      }
    }
  }
}

/* End of code generation (qrsolve.c) */
