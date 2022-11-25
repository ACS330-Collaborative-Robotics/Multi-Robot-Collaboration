/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzsvdc.c
 *
 * Code generation for function 'xzsvdc'
 *
 */

/* Include files */
#include "xzsvdc.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xscal.h"
#include "xswap.h"
#include "blas.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo uk_emlrtRSI = {
    428,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo vk_emlrtRSI = {
    407,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo wk_emlrtRSI = {
    394,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo xk_emlrtRSI = {
    391,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo yk_emlrtRSI = {
    380,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo al_emlrtRSI = {
    353,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo bl_emlrtRSI = {
    351,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo cl_emlrtRSI = {
    334,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo dl_emlrtRSI = {
    251,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo el_emlrtRSI = {
    240,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo fl_emlrtRSI = {
    215,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo gl_emlrtRSI = {
    194,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo hl_emlrtRSI = {
    184,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo il_emlrtRSI = {
    120,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo jl_emlrtRSI = {
    114,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo kl_emlrtRSI = {
    94,       /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo ll_emlrtRSI = {
    82,       /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo ml_emlrtRSI = {
    77,       /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo pl_emlrtRSI = {
    21,                   /* lineNo */
    "scaleVectorByRecip", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "scaleVectorByRecip.m" /* pathName */
};

static emlrtRTEInfo mb_emlrtRTEI = {
    269,      /* lineNo */
    13,       /* colNo */
    "xzsvdc", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pName */
};

/* Function Definitions */
void xzsvdc(const emlrtStack *sp, real_T A[9], real_T U[9], real_T S[3],
            real_T V[9])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T e[3];
  real_T s[3];
  real_T work[3];
  real_T b;
  real_T nrm;
  real_T rt;
  real_T scale;
  real_T sm;
  real_T snorm;
  real_T sqds;
  int32_T ii;
  int32_T k;
  int32_T m;
  int32_T q;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  int32_T qs;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  memset(&U[0], 0, 9U * sizeof(real_T));
  memset(&V[0], 0, 9U * sizeof(real_T));
  for (q = 0; q < 2; q++) {
    boolean_T apply_transform;
    qp1 = q + 2;
    qs = q + 3 * q;
    qq = qs + 1;
    apply_transform = false;
    st.site = &ml_emlrtRSI;
    nrm = xnrm2(&st, 3 - q, A, qs + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (A[qs] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      st.site = &ll_emlrtRSI;
      if (muDoubleScalarAbs(nrm) >= 1.0020841800044864E-292) {
        b_st.site = &pl_emlrtRSI;
        xscal(&b_st, 3 - q, 1.0 / nrm, A, qs + 1);
      } else {
        qjj = (qs - q) + 3;
        for (k = qq; k <= qjj; k++) {
          A[k - 1] /= s[q];
        }
      }
      A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (k = qp1; k < 4; k++) {
      qjj = q + 3 * (k - 1);
      if (apply_transform) {
        st.site = &kl_emlrtRSI;
        nrm = xdotc(&st, 3 - q, A, qs + 1, A, qjj + 1);
        nrm = -(nrm / A[qs]);
        xaxpy(3 - q, nrm, qs + 1, A, qjj + 1);
      }
      e[k - 1] = A[qjj];
    }
    for (ii = q + 1; ii < 4; ii++) {
      qjj = (ii + 3 * q) - 1;
      U[qjj] = A[qjj];
    }
    if (q + 1 <= 1) {
      st.site = &jl_emlrtRSI;
      nrm = b_xnrm2(e);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        st.site = &il_emlrtRSI;
        sm = e[0];
        if (muDoubleScalarAbs(e[0]) >= 1.0020841800044864E-292) {
          c_xscal(1.0 / e[0], e);
        } else {
          for (k = qp1; k < 4; k++) {
            e[k - 1] /= sm;
          }
        }
        e[1]++;
        e[0] = -e[0];
        for (ii = qp1; ii < 4; ii++) {
          work[ii - 1] = 0.0;
        }
        for (k = qp1; k < 4; k++) {
          b_xaxpy(e[k - 1], A, 3 * (k - 1) + 2, work);
        }
        for (k = qp1; k < 4; k++) {
          c_xaxpy(-e[k - 1] / e[1], work, A, 3 * (k - 1) + 2);
        }
      }
      for (ii = qp1; ii < 4; ii++) {
        V[ii - 1] = e[ii - 1];
      }
    }
  }
  m = 1;
  s[2] = A[8];
  e[1] = A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (q = 1; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 3 * q;
    if (s[q] != 0.0) {
      for (k = qp1; k < 4; k++) {
        qjj = (q + 3 * (k - 1)) + 1;
        st.site = &hl_emlrtRSI;
        nrm = xdotc(&st, 3 - q, U, qq + 1, U, qjj);
        nrm = -(nrm / U[qq]);
        xaxpy(3 - q, nrm, qq + 1, U, qjj);
      }
      for (ii = q + 1; ii < 4; ii++) {
        qjj = (ii + 3 * q) - 1;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      st.site = &gl_emlrtRSI;
      if (q - 1 >= 0) {
        U[3 * q] = 0.0;
      }
    } else {
      U[3 * q] = 0.0;
      U[3 * q + 1] = 0.0;
      U[3 * q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }
  for (q = 2; q >= 0; q--) {
    if ((q + 1 <= 1) && (e[0] != 0.0)) {
      st.site = &fl_emlrtRSI;
      nrm = xdotc(&st, 2, V, 2, V, 5);
      nrm = -(nrm / V[1]);
      xaxpy(2, nrm, 2, V, 5);
      st.site = &fl_emlrtRSI;
      nrm = xdotc(&st, 2, V, 2, V, 8);
      nrm = -(nrm / V[1]);
      xaxpy(2, nrm, 2, V, 8);
    }
    V[3 * q] = 0.0;
    V[3 * q + 1] = 0.0;
    V[3 * q + 2] = 0.0;
    V[q + 3 * q] = 1.0;
  }
  if (s[0] != 0.0) {
    rt = muDoubleScalarAbs(s[0]);
    nrm = s[0] / rt;
    s[0] = rt;
    e[0] /= nrm;
    st.site = &el_emlrtRSI;
    b_xscal(&st, nrm, U, 1);
  }
  if (e[0] != 0.0) {
    rt = muDoubleScalarAbs(e[0]);
    nrm = rt / e[0];
    e[0] = rt;
    s[1] *= nrm;
    st.site = &dl_emlrtRSI;
    b_xscal(&st, nrm, V, 4);
  }
  if (s[1] != 0.0) {
    rt = muDoubleScalarAbs(s[1]);
    nrm = s[1] / rt;
    s[1] = rt;
    e[1] = A[7] / nrm;
    st.site = &el_emlrtRSI;
    b_xscal(&st, nrm, U, 4);
  }
  if (e[1] != 0.0) {
    rt = muDoubleScalarAbs(e[1]);
    nrm = rt / e[1];
    e[1] = rt;
    s[2] = A[8] * nrm;
    st.site = &dl_emlrtRSI;
    b_xscal(&st, nrm, V, 7);
  }
  if (s[2] != 0.0) {
    rt = muDoubleScalarAbs(s[2]);
    nrm = s[2] / rt;
    s[2] = rt;
    st.site = &el_emlrtRSI;
    b_xscal(&st, nrm, U, 7);
  }
  qp1 = 0;
  snorm = muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarMax(s[0], e[0]),
                                              muDoubleScalarMax(s[1], e[1])),
                            muDoubleScalarMax(s[2], 0.0));
  exitg1 = false;
  while ((!exitg1) && (m + 2 > 0)) {
    if (qp1 >= 75) {
      emlrtErrorWithMessageIdR2018a(sp, &mb_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    } else {
      boolean_T exitg2;
      qq = m + 1;
      ii = m + 1;
      exitg2 = false;
      while (!(exitg2 || (ii == 0))) {
        nrm = muDoubleScalarAbs(e[ii - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (muDoubleScalarAbs(s[ii - 1]) +
                                              muDoubleScalarAbs(s[ii]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((qp1 > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[ii - 1] = 0.0;
          exitg2 = true;
        } else {
          ii--;
        }
      }
      if (ii == m + 1) {
        qjj = 4;
      } else {
        qs = m + 2;
        qjj = m + 2;
        exitg2 = false;
        while ((!exitg2) && (qjj >= ii)) {
          qs = qjj;
          if (qjj == ii) {
            exitg2 = true;
          } else {
            nrm = 0.0;
            if (qjj < m + 2) {
              nrm = muDoubleScalarAbs(e[qjj - 1]);
            }
            if (qjj > ii + 1) {
              nrm += muDoubleScalarAbs(e[qjj - 2]);
            }
            rt = muDoubleScalarAbs(s[qjj - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              s[qjj - 1] = 0.0;
              exitg2 = true;
            } else {
              qjj--;
            }
          }
        }
        if (qs == ii) {
          qjj = 3;
        } else if (qs == m + 2) {
          qjj = 1;
        } else {
          qjj = 2;
          ii = qs;
        }
      }
      switch (qjj) {
      case 1:
        rt = e[m];
        e[m] = 0.0;
        for (k = qq; k >= ii + 1; k--) {
          st.site = &cl_emlrtRSI;
          sqds = 0.0;
          scale = 0.0;
          drotg(&s[k - 1], &rt, &sqds, &scale);
          if (k > ii + 1) {
            rt = -scale * e[0];
            e[0] *= sqds;
          }
          xrot(V, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sqds, scale);
        }
        break;
      case 2:
        rt = e[ii - 1];
        e[ii - 1] = 0.0;
        st.site = &bl_emlrtRSI;
        for (k = ii + 1; k <= m + 2; k++) {
          st.site = &al_emlrtRSI;
          sqds = 0.0;
          scale = 0.0;
          drotg(&s[k - 1], &rt, &sqds, &scale);
          nrm = e[k - 1];
          rt = -scale * nrm;
          e[k - 1] = nrm * sqds;
          xrot(U, 3 * (k - 1) + 1, 3 * (ii - 1) + 1, sqds, scale);
        }
        break;
      case 3:
        nrm = s[m + 1];
        scale = muDoubleScalarMax(
            muDoubleScalarMax(
                muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarAbs(nrm),
                                                    muDoubleScalarAbs(s[m])),
                                  muDoubleScalarAbs(e[m])),
                muDoubleScalarAbs(s[ii])),
            muDoubleScalarAbs(e[ii]));
        sm = nrm / scale;
        nrm = s[m] / scale;
        rt = e[m] / scale;
        sqds = s[ii] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          rt = b * b + nrm;
          st.site = &yk_emlrtRSI;
          if (rt < 0.0) {
            emlrtErrorWithMessageIdR2018a(
                &st, &hb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
                "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
          }
          rt = muDoubleScalarSqrt(rt);
          if (b < 0.0) {
            rt = -rt;
          }
          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }
        rt += (sqds + sm) * (sqds - sm);
        nrm = sqds * (e[ii] / scale);
        st.site = &xk_emlrtRSI;
        for (k = ii + 1; k <= qq; k++) {
          st.site = &wk_emlrtRSI;
          sqds = 0.0;
          scale = 0.0;
          drotg(&rt, &nrm, &sqds, &scale);
          if (k > ii + 1) {
            e[0] = rt;
          }
          nrm = e[k - 1];
          rt = s[k - 1];
          sm = sqds * rt + scale * nrm;
          e[k - 1] = sqds * nrm - scale * rt;
          nrm = s[k];
          b = scale * nrm;
          nrm *= sqds;
          xrot(V, 3 * (k - 1) + 1, 3 * k + 1, sqds, scale);
          st.site = &vk_emlrtRSI;
          sqds = 0.0;
          scale = 0.0;
          drotg(&sm, &b, &sqds, &scale);
          s[k - 1] = sm;
          rt = sqds * e[k - 1] + scale * nrm;
          nrm = -scale * e[k - 1] + sqds * nrm;
          s[k] = nrm;
          nrm = scale * e[k];
          e[k] *= sqds;
          xrot(U, 3 * (k - 1) + 1, 3 * k + 1, sqds, scale);
        }
        e[m] = rt;
        qp1++;
        break;
      default:
        if (s[ii] < 0.0) {
          s[ii] = -s[ii];
          st.site = &uk_emlrtRSI;
          b_xscal(&st, -1.0, V, 3 * ii + 1);
        }
        qp1 = ii + 1;
        while ((ii + 1 < 3) && (s[ii] < s[qp1])) {
          rt = s[ii];
          s[ii] = s[qp1];
          s[qp1] = rt;
          xswap(V, 3 * ii + 1, 3 * (ii + 1) + 1);
          xswap(U, 3 * ii + 1, 3 * (ii + 1) + 1);
          ii = qp1;
          qp1++;
        }
        qp1 = 0;
        m--;
        break;
      }
    }
  }
  S[0] = s[0];
  S[1] = s[1];
  S[2] = s[2];
}

/* End of code generation (xzsvdc.c) */
