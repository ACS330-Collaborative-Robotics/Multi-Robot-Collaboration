/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inv.c
 *
 * Code generation for function 'inv'
 *
 */

/* Include files */
#include "inv.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_mexutil.h"
#include "inverse_kinematics_types.h"
#include "norm.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "blas.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo om_emlrtRSI =
    {
        27,       /* lineNo */
        "xgetrf", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pathName */
};

static emlrtRSInfo pm_emlrtRSI =
    {
        91,             /* lineNo */
        "ceval_xgetrf", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pathName */
};

static emlrtRSInfo qm_emlrtRSI =
    {
        58,             /* lineNo */
        "ceval_xgetrf", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pathName */
};

static emlrtRSInfo sm_emlrtRSI = {
    82,                                                           /* lineNo */
    "colon",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

static emlrtRSInfo tm_emlrtRSI = {
    148,                                                          /* lineNo */
    "eml_integer_colon_dispatcher",                               /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

static emlrtRSInfo um_emlrtRSI = {
    171,                                                          /* lineNo */
    "eml_signed_integer_colon",                                   /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

static emlrtRSInfo vm_emlrtRSI = {
    176,                                                          /* lineNo */
    "eml_signed_integer_colon",                                   /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

static emlrtRSInfo ym_emlrtRSI =
    {
        67,      /* lineNo */
        "xtrsm", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+blas/"
        "xtrsm.m" /* pathName */
};

static emlrtRSInfo an_emlrtRSI =
    {
        81,           /* lineNo */
        "xtrsm_blas", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+blas/"
        "xtrsm.m" /* pathName */
};

static emlrtRSInfo co_emlrtRSI = {
    21,                                                            /* lineNo */
    "inv",                                                         /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo do_emlrtRSI = {
    22,                                                            /* lineNo */
    "inv",                                                         /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo eo_emlrtRSI = {
    173,                                                           /* lineNo */
    "invNxN",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo fo_emlrtRSI = {
    174,                                                           /* lineNo */
    "invNxN",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo go_emlrtRSI = {
    177,                                                           /* lineNo */
    "invNxN",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo ho_emlrtRSI = {
    180,                                                           /* lineNo */
    "invNxN",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo io_emlrtRSI = {
    183,                                                           /* lineNo */
    "invNxN",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo jo_emlrtRSI = {
    190,                                                           /* lineNo */
    "invNxN",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo ko_emlrtRSI = {
    14,              /* lineNo */
    "eml_ipiv2perm", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/private/"
    "eml_ipiv2perm.m" /* pathName */
};

static emlrtRTEInfo qb_emlrtRTEI = {
    14,                                                            /* lineNo */
    15,                                                            /* colNo */
    "inv",                                                         /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pName */
};

static emlrtRTEInfo uj_emlrtRTEI = {
    19,                                                            /* lineNo */
    5,                                                             /* colNo */
    "inv",                                                         /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pName */
};

static emlrtRTEInfo vj_emlrtRTEI = {
    21,                                                            /* lineNo */
    5,                                                             /* colNo */
    "inv",                                                         /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pName */
};

static emlrtRTEInfo wj_emlrtRTEI =
    {
        1,        /* lineNo */
        37,       /* colNo */
        "xgetrf", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pName */
};

static emlrtRTEInfo xj_emlrtRTEI =
    {
        58,       /* lineNo */
        29,       /* colNo */
        "xgetrf", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pName */
};

static emlrtRTEInfo yj_emlrtRTEI =
    {
        89,       /* lineNo */
        27,       /* colNo */
        "xgetrf", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pName */
};

static emlrtRTEInfo ak_emlrtRTEI = {
    172,                                                          /* lineNo */
    20,                                                           /* colNo */
    "colon",                                                      /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pName */
};

static emlrtRTEInfo bk_emlrtRTEI = {
    174,                                                           /* lineNo */
    1,                                                             /* colNo */
    "inv",                                                         /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pName */
};

static emlrtRTEInfo ck_emlrtRTEI = {
    1,                                                             /* lineNo */
    14,                                                            /* colNo */
    "inv",                                                         /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pName */
};

static emlrtRTEInfo dk_emlrtRTEI =
    {
        58,       /* lineNo */
        5,        /* colNo */
        "xgetrf", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pName */
};

/* Function Definitions */
void inv(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y)
{
  static const int32_T b_iv[2] = {1, 6};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  ptrdiff_t info_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t n_t;
  ptrdiff_t *ipiv_t_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  emxArray_int32_T *ipiv;
  emxArray_int32_T *p;
  emxArray_ptrdiff_t *ipiv_t;
  emxArray_ptrdiff_t *r;
  emxArray_real_T *b_x;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const real_T *x_data;
  real_T n1x;
  real_T *b_x_data;
  real_T *y_data;
  int32_T b_i;
  int32_T b_n;
  int32_T i;
  int32_T k;
  int32_T *ipiv_data;
  int32_T *p_data;
  char_T DIAGA1;
  char_T SIDE1;
  char_T TRANSA1;
  char_T UPLO1;
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (x->size[0] != x->size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &qb_emlrtRTEI, "Coder:MATLAB:square",
                                  "Coder:MATLAB:square", 0);
  }
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    int32_T yk;
    i = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(sp, y, i, &uj_emlrtRTEI);
    y_data = y->data;
    yk = x->size[0] * x->size[1];
    for (i = 0; i < yk; i++) {
      y_data[i] = x_data[i];
    }
  } else {
    real_T n1xinv;
    real_T rc;
    int32_T n;
    int32_T yk;
    st.site = &co_emlrtRSI;
    n = x->size[0];
    i = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&st, y, i, &vj_emlrtRTEI);
    y_data = y->data;
    yk = x->size[0] * x->size[1];
    for (i = 0; i < yk; i++) {
      y_data[i] = 0.0;
    }
    b_st.site = &eo_emlrtRSI;
    emxInit_real_T(&b_st, &b_x, 2, &ck_emlrtRTEI);
    i = b_x->size[0] * b_x->size[1];
    b_x->size[0] = x->size[0];
    b_x->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&b_st, b_x, i, &wj_emlrtRTEI);
    b_x_data = b_x->data;
    yk = x->size[0] * x->size[1];
    for (i = 0; i < yk; i++) {
      b_x_data[i] = x_data[i];
    }
    c_st.site = &om_emlrtRSI;
    emxInit_ptrdiff_t(&c_st, &r, &xj_emlrtRTEI);
    d_st.site = &qm_emlrtRSI;
    b_repmat(&d_st, (ptrdiff_t)0.0, muIntScalarMin_sint32(n, n), r);
    emxInit_ptrdiff_t(&c_st, &ipiv_t, &dk_emlrtRTEI);
    i = ipiv_t->size[0];
    ipiv_t->size[0] = r->size[0];
    emxFree_ptrdiff_t(&c_st, &r);
    emxEnsureCapacity_ptrdiff_t(&c_st, ipiv_t, i, &xj_emlrtRTEI);
    ipiv_t_data = ipiv_t->data;
    info_t = LAPACKE_dgetrf_work(102, (ptrdiff_t)x->size[0],
                                 (ptrdiff_t)x->size[0], &b_x_data[0],
                                 (ptrdiff_t)x->size[0], &ipiv_t_data[0]);
    emxInit_int32_T(&c_st, &ipiv, 2, &ck_emlrtRTEI);
    i = ipiv->size[0] * ipiv->size[1];
    ipiv->size[0] = 1;
    ipiv->size[1] = ipiv_t->size[0];
    emxEnsureCapacity_int32_T(&c_st, ipiv, i, &yj_emlrtRTEI);
    ipiv_data = ipiv->data;
    d_st.site = &pm_emlrtRSI;
    if ((int32_T)info_t < 0) {
      if ((int32_T)info_t == -1010) {
        emlrtErrorWithMessageIdR2018a(&d_st, &ob_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&d_st, &pb_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      19, &cv2[0], 12, (int32_T)info_t);
      }
    }
    i = ipiv_t->size[0] - 1;
    for (k = 0; k <= i; k++) {
      ipiv_data[k] = (int32_T)ipiv_t_data[k];
    }
    emxFree_ptrdiff_t(&c_st, &ipiv_t);
    b_st.site = &fo_emlrtRSI;
    c_st.site = &ko_emlrtRSI;
    d_st.site = &rm_emlrtRSI;
    e_st.site = &sm_emlrtRSI;
    f_st.site = &tm_emlrtRSI;
    g_st.site = &um_emlrtRSI;
    b_n = x->size[0];
    emxInit_int32_T(&f_st, &p, 2, &bk_emlrtRTEI);
    i = p->size[0] * p->size[1];
    p->size[0] = 1;
    p->size[1] = x->size[0];
    emxEnsureCapacity_int32_T(&f_st, p, i, &ak_emlrtRTEI);
    p_data = p->data;
    p_data[0] = 1;
    yk = 1;
    g_st.site = &vm_emlrtRSI;
    if (x->size[0] > 2147483646) {
      h_st.site = &vc_emlrtRSI;
      check_forloop_overflow_error(&h_st);
    }
    for (k = 2; k <= b_n; k++) {
      yk++;
      p_data[k - 1] = yk;
    }
    i = ipiv->size[1];
    for (k = 0; k < i; k++) {
      yk = ipiv_data[k];
      if (yk > k + 1) {
        b_n = p_data[yk - 1];
        p_data[yk - 1] = p_data[k];
        p_data[k] = b_n;
      }
    }
    emxFree_int32_T(&b_st, &ipiv);
    b_st.site = &go_emlrtRSI;
    for (k = 0; k < n; k++) {
      i = p_data[k];
      y_data[k + y->size[0] * (i - 1)] = 1.0;
      b_st.site = &ho_emlrtRSI;
      if ((k + 1 <= n) && (n > 2147483646)) {
        c_st.site = &vc_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (b_n = k + 1; b_n <= n; b_n++) {
        if (y_data[(b_n + y->size[0] * (i - 1)) - 1] != 0.0) {
          yk = b_n + 1;
          b_st.site = &io_emlrtRSI;
          for (b_i = yk; b_i <= n; b_i++) {
            y_data[(b_i + y->size[0] * (i - 1)) - 1] -=
                y_data[(b_n + y->size[0] * (i - 1)) - 1] *
                b_x_data[(b_i + b_x->size[0] * (b_n - 1)) - 1];
          }
        }
      }
    }
    emxFree_int32_T(&st, &p);
    b_st.site = &jo_emlrtRSI;
    c_st.site = &ym_emlrtRSI;
    d_st.site = &an_emlrtRSI;
    n1x = 1.0;
    DIAGA1 = 'N';
    TRANSA1 = 'N';
    UPLO1 = 'U';
    SIDE1 = 'L';
    info_t = (ptrdiff_t)x->size[0];
    n_t = (ptrdiff_t)x->size[0];
    lda_t = (ptrdiff_t)x->size[0];
    ldb_t = (ptrdiff_t)x->size[0];
    dtrsm(&SIDE1, &UPLO1, &TRANSA1, &DIAGA1, &info_t, &n_t, &n1x, &b_x_data[0],
          &lda_t, &y_data[0], &ldb_t);
    emxFree_real_T(&d_st, &b_x);
    st.site = &do_emlrtRSI;
    n1x = d_norm(x);
    n1xinv = d_norm(y);
    rc = 1.0 / (n1x * n1xinv);
    if ((n1x == 0.0) || (n1xinv == 0.0) || (rc == 0.0)) {
      b_st.site = &lf_emlrtRSI;
      b_warning(&b_st);
    } else if (muDoubleScalarIsNaN(rc) || (rc < 2.2204460492503131E-16)) {
      char_T str[14];
      b_st.site = &mf_emlrtRSI;
      b_y = NULL;
      m = emlrtCreateCharArray(2, &b_iv[0]);
      emlrtInitCharArrayR2013a(&b_st, 6, m, &rfmt[0]);
      emlrtAssign(&b_y, m);
      c_y = NULL;
      m = emlrtCreateDoubleScalar(rc);
      emlrtAssign(&c_y, m);
      c_st.site = &mq_emlrtRSI;
      emlrt_marshallIn(&c_st, b_sprintf(&c_st, b_y, c_y, &d_emlrtMCI),
                       "<output of sprintf>", str);
      b_st.site = &mf_emlrtRSI;
      c_warning(&b_st, str);
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (inv.c) */
