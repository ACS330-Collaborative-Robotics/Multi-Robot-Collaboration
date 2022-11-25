/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * abs.c
 *
 * Code generation for function 'abs'
 *
 */

/* Include files */
#include "abs.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo vp_emlrtRSI = {
    18,                                                           /* lineNo */
    "abs",                                                        /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elfun/abs.m" /* pathName */
};

static emlrtRSInfo wp_emlrtRSI = {
    74,                    /* lineNo */
    "applyScalarFunction", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "applyScalarFunction.m" /* pathName */
};

static emlrtRTEInfo kk_emlrtRTEI = {
    30,                    /* lineNo */
    21,                    /* colNo */
    "applyScalarFunction", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "applyScalarFunction.m" /* pName */
};

/* Function Definitions */
void b_abs(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  const real_T *x_data;
  real_T *y_data;
  int32_T k;
  int32_T nx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  st.site = &vp_emlrtRSI;
  nx = x->size[0];
  k = y->size[0];
  y->size[0] = x->size[0];
  emxEnsureCapacity_real_T(&st, y, k, &kk_emlrtRTEI);
  y_data = y->data;
  b_st.site = &wp_emlrtRSI;
  if (x->size[0] > 2147483646) {
    c_st.site = &vc_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = 0; k < nx; k++) {
    y_data[k] = muDoubleScalarAbs(x_data[k]);
  }
}

/* End of code generation (abs.c) */
