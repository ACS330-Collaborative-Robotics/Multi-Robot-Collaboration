/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * any.c
 *
 * Code generation for function 'any'
 *
 */

/* Include files */
#include "any.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo xo_emlrtRSI = {
    13,                                                         /* lineNo */
    "any",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/any.m" /* pathName */
};

/* Function Definitions */
boolean_T any(const emlrtStack *sp, const emxArray_boolean_T *x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T ix;
  const boolean_T *x_data;
  boolean_T exitg1;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  st.site = &xo_emlrtRSI;
  y = false;
  b_st.site = &mg_emlrtRSI;
  if (x->size[0] > 2147483646) {
    c_st.site = &vc_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    if (x_data[ix - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

/* End of code generation (any.c) */
