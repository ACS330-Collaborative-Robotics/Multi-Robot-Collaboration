/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sqrt1.c
 *
 * Code generation for function 'sqrt1'
 *
 */

/* Include files */
#include "sqrt1.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo lo_emlrtRSI = {
    16,                                                            /* lineNo */
    "sqrt",                                                        /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elfun/sqrt.m" /* pathName
                                                                    */
};

static emlrtRSInfo mo_emlrtRSI = {
    33,                           /* lineNo */
    "applyScalarFunctionInPlace", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "applyScalarFunctionInPlace.m" /* pathName */
};

/* Function Definitions */
void c_sqrt(const emlrtStack *sp, emxArray_real_T *x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T *x_data;
  int32_T k;
  int32_T nx;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  p = false;
  nx = x->size[0];
  for (k = 0; k < nx; k++) {
    if (p || (x_data[k] < 0.0)) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        sp, &hb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  st.site = &lo_emlrtRSI;
  nx = x->size[0];
  b_st.site = &mo_emlrtRSI;
  if (x->size[0] > 2147483646) {
    c_st.site = &vc_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = 0; k < nx; k++) {
    x_data[k] = muDoubleScalarSqrt(x_data[k]);
  }
}

/* End of code generation (sqrt1.c) */
