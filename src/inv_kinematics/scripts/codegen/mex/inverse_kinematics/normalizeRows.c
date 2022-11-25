/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * normalizeRows.c
 *
 * Code generation for function 'normalizeRows'
 *
 */

/* Include files */
#include "normalizeRows.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo pj_emlrtRSI = {
    15,              /* lineNo */
    "normalizeRows", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+internal/normalizeRows.m" /* pathName */
};

/* Function Definitions */
void normalizeRows(const emlrtStack *sp, const real_T matrix[3],
                   real_T normRowMatrix[3])
{
  emlrtStack st;
  real_T y[3];
  real_T x;
  st.prev = sp;
  st.tls = sp->tls;
  y[0] = matrix[0] * matrix[0];
  y[1] = matrix[1] * matrix[1];
  y[2] = matrix[2] * matrix[2];
  x = sumColumnB(y);
  st.site = &pj_emlrtRSI;
  if (x < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &hb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  x = muDoubleScalarSqrt(x);
  x = 1.0 / x;
  normRowMatrix[0] = matrix[0] * x;
  normRowMatrix[1] = matrix[1] * x;
  normRowMatrix[2] = matrix[2] * x;
}

/* End of code generation (normalizeRows.c) */
