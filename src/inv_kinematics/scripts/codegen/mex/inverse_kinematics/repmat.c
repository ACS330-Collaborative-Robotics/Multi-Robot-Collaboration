/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo ue_emlrtRSI = {
    28,       /* lineNo */
    "repmat", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/repmat.m" /* pathName
                                                                      */
};

static emlrtDCInfo k_emlrtDCI = {
    31,       /* lineNo */
    14,       /* colNo */
    "repmat", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/repmat.m", /* pName
                                                                       */
    4 /* checkKind */
};

static emlrtRTEInfo ud_emlrtRTEI = {
    53,       /* lineNo */
    9,        /* colNo */
    "repmat", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/repmat.m" /* pName */
};

/* Function Definitions */
void b_repmat(const emlrtStack *sp, ptrdiff_t a, int32_T varargin_1,
              emxArray_ptrdiff_t *b)
{
  ptrdiff_t *b_data;
  int32_T i;
  i = b->size[0];
  b->size[0] = varargin_1;
  emxEnsureCapacity_ptrdiff_t(sp, b, i, &ud_emlrtRTEI);
  b_data = b->data;
  for (i = 0; i < varargin_1; i++) {
    b_data[i] = a;
  }
}

void repmat(const emlrtStack *sp, real_T varargin_2, c_emxArray_struct_T *b)
{
  static const d_struct_T r = {
      0.0 /* JointPosition */
  };
  d_struct_T *b_data;
  emlrtStack st;
  real_T d;
  int32_T i;
  int32_T loop_ub_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ue_emlrtRSI;
  if ((varargin_2 != muDoubleScalarFloor(varargin_2)) ||
      muDoubleScalarIsInf(varargin_2) || (varargin_2 < -2.147483648E+9) ||
      (varargin_2 > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &o_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (varargin_2 <= 0.0) {
    d = 0.0;
  } else {
    d = varargin_2;
  }
  if (!(d <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &n_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  if (!(varargin_2 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(varargin_2, &k_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  loop_ub_tmp = (int32_T)varargin_2;
  b->size[1] = (int32_T)varargin_2;
  emxEnsureCapacity_struct_T2(sp, b, i, &ud_emlrtRTEI);
  b_data = b->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    b_data[i] = r;
  }
}

/* End of code generation (repmat.c) */
