/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * strcmp.c
 *
 * Code generation for function 'strcmp'
 *
 */

/* Include files */
#include "strcmp.h"
#include "eml_int_forloop_overflow_check.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo uc_emlrtRSI = {
    233,       /* lineNo */
    "charcmp", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/strcmp.m" /* pathName
                                                                          */
};

/* Function Definitions */
boolean_T b_strcmp(const emlrtStack *sp, const emxArray_char_T *a,
                   const emxArray_char_T *b)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int32_T nb;
  const char_T *a_data;
  const char_T *b_data;
  boolean_T b_b;
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_data = b->data;
  a_data = a->data;
  st.site = &sc_emlrtRSI;
  b_st.site = &tc_emlrtRSI;
  b_bool = false;
  nb = b->size[1];
  b_b = (a->size[1] == 0);
  if (b_b && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] == b->size[1]) {
    int32_T kstr;
    c_st.site = &uc_emlrtRSI;
    if (nb > 2147483646) {
      d_st.site = &vc_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    kstr = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (kstr <= nb - 1) {
        if (a_data[kstr] != b_data[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return b_bool;
}

/* End of code generation (strcmp.c) */
