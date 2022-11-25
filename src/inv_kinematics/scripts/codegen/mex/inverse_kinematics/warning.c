/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * warning.c
 *
 * Code generation for function 'warning'
 *
 */

/* Include files */
#include "warning.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtMCInfo b_emlrtMCI = {
    14,        /* lineNo */
    25,        /* colNo */
    "warning", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "warning.m" /* pName */
};

static emlrtMCInfo c_emlrtMCI = {
    14,        /* lineNo */
    9,         /* colNo */
    "warning", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "warning.m" /* pName */
};

static emlrtRSInfo lq_emlrtRSI = {
    14,        /* lineNo */
    "warning", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "warning.m" /* pathName */
};

/* Function Declarations */
static void b_feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location);

static const mxArray *c_feval(const emlrtStack *sp, const mxArray *m1,
                              const mxArray *m2, const mxArray *m3,
                              emlrtMCInfo *location);

static const mxArray *d_feval(const emlrtStack *sp, const mxArray *m1,
                              const mxArray *m2, const mxArray *m3,
                              const mxArray *m4, emlrtMCInfo *location);

static const mxArray *feval(const emlrtStack *sp, const mxArray *m1,
                            const mxArray *m2, emlrtMCInfo *location);

/* Function Definitions */
static void b_feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "feval",
                        true, location);
}

static const mxArray *c_feval(const emlrtStack *sp, const mxArray *m1,
                              const mxArray *m2, const mxArray *m3,
                              emlrtMCInfo *location)
{
  const mxArray *pArrays[3];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  pArrays[2] = m3;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 3, &pArrays[0],
                               "feval", true, location);
}

static const mxArray *d_feval(const emlrtStack *sp, const mxArray *m1,
                              const mxArray *m2, const mxArray *m3,
                              const mxArray *m4, emlrtMCInfo *location)
{
  const mxArray *pArrays[4];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  pArrays[2] = m3;
  pArrays[3] = m4;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 4, &pArrays[0],
                               "feval", true, location);
}

static const mxArray *feval(const emlrtStack *sp, const mxArray *m1,
                            const mxArray *m2, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 2, &pArrays[0],
                               "feval", true, location);
}

void b_warning(const emlrtStack *sp)
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 21};
  static const char_T msgID[21] = {'M', 'A', 'T', 'L', 'A', 'B', ':',
                                   's', 'i', 'n', 'g', 'u', 'l', 'a',
                                   'r', 'M', 'a', 't', 'r', 'i', 'x'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 21, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  st.site = &lq_emlrtRSI;
  b_feval(&st, y, feval(&st, b_y, c_y, &b_emlrtMCI), &c_emlrtMCI);
}

void c_warning(const emlrtStack *sp, const char_T varargin_1[14])
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 33};
  static const int32_T iv3[2] = {1, 14};
  static const char_T msgID[33] = {'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T',
                                   'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o',
                                   'n', 'd', 'i', 't', 'i', 'o', 'n', 'e', 'd',
                                   'M', 'a', 't', 'r', 'i', 'x'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 33, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  d_y = NULL;
  m = emlrtCreateCharArray(2, &iv3[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 14, m, &varargin_1[0]);
  emlrtAssign(&d_y, m);
  st.site = &lq_emlrtRSI;
  b_feval(&st, y, c_feval(&st, b_y, c_y, d_y, &b_emlrtMCI), &c_emlrtMCI);
}

void d_warning(const emlrtStack *sp)
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 65};
  static const char_T msgID[65] = {
      'r', 'o', 'b', 'o', 't', 'i', 'c', 's', ':', 'r', 'o', 'b', 'o',
      't', 'm', 'a', 'n', 'i', 'p', ':', 'i', 'n', 'v', 'e', 'r', 's',
      'e', 'k', 'i', 'n', 'e', 'm', 'a', 't', 'i', 'c', 's', ':', 'H',
      'o', 'm', 'o', 'g', 'e', 'n', 'e', 'o', 'u', 's', 'T', 'r', 'a',
      'n', 's', 'f', 'o', 'r', 'm', 'I', 'n', 'v', 'a', 'l', 'i', 'd'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 65, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  st.site = &lq_emlrtRSI;
  b_feval(&st, y, feval(&st, b_y, c_y, &b_emlrtMCI), &c_emlrtMCI);
}

void e_warning(const emlrtStack *sp)
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 72};
  static const char_T msgID[72] = {
      'r', 'o', 'b', 'o', 't', 'i', 'c', 's', ':', 'r', 'o', 'b', 'o', 't', 'm',
      'a', 'n', 'i', 'p', ':', 'r', 'i', 'g', 'i', 'd', 'b', 'o', 'd', 'y', 't',
      'r', 'e', 'e', ':', 'C', 'o', 'n', 'f', 'i', 'g', 'J', 'o', 'i', 'n', 't',
      'L', 'i', 'm', 'i', 't', 's', 'V', 'i', 'o', 'l', 'a', 't', 'i', 'o', 'n',
      'A', 'u', 't', 'o', 'A', 'd', 'j', 'u', 's', 't', 'e', 'd'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 72, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  st.site = &lq_emlrtRSI;
  b_feval(&st, y, feval(&st, b_y, c_y, &b_emlrtMCI), &c_emlrtMCI);
}

void f_warning(const emlrtStack *sp, int32_T varargin_1,
               const char_T varargin_2[14])
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 32};
  static const int32_T iv3[2] = {1, 14};
  static const char_T msgID[32] = {'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
                                   'T', 'L', 'A', 'B', ':', 'r', 'a', 'n',
                                   'k', 'D', 'e', 'f', 'i', 'c', 'i', 'e',
                                   'n', 't', 'M', 'a', 't', 'r', 'i', 'x'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 32, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  d_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
  *(int32_T *)emlrtMxGetData(m) = varargin_1;
  emlrtAssign(&d_y, m);
  e_y = NULL;
  m = emlrtCreateCharArray(2, &iv3[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 14, m, &varargin_2[0]);
  emlrtAssign(&e_y, m);
  st.site = &lq_emlrtRSI;
  b_feval(&st, y, d_feval(&st, b_y, c_y, d_y, e_y, &b_emlrtMCI), &c_emlrtMCI);
}

void warning(const emlrtStack *sp)
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 38};
  static const char_T msgID[38] = {
      'r', 'o', 'b', 'o', 't', 'i', 'c', 's', ':', 'r', 'o', 'b', 'o',
      't', 'm', 'a', 'n', 'i', 'p', ':', 'j', 'o', 'i', 'n', 't', ':',
      'J', 'o', 'i', 'n', 't', 'N', 'o', 't', 'F', 'r', 'e', 'e'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 38, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  st.site = &lq_emlrtRSI;
  b_feval(&st, y, feval(&st, b_y, c_y, &b_emlrtMCI), &c_emlrtMCI);
}

/* End of code generation (warning.c) */
