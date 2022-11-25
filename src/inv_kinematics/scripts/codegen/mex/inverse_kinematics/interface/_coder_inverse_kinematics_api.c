/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_inverse_kinematics_api.c
 *
 * Code generation for function '_coder_inverse_kinematics_api'
 *
 */

/* Include files */
#include "_coder_inverse_kinematics_api.h"
#include "inverse_kinematics.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                 const char_T *identifier);

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const real_T u[6]);

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[6])
{
  static const int32_T b_iv[2] = {0, 0};
  static const int32_T iv1[2] = {1, 6};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&b_iv[0], mxDOUBLE_CLASS,
                              mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void inverse_kinematics_api(const mxArray *const prhs[6], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*joints)[6];
  real_T a;
  real_T b;
  real_T c;
  real_T x;
  real_T y;
  real_T z;
  st.tls = emlrtRootTLSGlobal;
  joints = (real_T(*)[6])mxMalloc(sizeof(real_T[6]));
  /* Marshall function inputs */
  x = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "x");
  y = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "y");
  z = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "z");
  a = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "a");
  b = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "b");
  c = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "c");
  /* Invoke the target function */
  inverse_kinematics(&st, x, y, z, a, b, c, *joints);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*joints);
}

/* End of code generation (_coder_inverse_kinematics_api.c) */
