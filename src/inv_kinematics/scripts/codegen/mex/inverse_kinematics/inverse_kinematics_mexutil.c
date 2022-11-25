/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_mexutil.c
 *
 * Code generation for function 'inverse_kinematics_mexutil'
 *
 */

/* Include files */
#include "inverse_kinematics_mexutil.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                        const emlrtMsgIdentifier *parentId, char_T y[14])
{
  e_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *m1,
                         const mxArray *m2, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 2, &pArrays[0],
                               "sprintf", true, location);
}

void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                        const emlrtMsgIdentifier *msgId, char_T ret[14])
{
  static const int32_T dims[2] = {1, 14};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 14);
  emlrtDestroyArray(&src);
}

void emlrt_marshallIn(const emlrtStack *sp,
                      const mxArray *a__output_of_sprintf_,
                      const char_T *identifier, char_T y[14])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

/* End of code generation (inverse_kinematics_mexutil.c) */
