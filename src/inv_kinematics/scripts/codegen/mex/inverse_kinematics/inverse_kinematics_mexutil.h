/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_mexutil.h
 *
 * Code generation for function 'inverse_kinematics_mexutil'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                        const emlrtMsgIdentifier *parentId, char_T y[14]);

const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *m1,
                         const mxArray *m2, emlrtMCInfo *location);

void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                        const emlrtMsgIdentifier *msgId, char_T ret[14]);

void emlrt_marshallIn(const emlrtStack *sp,
                      const mxArray *a__output_of_sprintf_,
                      const char_T *identifier, char_T y[14]);

/* End of code generation (inverse_kinematics_mexutil.h) */
