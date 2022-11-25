/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eye.h
 *
 * Code generation for function 'eye'
 *
 */

#pragma once

/* Include files */
#include "inverse_kinematics_types.h"
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_eye(const emlrtStack *sp, const real_T varargin_1[2],
           emxArray_real_T *b_I);

void eye(const emlrtStack *sp, real_T varargin_1, emxArray_real_T *b_I);

/* End of code generation (eye.h) */
