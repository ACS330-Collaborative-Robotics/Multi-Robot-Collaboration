/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics.h
 *
 * Code generation for function 'inverse_kinematics'
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
void inverse_kinematics(const emlrtStack *sp, real_T x, real_T y, real_T z,
                        real_T a, real_T b, real_T c, real_T joints[6]);

/* End of code generation (inverse_kinematics.h) */
