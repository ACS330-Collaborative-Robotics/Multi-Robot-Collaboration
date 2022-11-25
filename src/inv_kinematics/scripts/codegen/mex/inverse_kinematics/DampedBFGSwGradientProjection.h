/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * DampedBFGSwGradientProjection.h
 *
 * Code generation for function 'DampedBFGSwGradientProjection'
 *
 */

#pragma once

/* Include files */
#include "inverse_kinematics_internal_types.h"
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
void c_DampedBFGSwGradientProjection(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *obj,
                                     emxArray_real_T *xSol,
                                     c_robotics_core_internal_NLPSol *exitFlag,
                                     real_T *err, real_T *iter);

/* End of code generation (DampedBFGSwGradientProjection.h) */
