/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * IKHelpers.h
 *
 * Code generation for function 'IKHelpers'
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
void IKHelpers_computeCost(const emlrtStack *sp, const emxArray_real_T *x,
                           c_robotics_manip_internal_IKExt *args, real_T *cost,
                           real_T W[36], emxArray_real_T *Jac,
                           c_robotics_manip_internal_IKExt **b_args);

real_T IKHelpers_evaluateSolution(const emlrtStack *sp,
                                  const c_robotics_manip_internal_IKExt *args);

/* End of code generation (IKHelpers.h) */
