/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverseKinematics.h
 *
 * Code generation for function 'inverseKinematics'
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
inverseKinematics *c_inverseKinematics_inverseKine(const emlrtStack *sp,
                                                   inverseKinematics *obj,
                                                   b_rigidBodyTree *varargin_2);

void inverseKinematics_stepImpl(
    const emlrtStack *sp, inverseKinematics *obj, const real_T tform[16],
    const emxArray_struct_T *initialGuess, emxArray_struct_T *QSol,
    real_T *solutionInfo_Iterations, real_T *solutionInfo_NumRandomRestarts,
    real_T *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag,
    char_T solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2]);

/* End of code generation (inverseKinematics.h) */
