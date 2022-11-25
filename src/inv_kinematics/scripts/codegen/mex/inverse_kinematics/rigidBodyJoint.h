/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rigidBodyJoint.h
 *
 * Code generation for function 'rigidBodyJoint'
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
void b_minus(const emlrtStack *sp, emxArray_real_T *in1,
             const emxArray_real_T *in2);

void c_rigidBodyJoint_transformBodyT(const emlrtStack *sp,
                                     const rigidBodyJoint *obj,
                                     const emxArray_real_T *q, real_T T[16]);

void minus(const emlrtStack *sp, emxArray_real_T *in1,
           const emxArray_real_T *in2);

void plus(const emlrtStack *sp, emxArray_real_T *in1,
          const emxArray_real_T *in2);

void rigidBodyJoint_get_JointAxis(const rigidBodyJoint *obj, real_T ax[3]);

rigidBodyJoint *rigidBodyJoint_rigidBodyJoint(const emlrtStack *sp,
                                              rigidBodyJoint *obj,
                                              const emxArray_char_T *jname);

void times(const emlrtStack *sp, emxArray_real_T *in1,
           const emxArray_real_T *in2);

/* End of code generation (rigidBodyJoint.h) */
