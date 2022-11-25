/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * validatestring.h
 *
 * Code generation for function 'validatestring'
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
void validatestring(const emlrtStack *sp, const emxArray_char_T *str,
                    char_T out_data[], int32_T out_size[2]);

/* End of code generation (validatestring.h) */
