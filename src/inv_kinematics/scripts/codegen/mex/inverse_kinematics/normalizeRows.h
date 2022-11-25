/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * normalizeRows.h
 *
 * Code generation for function 'normalizeRows'
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
void normalizeRows(const emlrtStack *sp, const real_T matrix[3],
                   real_T normRowMatrix[3]);

/* End of code generation (normalizeRows.h) */
