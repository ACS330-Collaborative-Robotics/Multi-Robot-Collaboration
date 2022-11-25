/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xaxpy.c
 *
 * Code generation for function 'xaxpy'
 *
 */

/* Include files */
#include "xaxpy.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_xaxpy(real_T a, const real_T x[9], int32_T ix0, real_T y[3])
{
  int32_T k;
  if (!(a == 0.0)) {
    for (k = 0; k < 2; k++) {
      y[k + 1] += a * x[(ix0 + k) - 1];
    }
  }
}

void c_xaxpy(real_T a, const real_T x[3], real_T y[9], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    for (k = 0; k < 2; k++) {
      int32_T i;
      i = (iy0 + k) - 1;
      y[i] += a * x[k + 1];
    }
  }
}

void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T i;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      int32_T i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

/* End of code generation (xaxpy.c) */
