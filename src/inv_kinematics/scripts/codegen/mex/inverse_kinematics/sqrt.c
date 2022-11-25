/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sqrt.c
 *
 * Code generation for function 'sqrt'
 *
 */

/* Include files */
#include "sqrt.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void b_sqrt(creal_T *x)
{
  real_T absxi;
  real_T absxr;
  real_T xi;
  real_T xr;
  xr = x->re;
  xi = x->im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      absxr = 0.0;
      absxi = muDoubleScalarSqrt(-xr);
    } else {
      absxr = muDoubleScalarSqrt(xr);
      absxi = 0.0;
    }
  } else if (xr == 0.0) {
    if (xi < 0.0) {
      absxr = muDoubleScalarSqrt(-xi / 2.0);
      absxi = -absxr;
    } else {
      absxr = muDoubleScalarSqrt(xi / 2.0);
      absxi = absxr;
    }
  } else if (muDoubleScalarIsNaN(xr)) {
    absxr = rtNaN;
    absxi = rtNaN;
  } else if (muDoubleScalarIsNaN(xi)) {
    absxr = rtNaN;
    absxi = rtNaN;
  } else if (muDoubleScalarIsInf(xi)) {
    absxr = muDoubleScalarAbs(xi);
    absxi = xi;
  } else if (muDoubleScalarIsInf(xr)) {
    if (xr < 0.0) {
      absxr = 0.0;
      absxi = xi * -xr;
    } else {
      absxr = xr;
      absxi = 0.0;
    }
  } else {
    absxr = muDoubleScalarAbs(xr);
    absxi = muDoubleScalarAbs(xi);
    if ((absxr > 4.4942328371557893E+307) ||
        (absxi > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      absxi = muDoubleScalarHypot(absxr, absxi * 0.5);
      if (absxi > absxr) {
        absxr =
            muDoubleScalarSqrt(absxi) * muDoubleScalarSqrt(absxr / absxi + 1.0);
      } else {
        absxr = muDoubleScalarSqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxr =
          muDoubleScalarSqrt((muDoubleScalarHypot(absxr, absxi) + absxr) * 0.5);
    }
    if (xr > 0.0) {
      absxi = 0.5 * (xi / absxr);
    } else {
      if (xi < 0.0) {
        absxi = -absxr;
      } else {
        absxi = absxr;
      }
      absxr = 0.5 * (xi / absxi);
    }
  }
  x->re = absxr;
  x->im = absxi;
}

/* End of code generation (sqrt.c) */
