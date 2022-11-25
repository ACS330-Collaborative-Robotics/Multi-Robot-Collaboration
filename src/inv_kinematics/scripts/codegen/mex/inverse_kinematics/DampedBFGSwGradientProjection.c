/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * DampedBFGSwGradientProjection.c
 *
 * Code generation for function 'DampedBFGSwGradientProjection'
 *
 */

/* Include files */
#include "DampedBFGSwGradientProjection.h"
#include "IKHelpers.h"
#include "all.h"
#include "any.h"
#include "assertCompatibleDims.h"
#include "diag.h"
#include "div.h"
#include "eml_int_forloop_overflow_check.h"
#include "eml_mtimes_helper.h"
#include "eye.h"
#include "find.h"
#include "inv.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_internal_types.h"
#include "inverse_kinematics_types.h"
#include "isPositiveDefinite.h"
#include "mldivide.h"
#include "mtimes.h"
#include "norm.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "sqrt1.h"
#include "tic.h"
#include "toc.h"
#include "blas.h"
#include "emlrt.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo eh_emlrtRSI = {
    185,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo fh_emlrtRSI = {
    191,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo gh_emlrtRSI = {
    192,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo hh_emlrtRSI = {
    196,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ih_emlrtRSI = {
    200,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo jh_emlrtRSI = {
    212,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo kh_emlrtRSI = {
    213,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo lh_emlrtRSI = {
    223,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo mh_emlrtRSI = {
    226,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo nh_emlrtRSI = {
    235,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo oh_emlrtRSI = {
    238,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ph_emlrtRSI = {
    239,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo qh_emlrtRSI = {
    243,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo rh_emlrtRSI = {
    250,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo sh_emlrtRSI = {
    251,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo th_emlrtRSI = {
    258,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo uh_emlrtRSI = {
    266,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo vh_emlrtRSI = {
    271,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo wh_emlrtRSI = {
    286,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo xh_emlrtRSI = {
    290,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo yh_emlrtRSI = {
    291,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ai_emlrtRSI = {
    294,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo bi_emlrtRSI = {
    296,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ci_emlrtRSI = {
    320,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo di_emlrtRSI = {
    322,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ei_emlrtRSI = {
    324,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo fi_emlrtRSI = {
    327,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo gi_emlrtRSI = {
    334,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo hi_emlrtRSI = {
    337,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ii_emlrtRSI = {
    348,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ji_emlrtRSI = {
    355,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ki_emlrtRSI = {
    364,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo li_emlrtRSI = {
    365,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo mi_emlrtRSI = {
    369,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ni_emlrtRSI = {
    370,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo oi_emlrtRSI = {
    371,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo pi_emlrtRSI = {
    372,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo qi_emlrtRSI = {
    385,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ri_emlrtRSI = {
    388,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo si_emlrtRSI = {
    394,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ti_emlrtRSI = {
    398,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo ui_emlrtRSI = {
    410,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo bo_emlrtRSI = {
    415,                                            /* lineNo */
    "DampedBFGSwGradientProjection/atLocalMinimum", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo no_emlrtRSI =
    {
        34,               /* lineNo */
        "rdivide_helper", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
        "rdivide_helper.m" /* pathName */
};

static emlrtRSInfo oo_emlrtRSI = {
    51,    /* lineNo */
    "div", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/div.m" /* pathName
                                                                       */
};

static emlrtRSInfo po_emlrtRSI = {
    17,    /* lineNo */
    "max", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/datafun/max.m" /* pathName
                                                                     */
};

static emlrtRSInfo qo_emlrtRSI = {
    38,         /* lineNo */
    "minOrMax", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName
                                                                            */
};

static emlrtRSInfo ro_emlrtRSI = {
    77,        /* lineNo */
    "maximum", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName
                                                                            */
};

static emlrtRSInfo so_emlrtRSI =
    {
        173,             /* lineNo */
        "unaryMinOrMax", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
        "unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo to_emlrtRSI = {
    72,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo uo_emlrtRSI = {
    64,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo vo_emlrtRSI = {
    113,         /* lineNo */
    "findFirst", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo wo_emlrtRSI = {
    130,                        /* lineNo */
    "minOrMaxRealVectorKernel", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo yo_emlrtRSI = {
    17,    /* lineNo */
    "min", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/datafun/min.m" /* pathName
                                                                     */
};

static emlrtRSInfo ap_emlrtRSI = {
    40,         /* lineNo */
    "minOrMax", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName
                                                                            */
};

static emlrtRSInfo bp_emlrtRSI = {
    90,        /* lineNo */
    "minimum", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName
                                                                            */
};

static emlrtRSInfo kp_emlrtRSI = {
    419,                                                    /* lineNo */
    "DampedBFGSwGradientProjection/searchDirectionInvalid", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRTEInfo eb_emlrtRTEI =
    {
        134,             /* lineNo */
        27,              /* colNo */
        "unaryMinOrMax", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
        "unaryMinOrMax.m" /* pName */
};

static emlrtECInfo w_emlrtECI = {
    2,                                             /* nDims */
    385,                                           /* lineNo */
    67,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo x_emlrtECI = {
    1,                                             /* nDims */
    385,                                           /* lineNo */
    67,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo y_emlrtECI = {
    2,                                             /* nDims */
    372,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo ab_emlrtECI = {
    1,                                             /* nDims */
    372,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo bb_emlrtECI = {
    2,                                             /* nDims */
    371,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo cb_emlrtECI = {
    1,                                             /* nDims */
    371,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo db_emlrtECI = {
    1,                                             /* nDims */
    369,                                           /* lineNo */
    28,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo eb_emlrtECI = {
    1,                                             /* nDims */
    358,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo fb_emlrtECI = {
    2,                                             /* nDims */
    355,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo gb_emlrtECI = {
    1,                                             /* nDims */
    355,                                           /* lineNo */
    25,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtBCInfo tc_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    349,                                           /* lineNo */
    48,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtECInfo hb_emlrtECI = {
    1,                                             /* nDims */
    336,                                           /* lineNo */
    24,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo ib_emlrtECI = {
    1,                                             /* nDims */
    334,                                           /* lineNo */
    64,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo jb_emlrtECI = {
    1,                                             /* nDims */
    320,                                           /* lineNo */
    60,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo kb_emlrtECI = {
    1,                                             /* nDims */
    291,                                           /* lineNo */
    32,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo lb_emlrtECI = {
    2,                                             /* nDims */
    271,                                           /* lineNo */
    30,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo mb_emlrtECI = {
    1,                                             /* nDims */
    271,                                           /* lineNo */
    30,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtBCInfo uc_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    270,                                           /* lineNo */
    55,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtECInfo nb_emlrtECI = {
    2,                                             /* nDims */
    266,                                           /* lineNo */
    30,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo ob_emlrtECI = {
    1,                                             /* nDims */
    266,                                           /* lineNo */
    30,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtBCInfo vc_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    259,                                           /* lineNo */
    57,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtRTEInfo gb_emlrtRTEI = {
    220,                                           /* lineNo */
    21,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo pb_emlrtECI = {
    2,                                             /* nDims */
    213,                                           /* lineNo */
    21,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtECInfo qb_emlrtECI = {
    1,                                             /* nDims */
    213,                                           /* lineNo */
    21,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtBCInfo wc_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    211,                                           /* lineNo */
    25,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtECInfo rb_emlrtECI = {
    1,                                             /* nDims */
    200,                                           /* lineNo */
    29,                                            /* colNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtBCInfo xc_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    201,                                           /* lineNo */
    45,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo yc_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    288,                                           /* lineNo */
    47,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo ad_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    289,                                           /* lineNo */
    50,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo bd_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    351,                                           /* lineNo */
    31,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo cd_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    262,                                           /* lineNo */
    36,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo dd_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    352,                                           /* lineNo */
    49,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo ed_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    263,                                           /* lineNo */
    54,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo fd_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    296,                                           /* lineNo */
    51,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo gd_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    297,                                           /* lineNo */
    58,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo hd_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    297,                                           /* lineNo */
    60,                                            /* colNo */
    "",                                            /* aName */
    "DampedBFGSwGradientProjection/solveInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtECInfo ub_emlrtECI = {
    1,                                                      /* nDims */
    419,                                                    /* lineNo */
    45,                                                     /* colNo */
    "DampedBFGSwGradientProjection/searchDirectionInvalid", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ig_emlrtRTEI = {
    182,                             /* lineNo */
    13,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo jg_emlrtRTEI = {
    192,                             /* lineNo */
    13,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo kg_emlrtRTEI = {
    203,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo lg_emlrtRTEI = {
    200,                             /* lineNo */
    29,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo mg_emlrtRTEI = {
    216,                             /* lineNo */
    13,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ng_emlrtRTEI = {
    200,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo og_emlrtRTEI = {
    211,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo pg_emlrtRTEI = {
    180,                             /* lineNo */
    48,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo qg_emlrtRTEI = {
    201,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo rg_emlrtRTEI =
    {
        76,                  /* lineNo */
        9,                   /* colNo */
        "eml_mtimes_helper", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pName */
};

static emlrtRTEInfo sg_emlrtRTEI = {
    233,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo tg_emlrtRTEI =
    {
        86,                  /* lineNo */
        9,                   /* colNo */
        "eml_mtimes_helper", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pName */
};

static emlrtRTEInfo ug_emlrtRTEI = {
    235,                             /* lineNo */
    36,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo vg_emlrtRTEI = {
    286,                             /* lineNo */
    32,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo wg_emlrtRTEI = {
    320,                             /* lineNo */
    62,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo xg_emlrtRTEI = {
    320,                             /* lineNo */
    60,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo yg_emlrtRTEI = {
    288,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ah_emlrtRTEI =
    {
        74,                  /* lineNo */
        9,                   /* colNo */
        "eml_mtimes_helper", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pName */
};

static emlrtRTEInfo bh_emlrtRTEI = {
    289,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ch_emlrtRTEI = {
    336,                             /* lineNo */
    28,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo dh_emlrtRTEI = {
    258,                             /* lineNo */
    26,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo eh_emlrtRTEI = {
    290,                             /* lineNo */
    54,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo fh_emlrtRTEI = {
    334,                             /* lineNo */
    68,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo gh_emlrtRTEI = {
    336,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo hh_emlrtRTEI = {
    290,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ih_emlrtRTEI = {
    337,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo jh_emlrtRTEI = {
    334,                             /* lineNo */
    64,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo kh_emlrtRTEI = {
    358,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo lh_emlrtRTEI = {
    263,                             /* lineNo */
    26,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo mh_emlrtRTEI = {
    349,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo nh_emlrtRTEI = {
    266,                             /* lineNo */
    49,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo oh_emlrtRTEI = {
    294,                             /* lineNo */
    30,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ph_emlrtRTEI = {
    352,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo qh_emlrtRTEI = {
    294,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo rh_emlrtRTEI = {
    270,                             /* lineNo */
    26,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo sh_emlrtRTEI = {
    371,                             /* lineNo */
    33,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo th_emlrtRTEI = {
    403,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo uh_emlrtRTEI = {
    396,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo vh_emlrtRTEI = {
    404,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo wh_emlrtRTEI = {
    372,                             /* lineNo */
    34,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo xh_emlrtRTEI = {
    385,                             /* lineNo */
    67,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo yh_emlrtRTEI = {
    196,                             /* lineNo */
    13,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ai_emlrtRTEI = {
    238,                             /* lineNo */
    17,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo bi_emlrtRTEI = {
    250,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ci_emlrtRTEI = {
    266,                             /* lineNo */
    26,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo di_emlrtRTEI = {
    288,                             /* lineNo */
    47,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ei_emlrtRTEI = {
    289,                             /* lineNo */
    50,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo fi_emlrtRTEI = {
    263,                             /* lineNo */
    54,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo gi_emlrtRTEI = {
    296,                             /* lineNo */
    51,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo hi_emlrtRTEI = {
    352,                             /* lineNo */
    49,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ii_emlrtRTEI = {
    212,                             /* lineNo */
    23,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ji_emlrtRTEI = {
    213,                             /* lineNo */
    25,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ki_emlrtRTEI = {
    364,                             /* lineNo */
    31,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo tj_emlrtRTEI = {
    415,                             /* lineNo */
    58,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo ik_emlrtRTEI = {
    419,                             /* lineNo */
    45,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo jk_emlrtRTEI = {
    418,                             /* lineNo */
    25,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo pk_emlrtRTEI = {
    213,                             /* lineNo */
    21,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo qk_emlrtRTEI = {
    271,                             /* lineNo */
    30,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRTEInfo rk_emlrtRTEI = {
    336,                             /* lineNo */
    24,                              /* colNo */
    "DampedBFGSwGradientProjection", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pName */
};

static emlrtRSInfo oq_emlrtRSI = {
    358,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo pq_emlrtRSI = {
    336,                                           /* lineNo */
    "DampedBFGSwGradientProjection/solveInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo qq_emlrtRSI = {
    52,    /* lineNo */
    "div", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/div.m" /* pathName
                                                                       */
};

/* Function Declarations */
static void b_plus(const emlrtStack *sp, emxArray_real_T *in1,
                   const emxArray_real_T *in2);

static void c_binary_expand_op(const emlrtStack *sp, const emlrtRSInfo in1,
                               const emxArray_real_T *in2,
                               const emxArray_real_T *in3,
                               c_robotics_core_internal_Damped *in4,
                               real_T in5[36], emxArray_real_T *in6,
                               real_T *out1,
                               c_robotics_manip_internal_IKExt **out2);

static void c_minus(const emlrtStack *sp, emxArray_real_T *in1,
                    const emxArray_real_T *in2);

static void c_plus(const emlrtStack *sp, emxArray_real_T *in1,
                   const emxArray_real_T *in2);

static boolean_T d_DampedBFGSwGradientProjection(
    const emlrtStack *sp, const c_robotics_core_internal_Damped *obj,
    const emxArray_real_T *Hg, const emxArray_real_T *alpha);

static void d_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2,
                               const c_robotics_core_internal_Damped *in3);

static boolean_T
e_DampedBFGSwGradientProjection(const emlrtStack *sp,
                                const c_robotics_core_internal_Damped *obj,
                                const emxArray_real_T *xNew);

/* Function Definitions */
static void b_plus(const emlrtStack *sp, emxArray_real_T *in1,
                   const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &qk_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  if (in2->size[0] == 1) {
    b_in1->size[0] = in1->size[0];
  } else {
    b_in1->size[0] = in2->size[0];
  }
  if (in2->size[1] == 1) {
    b_in1->size[1] = in1->size[1];
  } else {
    b_in1->size[1] = in2->size[1];
  }
  emxEnsureCapacity_real_T(sp, b_in1, i, &qk_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_0_1 = (in1->size[1] != 1);
  stride_1_0 = (in2->size[0] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }
  for (i = 0; i < loop_ub; i++) {
    i1 = in2->size[0];
    if (i1 == 1) {
      b_loop_ub = in1->size[0];
    } else {
      b_loop_ub = i1;
    }
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[0] * i] =
          in1_data[i1 * stride_0_0 + in1->size[0] * aux_0_1] +
          in2_data[i1 * stride_1_0 + in2->size[0] * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = b_in1->size[0];
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &qk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
    }
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_binary_expand_op(const emlrtStack *sp, const emlrtRSInfo in1,
                               const emxArray_real_T *in2,
                               const emxArray_real_T *in3,
                               c_robotics_core_internal_Damped *in4,
                               real_T in5[36], emxArray_real_T *in6,
                               real_T *out1,
                               c_robotics_manip_internal_IKExt **out2)
{
  emlrtStack st;
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  const real_T *in3_data;
  real_T *b_in2_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  st.prev = sp;
  st.tls = sp->tls;
  in3_data = in3->data;
  in2_data = in2->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in2, 1, &jh_emlrtRTEI);
  i = b_in2->size[0];
  if (in3->size[0] == 1) {
    b_in2->size[0] = in2->size[0];
  } else {
    b_in2->size[0] = in3->size[0];
  }
  emxEnsureCapacity_real_T(sp, b_in2, i, &jh_emlrtRTEI);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] = in2_data[i * stride_0_0] + in3_data[i * stride_1_0];
  }
  st.site = (emlrtRSInfo *)&in1;
  IKHelpers_computeCost(&st, b_in2, in4->ExtraArgs, out1, in5, in6, out2);
  emxFree_real_T(sp, &b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_minus(const emlrtStack *sp, emxArray_real_T *in1,
                    const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &pk_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  if (in2->size[0] == 1) {
    b_in1->size[0] = in1->size[0];
  } else {
    b_in1->size[0] = in2->size[0];
  }
  if (in2->size[1] == 1) {
    b_in1->size[1] = in1->size[1];
  } else {
    b_in1->size[1] = in2->size[1];
  }
  emxEnsureCapacity_real_T(sp, b_in1, i, &pk_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_0_1 = (in1->size[1] != 1);
  stride_1_0 = (in2->size[0] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }
  for (i = 0; i < loop_ub; i++) {
    i1 = in2->size[0];
    if (i1 == 1) {
      b_loop_ub = in1->size[0];
    } else {
      b_loop_ub = i1;
    }
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[0] * i] =
          in1_data[i1 * stride_0_0 + in1->size[0] * aux_0_1] -
          in2_data[i1 * stride_1_0 + in2->size[0] * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = b_in1->size[0];
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &pk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
    }
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_plus(const emlrtStack *sp, emxArray_real_T *in1,
                   const emxArray_real_T *in2)
{
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  real_T *b_in2_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in2, 1, &rk_emlrtRTEI);
  i = b_in2->size[0];
  if (in1->size[0] == 1) {
    b_in2->size[0] = in2->size[0];
  } else {
    b_in2->size[0] = in1->size[0];
  }
  emxEnsureCapacity_real_T(sp, b_in2, i, &rk_emlrtRTEI);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  if (in1->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in1->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] = in2_data[i * stride_0_0] + in1_data[i * stride_1_0];
  }
  i = in1->size[0];
  in1->size[0] = b_in2->size[0];
  emxEnsureCapacity_real_T(sp, in1, i, &rk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in2->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in2_data[i];
  }
  emxFree_real_T(sp, &b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static boolean_T d_DampedBFGSwGradientProjection(
    const emlrtStack *sp, const c_robotics_core_internal_Damped *obj,
    const emxArray_real_T *Hg, const emxArray_real_T *alpha)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  emlrtStack st;
  emxArray_boolean_T *b_alpha;
  const real_T *Hg_data;
  const real_T *alpha_data;
  real_T y;
  int32_T i;
  boolean_T flag;
  boolean_T *b_alpha_data;
  st.prev = sp;
  st.tls = sp->tls;
  alpha_data = alpha->data;
  Hg_data = Hg->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (Hg->size[0] == 0) {
    y = 0.0;
  } else {
    n_t = (ptrdiff_t)Hg->size[0];
    incx_t = (ptrdiff_t)1;
    y = dnrm2(&n_t, (real_T *)&Hg_data[0], &incx_t);
  }
  emxInit_boolean_T(sp, &b_alpha, &tj_emlrtRTEI);
  if (y < obj->GradientTolerance) {
    int32_T loop_ub;
    i = b_alpha->size[0];
    b_alpha->size[0] = alpha->size[0];
    emxEnsureCapacity_boolean_T(sp, b_alpha, i, &tj_emlrtRTEI);
    b_alpha_data = b_alpha->data;
    loop_ub = alpha->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_alpha_data[i] = (alpha_data[i] <= 0.0);
    }
    st.site = &bo_emlrtRSI;
    if (all(&st, b_alpha)) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }
  emxFree_boolean_T(sp, &b_alpha);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return flag;
}

static void d_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2,
                               const c_robotics_core_internal_Damped *in3)
{
  const real_T *in2_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  boolean_T *in1_data;
  in2_data = in2->data;
  i = in1->size[0];
  if (in3->ConstraintBound->size[0] == 1) {
    in1->size[0] = in2->size[0];
  } else {
    in1->size[0] = in3->ConstraintBound->size[0];
  }
  emxEnsureCapacity_boolean_T(sp, in1, i, &ng_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->ConstraintBound->size[0] != 1);
  if (in3->ConstraintBound->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->ConstraintBound->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] >=
                   in3->ConstraintBound->data[i * stride_1_0]);
  }
}

static boolean_T
e_DampedBFGSwGradientProjection(const emlrtStack *sp,
                                const c_robotics_core_internal_Damped *obj,
                                const emxArray_real_T *xNew)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_boolean_T *r1;
  emxArray_real_T *r;
  real_T *r2;
  int32_T i;
  boolean_T flag;
  boolean_T *r3;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &r, 1, &jk_emlrtRTEI);
  emxInit_boolean_T(sp, &r1, &ik_emlrtRTEI);
  if (obj->ConstraintsOn) {
    int32_T loop_ub;
    int32_T stride_0_0;
    int32_T stride_1_0;
    st.site = &kp_emlrtRSI;
    b_st.site = &ek_emlrtRSI;
    if (obj->ConstraintMatrix->size[0] != xNew->size[0]) {
      if (((obj->ConstraintMatrix->size[0] == 1) &&
           (obj->ConstraintMatrix->size[1] == 1)) ||
          (xNew->size[0] == 1)) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &kb_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &jb_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    b_st.site = &fk_emlrtRSI;
    b_mtimes(&b_st, obj->ConstraintMatrix, xNew, r);
    r2 = r->data;
    i = obj->ConstraintBound->size[0];
    if ((r->size[0] != i) && ((r->size[0] != 1) && (i != 1))) {
      emlrtDimSizeImpxCheckR2021b(r->size[0], i, &ub_emlrtECI,
                                  (emlrtConstCTX)sp);
    }
    st.site = &kp_emlrtRSI;
    i = r1->size[0];
    if (obj->ConstraintBound->size[0] == 1) {
      r1->size[0] = r->size[0];
    } else {
      r1->size[0] = obj->ConstraintBound->size[0];
    }
    emxEnsureCapacity_boolean_T(sp, r1, i, &ik_emlrtRTEI);
    r3 = r1->data;
    stride_0_0 = (r->size[0] != 1);
    stride_1_0 = (obj->ConstraintBound->size[0] != 1);
    if (obj->ConstraintBound->size[0] == 1) {
      loop_ub = r->size[0];
    } else {
      loop_ub = obj->ConstraintBound->size[0];
    }
    for (i = 0; i < loop_ub; i++) {
      r3[i] = (r2[i * stride_0_0] - obj->ConstraintBound->data[i * stride_1_0] >
               1.4901161193847656E-8);
    }
    st.site = &kp_emlrtRSI;
    if (any(&st, r1)) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }
  emxFree_boolean_T(sp, &r1);
  emxFree_real_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return flag;
}

void c_DampedBFGSwGradientProjection(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *obj,
                                     emxArray_real_T *xSol,
                                     c_robotics_core_internal_NLPSol *exitFlag,
                                     real_T *err, real_T *iter)
{
  c_robotics_manip_internal_IKExt *args;
  c_robotics_manip_internal_IKExt *r;
  c_robotics_manip_internal_IKExt *r6;
  c_robotics_manip_internal_IKExt *r7;
  c_robotics_manip_internal_IKExt *r8;
  c_robotics_manip_internal_IKExt *r9;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  emxArray_boolean_T *activeSet;
  emxArray_boolean_T *b_activeSet;
  emxArray_int32_T *ii;
  emxArray_int32_T *inactiveConstraintIndices;
  emxArray_int32_T *r1;
  emxArray_int32_T *r2;
  emxArray_int32_T *r3;
  emxArray_int32_T *r4;
  emxArray_int32_T *r5;
  emxArray_real_T *A;
  emxArray_real_T *B;
  emxArray_real_T *H;
  emxArray_real_T *Hg;
  emxArray_real_T *P;
  emxArray_real_T *a;
  emxArray_real_T *a__2;
  emxArray_real_T *bIn;
  emxArray_real_T *b_y;
  emxArray_real_T *c_y;
  emxArray_real_T *d_y;
  emxArray_real_T *grad;
  emxArray_real_T *gradNew;
  emxArray_real_T *x;
  emxArray_real_T *y;
  real_T a__1[36];
  real_T b_gamma;
  real_T cost;
  real_T costNew;
  real_T d;
  real_T *A_data;
  real_T *B_data;
  real_T *H_data;
  real_T *Hg_data;
  real_T *a_data;
  real_T *gradNew_data;
  real_T *grad_data;
  real_T *xSol_data;
  real_T *x_data;
  real_T *y_data;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T end;
  int32_T i;
  int32_T i1;
  int32_T last;
  int32_T loop_ub;
  int32_T n;
  int32_T *ii_data;
  int32_T *inactiveConstraintIndices_data;
  boolean_T *activeSet_data;
  boolean_T *b_activeSet_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &x, 1, &ig_emlrtRTEI);
  i = x->size[0];
  x->size[0] = obj->SeedInternal->size[0];
  emxEnsureCapacity_real_T(sp, x, i, &ig_emlrtRTEI);
  x_data = x->data;
  loop_ub = obj->SeedInternal->size[0];
  for (i = 0; i < loop_ub; i++) {
    x_data[i] = obj->SeedInternal->data[i];
  }
  st.site = &eh_emlrtRSI;
  b_st.site = &xg_emlrtRSI;
  obj->TimeObjInternal.StartTime = tic(&b_st);
  b_st.site = &yg_emlrtRSI;
  n = x->size[0];
  emxInit_real_T(sp, &a__2, 2, &pg_emlrtRTEI);
  st.site = &fh_emlrtRSI;
  IKHelpers_computeCost(&st, x, obj->ExtraArgs, &cost, a__1, a__2, &r);
  obj->ExtraArgs = r;
  st.site = &gh_emlrtRSI;
  args = obj->ExtraArgs;
  emxInit_real_T(&st, &grad, 1, &jg_emlrtRTEI);
  i = grad->size[0];
  grad->size[0] = args->GradTemp->size[0];
  emxEnsureCapacity_real_T(&st, grad, i, &jg_emlrtRTEI);
  grad_data = grad->data;
  loop_ub = args->GradTemp->size[0];
  for (i = 0; i < loop_ub; i++) {
    grad_data[i] = args->GradTemp->data[i];
  }
  emxInit_real_T(sp, &H, 2, &yh_emlrtRTEI);
  st.site = &hh_emlrtRSI;
  eye(&st, x->size[0], H);
  H_data = H->data;
  emxInit_boolean_T(sp, &activeSet, &ng_emlrtRTEI);
  emxInit_real_T(sp, &A, 2, &qg_emlrtRTEI);
  A_data = A->data;
  emxInit_real_T(sp, &a, 1, &og_emlrtRTEI);
  emxInit_real_T(sp, &y, 2, &ji_emlrtRTEI);
  emxInit_int32_T(sp, &ii, 1, &rf_emlrtRTEI);
  if (obj->ConstraintsOn) {
    i = y->size[0] * y->size[1];
    y->size[0] = obj->ConstraintMatrix->size[0];
    y->size[1] = obj->ConstraintMatrix->size[1];
    emxEnsureCapacity_real_T(sp, y, i, &lg_emlrtRTEI);
    y_data = y->data;
    loop_ub = obj->ConstraintMatrix->size[0] * obj->ConstraintMatrix->size[1];
    for (i = 0; i < loop_ub; i++) {
      y_data[i] = obj->ConstraintMatrix->data[i];
    }
    st.site = &ih_emlrtRSI;
    b_st.site = &ek_emlrtRSI;
    dynamic_size_checks(&b_st, y, x, y->size[0], x->size[0]);
    b_st.site = &fk_emlrtRSI;
    b_mtimes(&b_st, y, x, a);
    a_data = a->data;
    i = obj->ConstraintBound->size[0];
    if ((a->size[0] != i) && ((a->size[0] != 1) && (i != 1))) {
      emlrtDimSizeImpxCheckR2021b(a->size[0], i, &rb_emlrtECI,
                                  (emlrtConstCTX)sp);
    }
    if (a->size[0] == obj->ConstraintBound->size[0]) {
      i = activeSet->size[0];
      activeSet->size[0] = a->size[0];
      emxEnsureCapacity_boolean_T(sp, activeSet, i, &ng_emlrtRTEI);
      activeSet_data = activeSet->data;
      loop_ub = a->size[0];
      for (i = 0; i < loop_ub; i++) {
        activeSet_data[i] = (a_data[i] >= obj->ConstraintBound->data[i]);
      }
    } else {
      st.site = &ih_emlrtRSI;
      d_binary_expand_op(&st, activeSet, a, obj);
      activeSet_data = activeSet->data;
    }
    end = activeSet->size[0] - 1;
    last = 0;
    for (b_i = 0; b_i <= end; b_i++) {
      if (activeSet_data[b_i]) {
        last++;
      }
    }
    i = ii->size[0];
    ii->size[0] = last;
    emxEnsureCapacity_int32_T(sp, ii, i, &pg_emlrtRTEI);
    ii_data = ii->data;
    last = 0;
    for (b_i = 0; b_i <= end; b_i++) {
      if (activeSet_data[b_i]) {
        ii_data[last] = b_i + 1;
        last++;
      }
    }
    last = obj->ConstraintMatrix->size[1];
    i = A->size[0] * A->size[1];
    A->size[0] = obj->ConstraintMatrix->size[0];
    A->size[1] = ii->size[0];
    emxEnsureCapacity_real_T(sp, A, i, &qg_emlrtRTEI);
    A_data = A->data;
    loop_ub = ii->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = obj->ConstraintMatrix->size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        if (ii_data[i] > last) {
          emlrtDynamicBoundsCheckR2012b(ii_data[i], 1, last, &xc_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        A_data[i1 + A->size[0] * i] =
            obj->ConstraintMatrix
                ->data[i1 + obj->ConstraintMatrix->size[0] * (ii_data[i] - 1)];
      }
    }
  } else {
    uint32_T unnamed_idx_0;
    unnamed_idx_0 = (uint32_T)obj->ConstraintBound->size[0];
    i = activeSet->size[0];
    activeSet->size[0] = (int32_T)unnamed_idx_0;
    emxEnsureCapacity_boolean_T(sp, activeSet, i, &kg_emlrtRTEI);
    activeSet_data = activeSet->data;
    loop_ub = (int32_T)unnamed_idx_0;
    for (i = 0; i < loop_ub; i++) {
      activeSet_data[i] = false;
    }
    A->size[0] = x->size[0];
    A->size[1] = 0;
  }
  i = A->size[1];
  emxInit_real_T(sp, &B, 2, &bi_emlrtRTEI);
  emxInit_real_T(sp, &b_y, 2, &ii_emlrtRTEI);
  emxInit_real_T(sp, &c_y, 2, &ji_emlrtRTEI);
  for (end = 0; end < i; end++) {
    if (end + 1 > A->size[1]) {
      emlrtDynamicBoundsCheckR2012b(end + 1, 1, A->size[1], &wc_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = A->size[0];
    i1 = a->size[0];
    a->size[0] = A->size[0];
    emxEnsureCapacity_real_T(sp, a, i1, &og_emlrtRTEI);
    a_data = a->data;
    for (i1 = 0; i1 < loop_ub; i1++) {
      a_data[i1] = A_data[i1 + A->size[0] * end];
    }
    st.site = &jh_emlrtRSI;
    b_st.site = &ek_emlrtRSI;
    b_dynamic_size_checks(&b_st, a, H, A->size[0], H->size[0]);
    b_st.site = &fk_emlrtRSI;
    c_mtimes(&b_st, a, H, b_y);
    st.site = &jh_emlrtRSI;
    b_st.site = &ek_emlrtRSI;
    c_dynamic_size_checks(&b_st, b_y, a, b_y->size[1], A->size[0]);
    b_gamma = 1.0 / d_mtimes(b_y, a);
    i1 = y->size[0] * y->size[1];
    y->size[0] = H->size[0];
    y->size[1] = H->size[1];
    emxEnsureCapacity_real_T(sp, y, i1, &rg_emlrtRTEI);
    y_data = y->data;
    loop_ub = H->size[0] * H->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      y_data[i1] = b_gamma * H_data[i1];
    }
    loop_ub = A->size[0];
    i1 = B->size[0] * B->size[1];
    B->size[0] = A->size[0];
    B->size[1] = A->size[0];
    emxEnsureCapacity_real_T(sp, B, i1, &tg_emlrtRTEI);
    B_data = B->data;
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_loop_ub = A->size[0];
      for (last = 0; last < b_loop_ub; last++) {
        B_data[last + B->size[0] * i1] =
            A_data[last + A->size[0] * end] * A_data[i1 + A->size[0] * end];
      }
    }
    st.site = &kh_emlrtRSI;
    b_st.site = &ek_emlrtRSI;
    d_dynamic_size_checks(&b_st, y, B, y->size[1], B->size[0]);
    b_st.site = &fk_emlrtRSI;
    e_mtimes(&b_st, y, B, c_y);
    st.site = &kh_emlrtRSI;
    b_st.site = &ek_emlrtRSI;
    d_dynamic_size_checks(&b_st, c_y, H, c_y->size[1], H->size[0]);
    b_st.site = &fk_emlrtRSI;
    e_mtimes(&b_st, c_y, H, y);
    y_data = y->data;
    if ((H->size[0] != y->size[0]) &&
        ((H->size[0] != 1) && (y->size[0] != 1))) {
      emlrtDimSizeImpxCheckR2021b(H->size[0], y->size[0], &qb_emlrtECI,
                                  (emlrtConstCTX)sp);
    }
    if ((H->size[1] != y->size[1]) &&
        ((H->size[1] != 1) && (y->size[1] != 1))) {
      emlrtDimSizeImpxCheckR2021b(H->size[1], y->size[1], &pb_emlrtECI,
                                  (emlrtConstCTX)sp);
    }
    if ((H->size[0] == y->size[0]) && (H->size[1] == y->size[1])) {
      loop_ub = H->size[0] * H->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        H_data[i1] -= y_data[i1];
      }
    } else {
      st.site = &kh_emlrtRSI;
      c_minus(&st, H, y);
      H_data = H->data;
    }
  }
  i = xSol->size[0];
  xSol->size[0] = x->size[0];
  emxEnsureCapacity_real_T(sp, xSol, i, &mg_emlrtRTEI);
  xSol_data = xSol->data;
  loop_ub = x->size[0];
  for (i = 0; i < loop_ub; i++) {
    xSol_data[i] = x_data[i];
  }
  d = obj->MaxNumIterationInternal;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                &gb_emlrtRTEI, (emlrtConstCTX)sp);
  b_i = 0;
  emxInit_real_T(sp, &Hg, 1, &ai_emlrtRTEI);
  emxInit_real_T(sp, &P, 2, &ci_emlrtRTEI);
  emxInit_real_T(sp, &bIn, 1, &yg_emlrtRTEI);
  emxInit_int32_T(sp, &inactiveConstraintIndices, 1, &hh_emlrtRTEI);
  emxInit_real_T(sp, &gradNew, 1, &ih_emlrtRTEI);
  emxInit_int32_T(sp, &r1, 1, &di_emlrtRTEI);
  emxInit_int32_T(sp, &r2, 1, &ei_emlrtRTEI);
  emxInit_int32_T(sp, &r3, 1, &fi_emlrtRTEI);
  emxInit_int32_T(sp, &r4, 1, &gi_emlrtRTEI);
  emxInit_int32_T(sp, &r5, 1, &hi_emlrtRTEI);
  emxInit_real_T(sp, &d_y, 2, &ki_emlrtRTEI);
  emxInit_boolean_T(sp, &b_activeSet, &vg_emlrtRTEI);
  int32_T exitg2;
  do {
    exitg2 = 0;
    if (b_i <= (int32_T)d - 1) {
      boolean_T valid;
      st.site = &lh_emlrtRSI;
      b_st.site = &am_emlrtRSI;
      valid = (obj->TimeObjInternal.StartTime.tv_sec > 0.0);
      if (!valid) {
        emlrtErrorWithMessageIdR2018a(&st, &bb_emlrtRTEI,
                                      "shared_robotics:robotutils:timeprovider:"
                                      "TimeProviderNotInitialized",
                                      "shared_robotics:robotutils:timeprovider:"
                                      "TimeProviderNotInitialized",
                                      0);
      }
      b_st.site = &bm_emlrtRSI;
      b_gamma = toc(&b_st, obj->TimeObjInternal.StartTime.tv_sec,
                    obj->TimeObjInternal.StartTime.tv_nsec);
      st.site = &lh_emlrtRSI;
      valid = (b_gamma > obj->MaxTimeInternal);
      if (valid) {
        *exitFlag = TimeLimitExceeded;
        st.site = &mh_emlrtRSI;
        *err = IKHelpers_evaluateSolution(&st, obj->ExtraArgs);
        *iter = (real_T)b_i + 1.0;
        exitg2 = 1;
      } else {
        if ((A->size[0] == 0) || (A->size[1] == 0)) {
          i = a->size[0];
          a->size[0] = 1;
          emxEnsureCapacity_real_T(sp, a, i, &sg_emlrtRTEI);
          a_data = a->data;
          a_data[0] = 0.0;
        } else {
          st.site = &nh_emlrtRSI;
          b_st.site = &ek_emlrtRSI;
          d_dynamic_size_checks(&b_st, A, A, A->size[0], A->size[0]);
          b_st.site = &fk_emlrtRSI;
          f_mtimes(&b_st, A, A, y);
          st.site = &nh_emlrtRSI;
          i = c_y->size[0] * c_y->size[1];
          c_y->size[0] = A->size[1];
          c_y->size[1] = A->size[0];
          emxEnsureCapacity_real_T(&st, c_y, i, &ug_emlrtRTEI);
          y_data = c_y->data;
          loop_ub = A->size[0];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = A->size[1];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              y_data[i1 + c_y->size[0] * i] = A_data[i + A->size[0] * i1];
            }
          }
          b_st.site = &nh_emlrtRSI;
          mldivide(&b_st, y, c_y, B);
          b_st.site = &ek_emlrtRSI;
          dynamic_size_checks(&b_st, B, grad, B->size[1], grad->size[0]);
          b_st.site = &fk_emlrtRSI;
          g_mtimes(&b_st, B, grad, a);
          a_data = a->data;
        }
        st.site = &oh_emlrtRSI;
        b_st.site = &ek_emlrtRSI;
        dynamic_size_checks(&b_st, H, grad, H->size[1], grad->size[0]);
        b_st.site = &fk_emlrtRSI;
        g_mtimes(&b_st, H, grad, Hg);
        Hg_data = Hg->data;
        st.site = &ph_emlrtRSI;
        if (d_DampedBFGSwGradientProjection(&st, obj, Hg, a)) {
          *exitFlag = LocalMinimumFound;
          st.site = &qh_emlrtRSI;
          *err = IKHelpers_evaluateSolution(&st, obj->ExtraArgs);
          *iter = (real_T)b_i + 1.0;
          exitg2 = 1;
        } else {
          real_T beta;
          int32_T b_a;
          int32_T idxl;
          boolean_T exitg3;
          boolean_T guard1 = false;
          boolean_T guard2 = false;
          boolean_T guard3 = false;
          guard1 = false;
          guard2 = false;
          guard3 = false;
          if (obj->ConstraintsOn && ((A->size[0] != 0) && (A->size[1] != 0))) {
            st.site = &rh_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            d_dynamic_size_checks(&b_st, A, A, A->size[0], A->size[0]);
            b_st.site = &fk_emlrtRSI;
            f_mtimes(&b_st, A, A, y);
            st.site = &rh_emlrtRSI;
            inv(&st, y, B);
            st.site = &sh_emlrtRSI;
            b_st.site = &sh_emlrtRSI;
            diag(&b_st, B, gradNew);
            b_st.site = &sh_emlrtRSI;
            c_sqrt(&b_st, gradNew);
            gradNew_data = gradNew->data;
            b_st.site = &no_emlrtRSI;
            c_st.site = &oo_emlrtRSI;
            assertCompatibleDims(&c_st, a, gradNew);
            if (a->size[0] == gradNew->size[0]) {
              loop_ub = a->size[0];
              for (i = 0; i < loop_ub; i++) {
                a_data[i] /= gradNew_data[i];
              }
            } else {
              c_st.site = &qq_emlrtRSI;
              rdivide(&c_st, a, gradNew);
              a_data = a->data;
            }
            st.site = &sh_emlrtRSI;
            b_st.site = &po_emlrtRSI;
            c_st.site = &qo_emlrtRSI;
            d_st.site = &ro_emlrtRSI;
            if (a->size[0] < 1) {
              emlrtErrorWithMessageIdR2018a(
                  &d_st, &eb_emlrtRTEI,
                  "Coder:toolbox:eml_min_or_max_varDimZero",
                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
            }
            e_st.site = &so_emlrtRSI;
            last = a->size[0];
            if (a->size[0] <= 2) {
              if (a->size[0] == 1) {
                b_gamma = a_data[0];
                idxl = 1;
              } else if ((a_data[0] < a_data[1]) ||
                         (muDoubleScalarIsNaN(a_data[0]) &&
                          (!muDoubleScalarIsNaN(a_data[1])))) {
                b_gamma = a_data[1];
                idxl = 2;
              } else {
                b_gamma = a_data[0];
                idxl = 1;
              }
            } else {
              f_st.site = &uo_emlrtRSI;
              if (!muDoubleScalarIsNaN(a_data[0])) {
                idxl = 1;
              } else {
                idxl = 0;
                g_st.site = &vo_emlrtRSI;
                if (a->size[0] > 2147483646) {
                  h_st.site = &vc_emlrtRSI;
                  check_forloop_overflow_error(&h_st);
                }
                end = 2;
                exitg3 = false;
                while ((!exitg3) && (end <= last)) {
                  if (!muDoubleScalarIsNaN(a_data[end - 1])) {
                    idxl = end;
                    exitg3 = true;
                  } else {
                    end++;
                  }
                }
              }
              if (idxl == 0) {
                b_gamma = a_data[0];
                idxl = 1;
              } else {
                f_st.site = &to_emlrtRSI;
                b_gamma = a_data[idxl - 1];
                b_a = idxl + 1;
                g_st.site = &wo_emlrtRSI;
                if ((idxl + 1 <= a->size[0]) && (a->size[0] > 2147483646)) {
                  h_st.site = &vc_emlrtRSI;
                  check_forloop_overflow_error(&h_st);
                }
                for (end = b_a; end <= last; end++) {
                  beta = a_data[end - 1];
                  if (b_gamma < beta) {
                    b_gamma = beta;
                    idxl = end;
                  }
                }
              }
            }
            if (c_norm(Hg) < 0.5 * b_gamma) {
              st.site = &th_emlrtRSI;
              b_st.site = &ng_emlrtRSI;
              eml_find(&b_st, activeSet, ii);
              ii_data = ii->data;
              i = a->size[0];
              a->size[0] = ii->size[0];
              emxEnsureCapacity_real_T(&st, a, i, &dh_emlrtRTEI);
              a_data = a->data;
              loop_ub = ii->size[0];
              for (i = 0; i < loop_ub; i++) {
                a_data[i] = ii_data[i];
              }
              if ((idxl < 1) || (idxl > a->size[0])) {
                emlrtDynamicBoundsCheckR2012b(idxl, 1, a->size[0], &vc_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              b_a = (int32_T)a_data[idxl - 1];
              if ((b_a < 1) || (b_a > activeSet->size[0])) {
                emlrtDynamicBoundsCheckR2012b((int32_T)a_data[idxl - 1], 1,
                                              activeSet->size[0], &cd_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              activeSet_data[b_a - 1] = false;
              end = activeSet->size[0] - 1;
              last = 0;
              for (c_i = 0; c_i <= end; c_i++) {
                if (activeSet_data[c_i]) {
                  last++;
                }
              }
              i = r3->size[0];
              r3->size[0] = last;
              emxEnsureCapacity_int32_T(sp, r3, i, &pg_emlrtRTEI);
              ii_data = r3->data;
              last = 0;
              for (c_i = 0; c_i <= end; c_i++) {
                if (activeSet_data[c_i]) {
                  ii_data[last] = c_i + 1;
                  last++;
                }
              }
              last = obj->ConstraintMatrix->size[1];
              i = A->size[0] * A->size[1];
              A->size[0] = obj->ConstraintMatrix->size[0];
              A->size[1] = r3->size[0];
              emxEnsureCapacity_real_T(sp, A, i, &lh_emlrtRTEI);
              A_data = A->data;
              loop_ub = r3->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = obj->ConstraintMatrix->size[0];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  if ((ii_data[i] < 1) || (ii_data[i] > last)) {
                    emlrtDynamicBoundsCheckR2012b(
                        ii_data[i], 1, last, &ed_emlrtBCI, (emlrtConstCTX)sp);
                  }
                  A_data[i1 + A->size[0] * i] =
                      obj->ConstraintMatrix
                          ->data[i1 + obj->ConstraintMatrix->size[0] *
                                          (ii_data[i] - 1)];
                }
              }
              st.site = &uh_emlrtRSI;
              eye(&st, n, P);
              xSol_data = P->data;
              st.site = &uh_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              d_dynamic_size_checks(&b_st, A, A, A->size[0], A->size[0]);
              b_st.site = &fk_emlrtRSI;
              f_mtimes(&b_st, A, A, y);
              st.site = &uh_emlrtRSI;
              i = c_y->size[0] * c_y->size[1];
              c_y->size[0] = A->size[1];
              c_y->size[1] = A->size[0];
              emxEnsureCapacity_real_T(&st, c_y, i, &nh_emlrtRTEI);
              y_data = c_y->data;
              loop_ub = A->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = A->size[1];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  y_data[i1 + c_y->size[0] * i] = A_data[i + A->size[0] * i1];
                }
              }
              b_st.site = &uh_emlrtRSI;
              mldivide(&b_st, y, c_y, B);
              b_st.site = &ek_emlrtRSI;
              d_dynamic_size_checks(&b_st, A, B, A->size[1], B->size[0]);
              b_st.site = &fk_emlrtRSI;
              e_mtimes(&b_st, A, B, y);
              y_data = y->data;
              if ((P->size[0] != y->size[0]) &&
                  ((P->size[0] != 1) && (y->size[0] != 1))) {
                emlrtDimSizeImpxCheckR2021b(P->size[0], y->size[0],
                                            &ob_emlrtECI, (emlrtConstCTX)sp);
              }
              if ((P->size[1] != y->size[1]) &&
                  ((P->size[1] != 1) && (y->size[1] != 1))) {
                emlrtDimSizeImpxCheckR2021b(P->size[1], y->size[1],
                                            &nb_emlrtECI, (emlrtConstCTX)sp);
              }
              if ((P->size[0] == y->size[0]) && (P->size[1] == y->size[1])) {
                loop_ub = P->size[0] * P->size[1];
                for (i = 0; i < loop_ub; i++) {
                  xSol_data[i] -= y_data[i];
                }
              } else {
                st.site = &uh_emlrtRSI;
                c_minus(&st, P, y);
                xSol_data = P->data;
              }
              i = obj->ConstraintMatrix->size[1];
              if (b_a > i) {
                emlrtDynamicBoundsCheckR2012b((int32_T)a_data[idxl - 1], 1, i,
                                              &uc_emlrtBCI, (emlrtConstCTX)sp);
              }
              i = a->size[0];
              a->size[0] = obj->ConstraintMatrix->size[0];
              emxEnsureCapacity_real_T(sp, a, i, &rh_emlrtRTEI);
              a_data = a->data;
              loop_ub = obj->ConstraintMatrix->size[0];
              for (i = 0; i < loop_ub; i++) {
                a_data[i] =
                    obj->ConstraintMatrix
                        ->data[i + obj->ConstraintMatrix->size[0] * (b_a - 1)];
              }
              st.site = &vh_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              b_dynamic_size_checks(&b_st, a, P, a->size[0], P->size[0]);
              b_st.site = &fk_emlrtRSI;
              c_mtimes(&b_st, a, P, b_y);
              st.site = &vh_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              c_dynamic_size_checks(&b_st, b_y, a, b_y->size[1], a->size[0]);
              b_gamma = 1.0 / d_mtimes(b_y, a);
              i = y->size[0] * y->size[1];
              y->size[0] = P->size[0];
              y->size[1] = P->size[1];
              emxEnsureCapacity_real_T(sp, y, i, &rg_emlrtRTEI);
              y_data = y->data;
              loop_ub = P->size[0] * P->size[1];
              for (i = 0; i < loop_ub; i++) {
                y_data[i] = b_gamma * xSol_data[i];
              }
              i = B->size[0] * B->size[1];
              B->size[0] = a->size[0];
              B->size[1] = a->size[0];
              emxEnsureCapacity_real_T(sp, B, i, &tg_emlrtRTEI);
              B_data = B->data;
              loop_ub = a->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = a->size[0];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  B_data[i1 + B->size[0] * i] = a_data[i1] * a_data[i];
                }
              }
              st.site = &vh_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              d_dynamic_size_checks(&b_st, y, B, y->size[1], B->size[0]);
              b_st.site = &fk_emlrtRSI;
              e_mtimes(&b_st, y, B, c_y);
              st.site = &vh_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              d_dynamic_size_checks(&b_st, c_y, P, c_y->size[1], P->size[0]);
              b_st.site = &fk_emlrtRSI;
              e_mtimes(&b_st, c_y, P, y);
              y_data = y->data;
              if ((H->size[0] != y->size[0]) &&
                  ((H->size[0] != 1) && (y->size[0] != 1))) {
                emlrtDimSizeImpxCheckR2021b(H->size[0], y->size[0],
                                            &mb_emlrtECI, (emlrtConstCTX)sp);
              }
              if ((H->size[1] != y->size[1]) &&
                  ((H->size[1] != 1) && (y->size[1] != 1))) {
                emlrtDimSizeImpxCheckR2021b(H->size[1], y->size[1],
                                            &lb_emlrtECI, (emlrtConstCTX)sp);
              }
              if ((H->size[0] == y->size[0]) && (H->size[1] == y->size[1])) {
                loop_ub = H->size[0] * H->size[1];
                for (i = 0; i < loop_ub; i++) {
                  H_data[i] += y_data[i];
                }
              } else {
                st.site = &vh_emlrtRSI;
                b_plus(&st, H, y);
                H_data = H->data;
              }
              b_i++;
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }
          if (guard3) {
            real_T lambda;
            real_T m;
            real_T sigma;
            int32_T exitg1;
            loop_ub = Hg->size[0];
            for (i = 0; i < loop_ub; i++) {
              Hg_data[i] = -Hg_data[i];
            }
            idxl = -1;
            if (obj->ConstraintsOn) {
              i = b_activeSet->size[0];
              b_activeSet->size[0] = activeSet->size[0];
              emxEnsureCapacity_boolean_T(sp, b_activeSet, i, &vg_emlrtRTEI);
              b_activeSet_data = b_activeSet->data;
              loop_ub = activeSet->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_activeSet_data[i] = !activeSet_data[i];
              }
              st.site = &wh_emlrtRSI;
              if (any(&st, b_activeSet)) {
                end = activeSet->size[0] - 1;
                last = 0;
                for (c_i = 0; c_i <= end; c_i++) {
                  if (!activeSet_data[c_i]) {
                    last++;
                  }
                }
                i = r1->size[0];
                r1->size[0] = last;
                emxEnsureCapacity_int32_T(sp, r1, i, &pg_emlrtRTEI);
                ii_data = r1->data;
                last = 0;
                for (c_i = 0; c_i <= end; c_i++) {
                  if (!activeSet_data[c_i]) {
                    ii_data[last] = c_i + 1;
                    last++;
                  }
                }
                last = obj->ConstraintBound->size[0];
                i = bIn->size[0];
                bIn->size[0] = r1->size[0];
                emxEnsureCapacity_real_T(sp, bIn, i, &yg_emlrtRTEI);
                xSol_data = bIn->data;
                loop_ub = r1->size[0];
                for (i = 0; i < loop_ub; i++) {
                  if ((ii_data[i] < 1) || (ii_data[i] > last)) {
                    emlrtDynamicBoundsCheckR2012b(
                        ii_data[i], 1, last, &yc_emlrtBCI, (emlrtConstCTX)sp);
                  }
                  xSol_data[i] = obj->ConstraintBound->data[ii_data[i] - 1];
                }
                end = activeSet->size[0] - 1;
                last = 0;
                for (c_i = 0; c_i <= end; c_i++) {
                  if (!activeSet_data[c_i]) {
                    last++;
                  }
                }
                i = r2->size[0];
                r2->size[0] = last;
                emxEnsureCapacity_int32_T(sp, r2, i, &pg_emlrtRTEI);
                ii_data = r2->data;
                last = 0;
                for (c_i = 0; c_i <= end; c_i++) {
                  if (!activeSet_data[c_i]) {
                    ii_data[last] = c_i + 1;
                    last++;
                  }
                }
                last = obj->ConstraintMatrix->size[1];
                i = B->size[0] * B->size[1];
                B->size[0] = obj->ConstraintMatrix->size[0];
                B->size[1] = r2->size[0];
                emxEnsureCapacity_real_T(sp, B, i, &bh_emlrtRTEI);
                B_data = B->data;
                loop_ub = r2->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = obj->ConstraintMatrix->size[0];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    if ((ii_data[i] < 1) || (ii_data[i] > last)) {
                      emlrtDynamicBoundsCheckR2012b(
                          ii_data[i], 1, last, &ad_emlrtBCI, (emlrtConstCTX)sp);
                    }
                    B_data[i1 + B->size[0] * i] =
                        obj->ConstraintMatrix
                            ->data[i1 + obj->ConstraintMatrix->size[0] *
                                            (ii_data[i] - 1)];
                  }
                }
                st.site = &xh_emlrtRSI;
                i = b_activeSet->size[0];
                b_activeSet->size[0] = activeSet->size[0];
                emxEnsureCapacity_boolean_T(&st, b_activeSet, i, &eh_emlrtRTEI);
                b_activeSet_data = b_activeSet->data;
                loop_ub = activeSet->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_activeSet_data[i] = !activeSet_data[i];
                }
                b_st.site = &ng_emlrtRSI;
                eml_find(&b_st, b_activeSet, ii);
                ii_data = ii->data;
                i = inactiveConstraintIndices->size[0];
                inactiveConstraintIndices->size[0] = ii->size[0];
                emxEnsureCapacity_int32_T(&st, inactiveConstraintIndices, i,
                                          &hh_emlrtRTEI);
                inactiveConstraintIndices_data =
                    inactiveConstraintIndices->data;
                loop_ub = ii->size[0];
                for (i = 0; i < loop_ub; i++) {
                  inactiveConstraintIndices_data[i] = ii_data[i];
                }
                st.site = &yh_emlrtRSI;
                b_st.site = &ek_emlrtRSI;
                dynamic_size_checks(&b_st, B, x, B->size[0], x->size[0]);
                b_st.site = &fk_emlrtRSI;
                b_mtimes(&b_st, B, x, a);
                a_data = a->data;
                if ((bIn->size[0] != a->size[0]) &&
                    ((bIn->size[0] != 1) && (a->size[0] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(bIn->size[0], a->size[0],
                                              &kb_emlrtECI, (emlrtConstCTX)sp);
                }
                st.site = &yh_emlrtRSI;
                b_st.site = &ek_emlrtRSI;
                dynamic_size_checks(&b_st, B, Hg, B->size[0], Hg->size[0]);
                b_st.site = &fk_emlrtRSI;
                b_mtimes(&b_st, B, Hg, gradNew);
                gradNew_data = gradNew->data;
                st.site = &yh_emlrtRSI;
                if (bIn->size[0] == a->size[0]) {
                  loop_ub = bIn->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    xSol_data[i] -= a_data[i];
                  }
                } else {
                  b_st.site = &yh_emlrtRSI;
                  minus(&b_st, bIn, a);
                  xSol_data = bIn->data;
                }
                b_st.site = &no_emlrtRSI;
                c_st.site = &oo_emlrtRSI;
                assertCompatibleDims(&c_st, bIn, gradNew);
                if (bIn->size[0] == gradNew->size[0]) {
                  loop_ub = bIn->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    xSol_data[i] /= gradNew_data[i];
                  }
                } else {
                  c_st.site = &qq_emlrtRSI;
                  rdivide(&c_st, bIn, gradNew);
                  xSol_data = bIn->data;
                }
                st.site = &ai_emlrtRSI;
                i = b_activeSet->size[0];
                b_activeSet->size[0] = bIn->size[0];
                emxEnsureCapacity_boolean_T(&st, b_activeSet, i, &oh_emlrtRTEI);
                b_activeSet_data = b_activeSet->data;
                loop_ub = bIn->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_activeSet_data[i] = (xSol_data[i] > 0.0);
                }
                b_st.site = &ng_emlrtRSI;
                eml_find(&b_st, b_activeSet, ii);
                ii_data = ii->data;
                i = a->size[0];
                a->size[0] = ii->size[0];
                emxEnsureCapacity_real_T(&st, a, i, &qh_emlrtRTEI);
                a_data = a->data;
                loop_ub = ii->size[0];
                for (i = 0; i < loop_ub; i++) {
                  a_data[i] = ii_data[i];
                }
                if (a->size[0] != 0) {
                  end = bIn->size[0] - 1;
                  last = 0;
                  for (c_i = 0; c_i <= end; c_i++) {
                    if (xSol_data[c_i] > 0.0) {
                      last++;
                    }
                  }
                  i = r4->size[0];
                  r4->size[0] = last;
                  emxEnsureCapacity_int32_T(sp, r4, i, &pg_emlrtRTEI);
                  ii_data = r4->data;
                  last = 0;
                  for (c_i = 0; c_i <= end; c_i++) {
                    if (xSol_data[c_i] > 0.0) {
                      ii_data[last] = c_i + 1;
                      last++;
                    }
                  }
                  st.site = &bi_emlrtRSI;
                  loop_ub = r4->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    if ((ii_data[i] < 1) || (ii_data[i] > bIn->size[0])) {
                      emlrtDynamicBoundsCheckR2012b(ii_data[i], 1, bIn->size[0],
                                                    &fd_emlrtBCI, &st);
                    }
                  }
                  b_st.site = &yo_emlrtRSI;
                  c_st.site = &ap_emlrtRSI;
                  d_st.site = &bp_emlrtRSI;
                  if (r4->size[0] < 1) {
                    emlrtErrorWithMessageIdR2018a(
                        &d_st, &eb_emlrtRTEI,
                        "Coder:toolbox:eml_min_or_max_varDimZero",
                        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
                  }
                  e_st.site = &so_emlrtRSI;
                  last = r4->size[0];
                  if (r4->size[0] <= 2) {
                    if (r4->size[0] == 1) {
                      lambda = xSol_data[ii_data[0] - 1];
                      idxl = 1;
                    } else {
                      beta = xSol_data[ii_data[1] - 1];
                      lambda = xSol_data[ii_data[0] - 1];
                      if ((lambda > beta) ||
                          (muDoubleScalarIsNaN(xSol_data[ii_data[0] - 1]) &&
                           (!muDoubleScalarIsNaN(beta)))) {
                        lambda = beta;
                        idxl = 2;
                      } else {
                        idxl = 1;
                      }
                    }
                  } else {
                    f_st.site = &uo_emlrtRSI;
                    lambda = xSol_data[ii_data[0] - 1];
                    if (!muDoubleScalarIsNaN(lambda)) {
                      idxl = 1;
                    } else {
                      idxl = 0;
                      g_st.site = &vo_emlrtRSI;
                      if (r4->size[0] > 2147483646) {
                        h_st.site = &vc_emlrtRSI;
                        check_forloop_overflow_error(&h_st);
                      }
                      end = 2;
                      exitg3 = false;
                      while ((!exitg3) && (end <= last)) {
                        if (!muDoubleScalarIsNaN(
                                xSol_data[ii_data[end - 1] - 1])) {
                          idxl = end;
                          exitg3 = true;
                        } else {
                          end++;
                        }
                      }
                    }
                    if (idxl == 0) {
                      idxl = 1;
                    } else {
                      f_st.site = &to_emlrtRSI;
                      lambda = xSol_data[ii_data[idxl - 1] - 1];
                      b_a = idxl + 1;
                      g_st.site = &wo_emlrtRSI;
                      if ((idxl + 1 <= r4->size[0]) &&
                          (r4->size[0] > 2147483646)) {
                        h_st.site = &vc_emlrtRSI;
                        check_forloop_overflow_error(&h_st);
                      }
                      for (end = b_a; end <= last; end++) {
                        beta = xSol_data[ii_data[end - 1] - 1];
                        if (lambda > beta) {
                          lambda = beta;
                          idxl = end;
                        }
                      }
                    }
                  }
                  if ((idxl < 1) || (idxl > a->size[0])) {
                    emlrtDynamicBoundsCheckR2012b(
                        idxl, 1, a->size[0], &hd_emlrtBCI, (emlrtConstCTX)sp);
                  }
                  i = (int32_T)a_data[idxl - 1];
                  if ((i < 1) || (i > inactiveConstraintIndices->size[0])) {
                    emlrtDynamicBoundsCheckR2012b(
                        i, 1, inactiveConstraintIndices->size[0], &gd_emlrtBCI,
                        (emlrtConstCTX)sp);
                  }
                  idxl = inactiveConstraintIndices_data[i - 1];
                } else {
                  lambda = 0.0;
                }
              } else {
                lambda = 0.0;
              }
            } else {
              lambda = 0.0;
            }
            if (lambda > 0.0) {
              b_gamma = muDoubleScalarMin(1.0, lambda);
            } else {
              b_gamma = 1.0;
            }
            beta = obj->ArmijoRuleBeta;
            sigma = obj->ArmijoRuleSigma;
            i = a->size[0];
            a->size[0] = Hg->size[0];
            emxEnsureCapacity_real_T(sp, a, i, &wg_emlrtRTEI);
            a_data = a->data;
            loop_ub = Hg->size[0];
            for (i = 0; i < loop_ub; i++) {
              a_data[i] = b_gamma * Hg_data[i];
            }
            if ((x->size[0] != a->size[0]) &&
                ((x->size[0] != 1) && (a->size[0] != 1))) {
              emlrtDimSizeImpxCheckR2021b(x->size[0], a->size[0], &jb_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if (x->size[0] == a->size[0]) {
              i = a->size[0];
              a->size[0] = x->size[0];
              emxEnsureCapacity_real_T(sp, a, i, &xg_emlrtRTEI);
              a_data = a->data;
              loop_ub = x->size[0];
              for (i = 0; i < loop_ub; i++) {
                a_data[i] += x_data[i];
              }
              st.site = &ci_emlrtRSI;
              IKHelpers_computeCost(&st, a, obj->ExtraArgs, &costNew, a__1,
                                    a__2, &r7);
            } else {
              st.site = &ci_emlrtRSI;
              c_binary_expand_op(&st, ci_emlrtRSI, x, a, obj, a__1, a__2,
                                 &costNew, &r6);
              r7 = r6;
            }
            obj->ExtraArgs = r7;
            m = 0.0;
            do {
              exitg1 = 0;
              i = b_y->size[0] * b_y->size[1];
              b_y->size[0] = 1;
              b_y->size[1] = grad->size[0];
              emxEnsureCapacity_real_T(sp, b_y, i, &ah_emlrtRTEI);
              y_data = b_y->data;
              loop_ub = grad->size[0];
              for (i = 0; i < loop_ub; i++) {
                y_data[i] = -sigma * grad_data[i];
              }
              i = gradNew->size[0];
              gradNew->size[0] = Hg->size[0];
              emxEnsureCapacity_real_T(sp, gradNew, i, &rg_emlrtRTEI);
              gradNew_data = gradNew->data;
              loop_ub = Hg->size[0];
              for (i = 0; i < loop_ub; i++) {
                gradNew_data[i] = b_gamma * Hg_data[i];
              }
              st.site = &di_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              c_dynamic_size_checks(&b_st, b_y, gradNew, b_y->size[1],
                                    gradNew->size[0]);
              if (cost - costNew < d_mtimes(b_y, gradNew)) {
                st.site = &ei_emlrtRSI;
                valid = (b_gamma < obj->StepTolerance);
                if (valid) {
                  *exitFlag = StepSizeBelowMinimum;
                  st.site = &fi_emlrtRSI;
                  *err = IKHelpers_evaluateSolution(&st, obj->ExtraArgs);
                  *iter = (real_T)b_i + 1.0;
                  exitg1 = 1;
                } else {
                  b_gamma *= beta;
                  m++;
                  i = a->size[0];
                  a->size[0] = Hg->size[0];
                  emxEnsureCapacity_real_T(sp, a, i, &fh_emlrtRTEI);
                  a_data = a->data;
                  loop_ub = Hg->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    a_data[i] = b_gamma * Hg_data[i];
                  }
                  if ((x->size[0] != a->size[0]) &&
                      ((x->size[0] != 1) && (a->size[0] != 1))) {
                    emlrtDimSizeImpxCheckR2021b(x->size[0], a->size[0],
                                                &ib_emlrtECI,
                                                (emlrtConstCTX)sp);
                  }
                  if (x->size[0] == a->size[0]) {
                    i = a->size[0];
                    a->size[0] = x->size[0];
                    emxEnsureCapacity_real_T(sp, a, i, &jh_emlrtRTEI);
                    a_data = a->data;
                    loop_ub = x->size[0];
                    for (i = 0; i < loop_ub; i++) {
                      a_data[i] += x_data[i];
                    }
                    st.site = &gi_emlrtRSI;
                    IKHelpers_computeCost(&st, a, obj->ExtraArgs, &costNew,
                                          a__1, a__2, &r9);
                  } else {
                    st.site = &gi_emlrtRSI;
                    c_binary_expand_op(&st, gi_emlrtRSI, x, a, obj, a__1, a__2,
                                       &costNew, &r8);
                    r9 = r8;
                  }
                  obj->ExtraArgs = r9;
                }
              } else {
                i = xSol->size[0];
                xSol->size[0] = Hg->size[0];
                emxEnsureCapacity_real_T(sp, xSol, i, &ch_emlrtRTEI);
                xSol_data = xSol->data;
                loop_ub = Hg->size[0];
                for (i = 0; i < loop_ub; i++) {
                  xSol_data[i] = b_gamma * Hg_data[i];
                }
                if ((x->size[0] != xSol->size[0]) &&
                    ((x->size[0] != 1) && (xSol->size[0] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(x->size[0], xSol->size[0],
                                              &hb_emlrtECI, (emlrtConstCTX)sp);
                }
                if (x->size[0] == xSol->size[0]) {
                  i = xSol->size[0];
                  xSol->size[0] = x->size[0];
                  emxEnsureCapacity_real_T(sp, xSol, i, &gh_emlrtRTEI);
                  xSol_data = xSol->data;
                  loop_ub = x->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    xSol_data[i] += x_data[i];
                  }
                } else {
                  st.site = &pq_emlrtRSI;
                  c_plus(&st, xSol, x);
                  xSol_data = xSol->data;
                }
                st.site = &hi_emlrtRSI;
                args = obj->ExtraArgs;
                i = gradNew->size[0];
                gradNew->size[0] = args->GradTemp->size[0];
                emxEnsureCapacity_real_T(&st, gradNew, i, &ih_emlrtRTEI);
                gradNew_data = gradNew->data;
                loop_ub = args->GradTemp->size[0];
                for (i = 0; i < loop_ub; i++) {
                  gradNew_data[i] = args->GradTemp->data[i];
                }
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = 1;
            } else if (m == 0.0) {
              st.site = &ii_emlrtRSI;
              if (muDoubleScalarAbs(b_gamma - lambda) < 1.4901161193847656E-8) {
                i = obj->ConstraintMatrix->size[1];
                if ((idxl < 1) || (idxl > i)) {
                  emlrtDynamicBoundsCheckR2012b(idxl, 1, i, &tc_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                i = a->size[0];
                a->size[0] = obj->ConstraintMatrix->size[0];
                emxEnsureCapacity_real_T(sp, a, i, &mh_emlrtRTEI);
                a_data = a->data;
                loop_ub = obj->ConstraintMatrix->size[0];
                for (i = 0; i < loop_ub; i++) {
                  a_data[i] = obj->ConstraintMatrix
                                  ->data[i + obj->ConstraintMatrix->size[0] *
                                                 (idxl - 1)];
                }
                if (idxl > activeSet->size[0]) {
                  emlrtDynamicBoundsCheckR2012b(idxl, 1, activeSet->size[0],
                                                &bd_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                activeSet_data[idxl - 1] = true;
                end = activeSet->size[0] - 1;
                last = 0;
                for (c_i = 0; c_i <= end; c_i++) {
                  if (activeSet_data[c_i]) {
                    last++;
                  }
                }
                i = r5->size[0];
                r5->size[0] = last;
                emxEnsureCapacity_int32_T(sp, r5, i, &pg_emlrtRTEI);
                ii_data = r5->data;
                last = 0;
                for (c_i = 0; c_i <= end; c_i++) {
                  if (activeSet_data[c_i]) {
                    ii_data[last] = c_i + 1;
                    last++;
                  }
                }
                last = obj->ConstraintMatrix->size[1];
                i = A->size[0] * A->size[1];
                A->size[0] = obj->ConstraintMatrix->size[0];
                A->size[1] = r5->size[0];
                emxEnsureCapacity_real_T(sp, A, i, &ph_emlrtRTEI);
                A_data = A->data;
                loop_ub = r5->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = obj->ConstraintMatrix->size[0];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    if ((ii_data[i] < 1) || (ii_data[i] > last)) {
                      emlrtDynamicBoundsCheckR2012b(
                          ii_data[i], 1, last, &dd_emlrtBCI, (emlrtConstCTX)sp);
                    }
                    A_data[i1 + A->size[0] * i] =
                        obj->ConstraintMatrix
                            ->data[i1 + obj->ConstraintMatrix->size[0] *
                                            (ii_data[i] - 1)];
                  }
                }
                st.site = &ji_emlrtRSI;
                b_st.site = &ek_emlrtRSI;
                b_dynamic_size_checks(&b_st, a, H, a->size[0], H->size[0]);
                b_st.site = &fk_emlrtRSI;
                c_mtimes(&b_st, a, H, b_y);
                st.site = &ji_emlrtRSI;
                b_st.site = &ek_emlrtRSI;
                c_dynamic_size_checks(&b_st, b_y, a, b_y->size[1], a->size[0]);
                i = y->size[0] * y->size[1];
                y->size[0] = a->size[0];
                y->size[1] = a->size[0];
                emxEnsureCapacity_real_T(sp, y, i, &tg_emlrtRTEI);
                y_data = y->data;
                loop_ub = a->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = a->size[0];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    y_data[i1 + y->size[0] * i] = a_data[i1] * a_data[i];
                  }
                }
                st.site = &ji_emlrtRSI;
                b_st.site = &ek_emlrtRSI;
                d_dynamic_size_checks(&b_st, y, H, y->size[1], H->size[0]);
                b_st.site = &fk_emlrtRSI;
                e_mtimes(&b_st, y, H, B);
                st.site = &ji_emlrtRSI;
                b_st.site = &ek_emlrtRSI;
                d_dynamic_size_checks(&b_st, H, B, H->size[1], B->size[0]);
                b_st.site = &fk_emlrtRSI;
                e_mtimes(&b_st, H, B, y);
                y_data = y->data;
                b_gamma = 1.0 / d_mtimes(b_y, a);
                loop_ub = y->size[0] * y->size[1];
                for (i = 0; i < loop_ub; i++) {
                  y_data[i] *= b_gamma;
                }
                if ((H->size[0] != y->size[0]) &&
                    ((H->size[0] != 1) && (y->size[0] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(H->size[0], y->size[0],
                                              &gb_emlrtECI, (emlrtConstCTX)sp);
                }
                if ((H->size[1] != y->size[1]) &&
                    ((H->size[1] != 1) && (y->size[1] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(H->size[1], y->size[1],
                                              &fb_emlrtECI, (emlrtConstCTX)sp);
                }
                if ((H->size[0] == y->size[0]) && (H->size[1] == y->size[1])) {
                  loop_ub = H->size[0] * H->size[1];
                  for (i = 0; i < loop_ub; i++) {
                    H_data[i] -= y_data[i];
                  }
                } else {
                  st.site = &ji_emlrtRSI;
                  c_minus(&st, H, y);
                  H_data = H->data;
                }
                guard1 = true;
              } else {
                guard2 = true;
              }
            } else {
              guard2 = true;
            }
          }
          if (guard2) {
            real_T b_H[2];
            int32_T aux_1_1;
            if ((gradNew->size[0] != grad->size[0]) &&
                ((gradNew->size[0] != 1) && (grad->size[0] != 1))) {
              emlrtDimSizeImpxCheckR2021b(gradNew->size[0], grad->size[0],
                                          &eb_emlrtECI, (emlrtConstCTX)sp);
            }
            if (gradNew->size[0] == grad->size[0]) {
              i = grad->size[0];
              grad->size[0] = gradNew->size[0];
              emxEnsureCapacity_real_T(sp, grad, i, &kh_emlrtRTEI);
              grad_data = grad->data;
              loop_ub = gradNew->size[0];
              for (i = 0; i < loop_ub; i++) {
                grad_data[i] = gradNew_data[i] - grad_data[i];
              }
            } else {
              st.site = &oq_emlrtRSI;
              b_minus(&st, grad, gradNew);
              grad_data = grad->data;
            }
            st.site = &ki_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            e_dynamic_size_checks(&b_st, Hg, grad, Hg->size[0], grad->size[0]);
            b_gamma = h_mtimes(Hg, grad);
            i = b_y->size[0] * b_y->size[1];
            b_y->size[0] = 1;
            b_y->size[1] = grad->size[0];
            emxEnsureCapacity_real_T(sp, b_y, i, &ah_emlrtRTEI);
            y_data = b_y->data;
            loop_ub = grad->size[0];
            for (i = 0; i < loop_ub; i++) {
              y_data[i] = 0.2 * grad_data[i];
            }
            st.site = &ki_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            f_dynamic_size_checks(&b_st, b_y, H, b_y->size[1], H->size[0]);
            b_st.site = &fk_emlrtRSI;
            i_mtimes(&b_st, b_y, H, d_y);
            st.site = &ki_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            c_dynamic_size_checks(&b_st, d_y, grad, d_y->size[1],
                                  grad->size[0]);
            if (b_gamma < d_mtimes(d_y, grad)) {
              i = b_y->size[0] * b_y->size[1];
              b_y->size[0] = 1;
              b_y->size[1] = grad->size[0];
              emxEnsureCapacity_real_T(sp, b_y, i, &ah_emlrtRTEI);
              y_data = b_y->data;
              loop_ub = grad->size[0];
              for (i = 0; i < loop_ub; i++) {
                y_data[i] = 0.8 * grad_data[i];
              }
              st.site = &li_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              f_dynamic_size_checks(&b_st, b_y, H, b_y->size[1], H->size[0]);
              b_st.site = &fk_emlrtRSI;
              i_mtimes(&b_st, b_y, H, d_y);
              st.site = &li_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              c_dynamic_size_checks(&b_st, d_y, grad, d_y->size[1],
                                    grad->size[0]);
              st.site = &li_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              b_dynamic_size_checks(&b_st, grad, H, grad->size[0], H->size[0]);
              b_st.site = &fk_emlrtRSI;
              c_mtimes(&b_st, grad, H, b_y);
              st.site = &li_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              c_dynamic_size_checks(&b_st, b_y, grad, b_y->size[1],
                                    grad->size[0]);
              st.site = &li_emlrtRSI;
              b_st.site = &ek_emlrtRSI;
              e_dynamic_size_checks(&b_st, Hg, grad, Hg->size[0],
                                    grad->size[0]);
              b_gamma = d_mtimes(d_y, grad) / (d_mtimes(b_y, grad) - b_gamma);
            } else {
              b_gamma = 1.0;
            }
            loop_ub = Hg->size[0];
            for (i = 0; i < loop_ub; i++) {
              Hg_data[i] *= b_gamma;
            }
            i = y->size[0] * y->size[1];
            y->size[0] = H->size[0];
            y->size[1] = H->size[1];
            emxEnsureCapacity_real_T(sp, y, i, &rg_emlrtRTEI);
            y_data = y->data;
            loop_ub = H->size[0] * H->size[1];
            for (i = 0; i < loop_ub; i++) {
              y_data[i] = (1.0 - b_gamma) * H_data[i];
            }
            st.site = &mi_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            dynamic_size_checks(&b_st, y, grad, y->size[1], grad->size[0]);
            b_st.site = &fk_emlrtRSI;
            g_mtimes(&b_st, y, grad, a);
            a_data = a->data;
            if ((Hg->size[0] != a->size[0]) &&
                ((Hg->size[0] != 1) && (a->size[0] != 1))) {
              emlrtDimSizeImpxCheckR2021b(Hg->size[0], a->size[0], &db_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if (Hg->size[0] == a->size[0]) {
              loop_ub = Hg->size[0];
              for (i = 0; i < loop_ub; i++) {
                Hg_data[i] += a_data[i];
              }
            } else {
              st.site = &mi_emlrtRSI;
              plus(&st, Hg, a);
              Hg_data = Hg->data;
            }
            st.site = &ni_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            e_dynamic_size_checks(&b_st, Hg, grad, Hg->size[0], grad->size[0]);
            b_gamma = h_mtimes(Hg, grad);
            st.site = &oi_emlrtRSI;
            eye(&st, n, B);
            B_data = B->data;
            i = y->size[0] * y->size[1];
            y->size[0] = Hg->size[0];
            y->size[1] = grad->size[0];
            emxEnsureCapacity_real_T(sp, y, i, &sh_emlrtRTEI);
            y_data = y->data;
            loop_ub = grad->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_loop_ub = Hg->size[0];
              for (i1 = 0; i1 < b_loop_ub; i1++) {
                y_data[i1 + y->size[0] * i] =
                    Hg_data[i1] * grad_data[i] / b_gamma;
              }
            }
            if ((B->size[0] != y->size[0]) &&
                ((B->size[0] != 1) && (y->size[0] != 1))) {
              emlrtDimSizeImpxCheckR2021b(B->size[0], y->size[0], &cb_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if ((B->size[1] != y->size[1]) &&
                ((B->size[1] != 1) && (y->size[1] != 1))) {
              emlrtDimSizeImpxCheckR2021b(B->size[1], y->size[1], &bb_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if ((B->size[0] == y->size[0]) && (B->size[1] == y->size[1])) {
              loop_ub = B->size[0] * B->size[1];
              for (i = 0; i < loop_ub; i++) {
                B_data[i] -= y_data[i];
              }
            } else {
              st.site = &oi_emlrtRSI;
              c_minus(&st, B, y);
            }
            st.site = &pi_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            d_dynamic_size_checks(&b_st, B, H, B->size[1], H->size[0]);
            b_st.site = &fk_emlrtRSI;
            e_mtimes(&b_st, B, H, y);
            st.site = &pi_emlrtRSI;
            b_st.site = &ek_emlrtRSI;
            d_dynamic_size_checks(&b_st, y, B, y->size[1], B->size[1]);
            b_st.site = &fk_emlrtRSI;
            j_mtimes(&b_st, y, B, H);
            H_data = H->data;
            i = y->size[0] * y->size[1];
            y->size[0] = Hg->size[0];
            y->size[1] = Hg->size[0];
            emxEnsureCapacity_real_T(sp, y, i, &wh_emlrtRTEI);
            y_data = y->data;
            loop_ub = Hg->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_loop_ub = Hg->size[0];
              for (i1 = 0; i1 < b_loop_ub; i1++) {
                y_data[i1 + y->size[0] * i] =
                    Hg_data[i1] * Hg_data[i] / b_gamma;
              }
            }
            if ((H->size[0] != y->size[0]) &&
                ((H->size[0] != 1) && (y->size[0] != 1))) {
              emlrtDimSizeImpxCheckR2021b(H->size[0], y->size[0], &ab_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if ((H->size[1] != y->size[1]) &&
                ((H->size[1] != 1) && (y->size[1] != 1))) {
              emlrtDimSizeImpxCheckR2021b(H->size[1], y->size[1], &y_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if ((H->size[0] == y->size[0]) && (H->size[1] == y->size[1])) {
              loop_ub = H->size[0] * H->size[1];
              for (i = 0; i < loop_ub; i++) {
                H_data[i] += y_data[i];
              }
            } else {
              st.site = &pi_emlrtRSI;
              b_plus(&st, H, y);
              H_data = H->data;
            }
            b_H[0] = H->size[0];
            b_H[1] = H->size[1];
            st.site = &qi_emlrtRSI;
            b_eye(&st, b_H, B);
            B_data = B->data;
            loop_ub = B->size[0] * B->size[1];
            for (i = 0; i < loop_ub; i++) {
              B_data[i] *= 1.4901161193847656E-8;
            }
            if ((H->size[0] != B->size[0]) &&
                ((H->size[0] != 1) && (B->size[0] != 1))) {
              emlrtDimSizeImpxCheckR2021b(H->size[0], B->size[0], &x_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            if ((H->size[1] != B->size[1]) &&
                ((H->size[1] != 1) && (B->size[1] != 1))) {
              emlrtDimSizeImpxCheckR2021b(H->size[1], B->size[1], &w_emlrtECI,
                                          (emlrtConstCTX)sp);
            }
            i = y->size[0] * y->size[1];
            if (B->size[0] == 1) {
              y->size[0] = H->size[0];
            } else {
              y->size[0] = B->size[0];
            }
            if (B->size[1] == 1) {
              y->size[1] = H->size[1];
            } else {
              y->size[1] = B->size[1];
            }
            emxEnsureCapacity_real_T(sp, y, i, &xh_emlrtRTEI);
            y_data = y->data;
            last = (H->size[0] != 1);
            end = (H->size[1] != 1);
            idxl = (B->size[0] != 1);
            b_a = (B->size[1] != 1);
            c_i = 0;
            aux_1_1 = 0;
            if (B->size[1] == 1) {
              loop_ub = H->size[1];
            } else {
              loop_ub = B->size[1];
            }
            for (i = 0; i < loop_ub; i++) {
              if (B->size[0] == 1) {
                b_loop_ub = H->size[0];
              } else {
                b_loop_ub = B->size[0];
              }
              for (i1 = 0; i1 < b_loop_ub; i1++) {
                y_data[i1 + y->size[0] * i] =
                    H_data[i1 * last + H->size[0] * c_i] +
                    B_data[i1 * idxl + B->size[0] * aux_1_1];
              }
              aux_1_1 += b_a;
              c_i += end;
            }
            i = B->size[0] * B->size[1];
            B->size[0] = y->size[0];
            B->size[1] = y->size[1];
            emxEnsureCapacity_real_T(sp, B, i, &xh_emlrtRTEI);
            B_data = B->data;
            loop_ub = y->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_loop_ub = y->size[0];
              for (i1 = 0; i1 < b_loop_ub; i1++) {
                B_data[i1 + B->size[0] * i] = y_data[i1 + y->size[0] * i];
              }
            }
            st.site = &qi_emlrtRSI;
            if (!isPositiveDefinite(&st, B)) {
              *exitFlag = HessianNotPositiveSemidefinite;
              st.site = &ri_emlrtRSI;
              *err = IKHelpers_evaluateSolution(&st, obj->ExtraArgs);
              *iter = (real_T)b_i + 1.0;
              exitg2 = 1;
            } else {
              guard1 = true;
            }
          }
          if (guard1) {
            st.site = &si_emlrtRSI;
            if (e_DampedBFGSwGradientProjection(&st, obj, xSol)) {
              i = xSol->size[0];
              xSol->size[0] = x->size[0];
              emxEnsureCapacity_real_T(sp, xSol, i, &uh_emlrtRTEI);
              xSol_data = xSol->data;
              loop_ub = x->size[0];
              for (i = 0; i < loop_ub; i++) {
                xSol_data[i] = x_data[i];
              }
              *exitFlag = SearchDirectionInvalid;
              st.site = &ti_emlrtRSI;
              *err = IKHelpers_evaluateSolution(&st, obj->ExtraArgs);
              *iter = (real_T)b_i + 1.0;
              exitg2 = 1;
            } else {
              i = x->size[0];
              x->size[0] = xSol->size[0];
              emxEnsureCapacity_real_T(sp, x, i, &th_emlrtRTEI);
              x_data = x->data;
              loop_ub = xSol->size[0];
              for (i = 0; i < loop_ub; i++) {
                x_data[i] = xSol_data[i];
              }
              i = grad->size[0];
              grad->size[0] = gradNew->size[0];
              emxEnsureCapacity_real_T(sp, grad, i, &vh_emlrtRTEI);
              grad_data = grad->data;
              loop_ub = gradNew->size[0];
              for (i = 0; i < loop_ub; i++) {
                grad_data[i] = gradNew_data[i];
              }
              cost = costNew;
              b_i++;
            }
          }
        }
      }
    } else {
      *exitFlag = IterationLimitExceeded;
      st.site = &ui_emlrtRSI;
      *err = IKHelpers_evaluateSolution(&st, obj->ExtraArgs);
      *iter = obj->MaxNumIterationInternal;
      exitg2 = 1;
    }
  } while (exitg2 == 0);
  emxFree_boolean_T(sp, &b_activeSet);
  emxFree_real_T(sp, &d_y);
  emxFree_int32_T(sp, &ii);
  emxFree_real_T(sp, &c_y);
  emxFree_real_T(sp, &y);
  emxFree_real_T(sp, &b_y);
  emxFree_int32_T(sp, &r5);
  emxFree_int32_T(sp, &r4);
  emxFree_int32_T(sp, &r3);
  emxFree_int32_T(sp, &r2);
  emxFree_int32_T(sp, &r1);
  emxFree_real_T(sp, &gradNew);
  emxFree_int32_T(sp, &inactiveConstraintIndices);
  emxFree_real_T(sp, &bIn);
  emxFree_real_T(sp, &P);
  emxFree_real_T(sp, &B);
  emxFree_real_T(sp, &Hg);
  emxFree_real_T(sp, &a);
  emxFree_real_T(sp, &A);
  emxFree_boolean_T(sp, &activeSet);
  emxFree_real_T(sp, &H);
  emxFree_real_T(sp, &grad);
  emxFree_real_T(sp, &a__2);
  emxFree_real_T(sp, &x);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (DampedBFGSwGradientProjection.c) */
