/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverseKinematics.c
 *
 * Code generation for function 'inverseKinematics'
 *
 */

/* Include files */
#include "inverseKinematics.h"
#include "CollisionSet.h"
#include "NLPSolverInterface.h"
#include "RigidBodyTree.h"
#include "indexShapeCheck.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_internal_types.h"
#include "inverse_kinematics_mexutil.h"
#include "inverse_kinematics_types.h"
#include "norm.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "emlrt.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ab_emlrtRSI = {
    145,                           /* lineNo */
    "rigidBodyTree/rigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pathName
                                                                            */
};

static emlrtRSInfo bb_emlrtRSI =
    {
        210,                                   /* lineNo */
        "inverseKinematics/inverseKinematics", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI =
    {
        241,                                   /* lineNo */
        "inverseKinematics/inverseKinematics", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI =
    {
        253,                                   /* lineNo */
        "inverseKinematics/inverseKinematics", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    1,               /* lineNo */
    "System/System", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/system/coder/+matlab/+system/"
    "+coder/System.p" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    1,                          /* lineNo */
    "SystemProp/setProperties", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/system/coder/+matlab/+system/"
    "+coder/SystemProp.p" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI =
    {
        455,                                     /* lineNo */
        "inverseKinematics/set.SolverAlgorithm", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    1,                                       /* lineNo */
    "NLPSolverInterface/NLPSolverInterface", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pathName */
};

static emlrtRSInfo jb_emlrtRSI = {
    70,                                                            /* lineNo */
    "DampedBFGSwGradientProjection/DampedBFGSwGradientProjection", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo kb_emlrtRSI = {
    112,                                                           /* lineNo */
    "DampedBFGSwGradientProjection/DampedBFGSwGradientProjection", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    113,                                                           /* lineNo */
    "DampedBFGSwGradientProjection/DampedBFGSwGradientProjection", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/DampedBFGSwGradientProjection.m" /* pathName */
};

static emlrtRSInfo mb_emlrtRSI = {
    42,                                      /* lineNo */
    "SystemTimeProvider/SystemTimeProvider", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/SystemTimeProvider.m" /* pathName */
};

static emlrtRSInfo nb_emlrtRSI = {
    1,                           /* lineNo */
    "TimeProvider/TimeProvider", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/TimeProvider.m" /* pathName */
};

static emlrtRSInfo ob_emlrtRSI = {
    1,                                /* lineNo */
    "ProcessConstructorArguments/do", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/system/coder/+matlab/+system/"
    "+coder/ProcessConstructorArguments.p" /* pathName */
};

static emlrtRSInfo pb_emlrtRSI =
    {
        478,                                   /* lineNo */
        "inverseKinematics/set.RigidBodyTree", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo qb_emlrtRSI =
    {
        479,                                   /* lineNo */
        "inverseKinematics/set.RigidBodyTree", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo rb_emlrtRSI =
    {
        483,                                   /* lineNo */
        "inverseKinematics/set.RigidBodyTree", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo sb_emlrtRSI = {
    365,                  /* lineNo */
    "rigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pathName
                                                                            */
};

static emlrtRSInfo tb_emlrtRSI = {
    364,                  /* lineNo */
    "rigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pathName
                                                                            */
};

static emlrtRSInfo ub_emlrtRSI = {
    539,                  /* lineNo */
    "RigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo vb_emlrtRSI = {
    540,                  /* lineNo */
    "RigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo wb_emlrtRSI = {
    545,                  /* lineNo */
    "RigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo xb_emlrtRSI = {
    558,                  /* lineNo */
    "RigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo yb_emlrtRSI = {
    544,                  /* lineNo */
    "RigidBodyTree/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo lc_emlrtRSI = {
    2189,                         /* lineNo */
    "RigidBodyTree/get.BaseName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo mc_emlrtRSI = {
    2194,                         /* lineNo */
    "RigidBodyTree/set.BaseName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI = {
    2197,                         /* lineNo */
    "RigidBodyTree/set.BaseName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI = {
    2205,                         /* lineNo */
    "RigidBodyTree/set.BaseName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bd_emlrtRSI = {
    2256,                        /* lineNo */
    "RigidBodyTree/set.Gravity", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo oe_emlrtRSI = {
    172,                           /* lineNo */
    "rigidBodyTree/rigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pathName
                                                                            */
};

static emlrtRSInfo pe_emlrtRSI = {
    182,                           /* lineNo */
    "rigidBodyTree/rigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pathName
                                                                            */
};

static emlrtRSInfo kf_emlrtRSI = {
    31,                                                            /* lineNo */
    "inv",                                                         /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

static emlrtRSInfo nf_emlrtRSI =
    {
        158,                          /* lineNo */
        "inverseKinematics/stepImpl", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo of_emlrtRSI =
    {
        159,                          /* lineNo */
        "inverseKinematics/stepImpl", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo pf_emlrtRSI =
    {
        262,                             /* lineNo */
        "inverseKinematics/setPoseGoal", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo qf_emlrtRSI =
    {
        263,                             /* lineNo */
        "inverseKinematics/setPoseGoal", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo rf_emlrtRSI =
    {
        265,                             /* lineNo */
        "inverseKinematics/setPoseGoal", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo sf_emlrtRSI =
    {
        274,                             /* lineNo */
        "inverseKinematics/setPoseGoal", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo tf_emlrtRSI =
    {
        275,                             /* lineNo */
        "inverseKinematics/setPoseGoal", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo uf_emlrtRSI =
    {
        311,                       /* lineNo */
        "inverseKinematics/solve", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo vf_emlrtRSI =
    {
        313,                       /* lineNo */
        "inverseKinematics/solve", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo wf_emlrtRSI =
    {
        319,                       /* lineNo */
        "inverseKinematics/solve", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo xf_emlrtRSI =
    {
        320,                       /* lineNo */
        "inverseKinematics/solve", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo yf_emlrtRSI =
    {
        324,                       /* lineNo */
        "inverseKinematics/solve", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo xp_emlrtRSI = {
    1951,                                /* lineNo */
    "RigidBodyTree/kinematicPathToBase", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo yp_emlrtRSI = {
    1964,                                /* lineNo */
    "RigidBodyTree/kinematicPathToBase", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo aq_emlrtRSI = {
    736,                                          /* lineNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bq_emlrtRSI = {
    125,                                                          /* lineNo */
    "colon",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

static emlrtRSInfo cq_emlrtRSI = {
    319,                                                          /* lineNo */
    "eml_float_colon",                                            /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

static emlrtRSInfo eq_emlrtRSI = {
    698,                                 /* lineNo */
    "RigidBodyTree/formatConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo fq_emlrtRSI = {
    707,                                 /* lineNo */
    "RigidBodyTree/formatConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo gq_emlrtRSI = {
    708,                                 /* lineNo */
    "RigidBodyTree/formatConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtBCInfo h_emlrtBCI = {
    0,                    /* iFirst */
    5,                    /* iLast */
    554,                  /* lineNo */
    45,                   /* colNo */
    "",                   /* aName */
    "RigidBodyTree/copy", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    554,                  /* lineNo */
    45,                   /* colNo */
    "RigidBodyTree/copy", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtRTEInfo s_emlrtRTEI = {
    419,                                                          /* lineNo */
    15,                                                           /* colNo */
    "assert_pmaxsize",                                            /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pName */
};

static emlrtECInfo g_emlrtECI = {
    1,                                   /* nDims */
    708,                                 /* lineNo */
    25,                                  /* colNo */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo lb_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    708,                                 /* lineNo */
    58,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo ib_emlrtDCI = {
    708,                                 /* lineNo */
    58,                                  /* colNo */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo mb_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    708,                                 /* lineNo */
    48,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo jb_emlrtDCI = {
    708,                                 /* lineNo */
    48,                                  /* colNo */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtECInfo h_emlrtECI = {
    2,                                   /* nDims */
    707,                                 /* lineNo */
    25,                                  /* colNo */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo nb_emlrtBCI = {
    1,                                   /* iFirst */
    6,                                   /* iLast */
    706,                                 /* lineNo */
    53,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo kb_emlrtDCI = {
    706,                                 /* lineNo */
    53,                                  /* colNo */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtRTEInfo t_emlrtRTEI = {
    703,                                 /* lineNo */
    25,                                  /* colNo */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo ob_emlrtBCI = {
    -1,                                           /* iFirst */
    -1,                                           /* iLast */
    740,                                          /* lineNo */
    39,                                           /* colNo */
    "",                                           /* aName */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo lb_emlrtDCI = {
    740,                                          /* lineNo */
    39,                                           /* colNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = {
    -1,                                           /* iFirst */
    -1,                                           /* iLast */
    740,                                          /* lineNo */
    37,                                           /* colNo */
    "",                                           /* aName */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo i_emlrtECI = {
    -1,                                           /* nDims */
    736,                                          /* lineNo */
    21,                                           /* colNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo qb_emlrtBCI = {
    -1,                                           /* iFirst */
    -1,                                           /* iLast */
    733,                                          /* lineNo */
    33,                                           /* colNo */
    "",                                           /* aName */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo rb_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    1964,                                /* lineNo */
    38,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/kinematicPathToBase", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo sb_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    1964,                                /* lineNo */
    36,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/kinematicPathToBase", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo j_emlrtECI =
    {
        -1,                        /* nDims */
        322,                       /* lineNo */
        13,                        /* colNo */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtDCInfo mb_emlrtDCI = {
    729,                                          /* lineNo */
    34,                                           /* colNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo nb_emlrtDCI = {
    729,                                          /* lineNo */
    34,                                           /* colNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo tb_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    707,                                 /* lineNo */
    27,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    708,                                 /* lineNo */
    27,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo ob_emlrtDCI =
    {
        319,                       /* lineNo */
        13,                        /* colNo */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtDCInfo pb_emlrtDCI =
    {
        319,                       /* lineNo */
        13,                        /* colNo */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        4                      /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = {
    0,                                   /* iFirst */
    5,                                   /* iLast */
    1956,                                /* lineNo */
    35,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/kinematicPathToBase", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    1960,                                /* lineNo */
    29,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/kinematicPathToBase", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo xb_emlrtBCI = {
    0,                                   /* iFirst */
    5,                                   /* iLast */
    1961,                                /* lineNo */
    39,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/kinematicPathToBase", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo qb_emlrtDCI = {
    1961,                                /* lineNo */
    39,                                  /* colNo */
    "RigidBodyTree/kinematicPathToBase", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = {
    1,                                            /* iFirst */
    6,                                            /* iLast */
    728,                                          /* lineNo */
    46,                                           /* colNo */
    "",                                           /* aName */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo rb_emlrtDCI = {
    728,                                          /* lineNo */
    46,                                           /* colNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo ac_emlrtBCI = {
    -1,                                           /* iFirst */
    -1,                                           /* iLast */
    728,                                          /* lineNo */
    58,                                           /* colNo */
    "",                                           /* aName */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo sb_emlrtDCI =
    {
        320,                       /* lineNo */
        13,                        /* colNo */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtDCInfo tb_emlrtDCI =
    {
        320,                       /* lineNo */
        13,                        /* colNo */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        4                      /* checkKind */
};

static emlrtDCInfo ub_emlrtDCI =
    {
        322,                       /* lineNo */
        19,                        /* colNo */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo bc_emlrtBCI =
    {
        -1,                        /* iFirst */
        -1,                        /* iLast */
        322,                       /* lineNo */
        19,                        /* colNo */
        "",                        /* aName */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo vb_emlrtDCI = {
    736,                                          /* lineNo */
    32,                                           /* colNo */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo cc_emlrtBCI = {
    -1,                                           /* iFirst */
    -1,                                           /* iLast */
    736,                                          /* lineNo */
    32,                                           /* colNo */
    "",                                           /* aName */
    "RigidBodyTree/bodyIndicesToPositionIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI =
    {
        -1,                        /* iFirst */
        -1,                        /* iLast */
        322,                       /* lineNo */
        47,                        /* colNo */
        "",                        /* aName */
        "inverseKinematics/solve", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtBCInfo ec_emlrtBCI = {
    0,                                   /* iFirst */
    5,                                   /* iLast */
    704,                                 /* lineNo */
    39,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo fc_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    707,                                 /* lineNo */
    25,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo gc_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    708,                                 /* lineNo */
    25,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/formatConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo wc_emlrtRTEI = {
    2189,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = {
    2203,            /* lineNo */
    17,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ad_emlrtRTEI = {
    540,             /* lineNo */
    33,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ne_emlrtRTEI =
    {
        319,                 /* lineNo */
        73,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo oe_emlrtRTEI =
    {
        319,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo pe_emlrtRTEI =
    {
        320,                 /* lineNo */
        31,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo qe_emlrtRTEI = {
    728,             /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo re_emlrtRTEI =
    {
        320,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo se_emlrtRTEI =
    {
        322,                 /* lineNo */
        19,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo te_emlrtRTEI = {
    28,                                                           /* lineNo */
    9,                                                            /* colNo */
    "colon",                                                      /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pName */
};

static emlrtRTEInfo ue_emlrtRTEI = {
    736,             /* lineNo */
    32,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ve_emlrtRTEI = {
    736,             /* lineNo */
    63,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo we_emlrtRTEI = {
    705,             /* lineNo */
    32,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo xe_emlrtRTEI = {
    320,                                                          /* lineNo */
    20,                                                           /* colNo */
    "colon",                                                      /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pName */
};

static emlrtRTEInfo ye_emlrtRTEI = {
    707,             /* lineNo */
    42,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo af_emlrtRTEI =
    {
        311,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo bf_emlrtRTEI =
    {
        155,                 /* lineNo */
        41,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo cf_emlrtRTEI = {
    728,             /* lineNo */
    58,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo df_emlrtRTEI = {
    736,             /* lineNo */
    44,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ef_emlrtRTEI =
    {
        324,                 /* lineNo */
        20,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo ff_emlrtRTEI = {
    698,             /* lineNo */
    21,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo gf_emlrtRTEI =
    {
        287,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo hf_emlrtRTEI =
    {
        262,                 /* lineNo */
        23,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

/* Function Declarations */
static void inverseKinematics_setPoseGoal(const emlrtStack *sp,
                                          inverseKinematics *obj,
                                          const real_T tform[16]);

/* Function Definitions */
static void inverseKinematics_setPoseGoal(const emlrtStack *sp,
                                          inverseKinematics *obj,
                                          const real_T tform[16])
{
  static const real_T dv[6] = {0.5, 0.5, 0.5, 1.0, 1.0, 0.2};
  static const int32_T b_iv[2] = {1, 6};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  static const char_T b[5] = {'l', 'i', 'n', 'k', '6'};
  static const char_T b_cv[5] = {'l', 'i', 'n', 'k', '6'};
  c_robotics_manip_internal_IKExt *args;
  c_robotics_manip_internal_Rigid *c_obj;
  e_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_char_T *b_basename;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  real_T weightMatrix[36];
  real_T x[9];
  real_T y[9];
  real_T maxval[3];
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int32_T bid;
  int32_T exitg1;
  int32_T itmp;
  int32_T p1;
  int32_T p3;
  char_T *basename_data;
  boolean_T b_bool;
  boolean_T exitg2;
  boolean_T guard1 = false;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_char_T(sp, &b_basename, &hf_emlrtRTEI);
  st.site = &pf_emlrtRSI;
  b_obj = obj->RigidBodyTreeInternal;
  b_st.site = &lc_emlrtRSI;
  itmp = b_basename->size[0] * b_basename->size[1];
  b_basename->size[0] = 1;
  b_basename->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(&b_st, b_basename, itmp, &wc_emlrtRTEI);
  basename_data = b_basename->data;
  p1 = b_obj->Base.NameInternal->size[1];
  for (itmp = 0; itmp < p1; itmp++) {
    basename_data[itmp] = b_obj->Base.NameInternal->data[itmp];
  }
  b_bool = false;
  if (b_basename->size[1] == 5) {
    p1 = 0;
    do {
      exitg1 = 0;
      if (p1 < 5) {
        if (basename_data[p1] != b_cv[p1]) {
          exitg1 = 1;
        } else {
          p1++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    st.site = &qf_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &st, &d_emlrtRTEI,
        "robotics:robotmanip:inversekinematics:EndEffectorIsBase",
        "robotics:robotmanip:inversekinematics:EndEffectorIsBase", 0);
  }
  st.site = &rf_emlrtRSI;
  b_obj = obj->RigidBodyTreeInternal;
  b_st.site = &nd_emlrtRSI;
  bid = -1;
  c_st.site = &qc_emlrtRSI;
  itmp = b_basename->size[0] * b_basename->size[1];
  b_basename->size[0] = 1;
  b_basename->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(&c_st, b_basename, itmp, &yc_emlrtRTEI);
  basename_data = b_basename->data;
  p1 = b_obj->Base.NameInternal->size[1];
  for (itmp = 0; itmp < p1; itmp++) {
    basename_data[itmp] = b_obj->Base.NameInternal->data[itmp];
  }
  b_bool = false;
  if (b_basename->size[1] == 5) {
    p1 = 0;
    do {
      exitg1 = 0;
      if (p1 < 5) {
        if (basename_data[p1] != b_cv[p1]) {
          exitg1 = 1;
        } else {
          p1++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    bid = 0;
  } else {
    absx21 = b_obj->NumBodies;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, absx21, mxDOUBLE_CLASS,
                                  (int32_T)absx21, &h_emlrtRTEI, &b_st);
    p3 = 0;
    exitg2 = false;
    while ((!exitg2) && (p3 <= (int32_T)absx21 - 1)) {
      c_st.site = &rc_emlrtRSI;
      if (p3 > 5) {
        emlrtDynamicBoundsCheckR2012b(6, 0, 5, &j_emlrtBCI, &c_st);
      }
      c_obj = b_obj->Bodies[p3];
      itmp = b_basename->size[0] * b_basename->size[1];
      b_basename->size[0] = 1;
      b_basename->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(&c_st, b_basename, itmp, &yc_emlrtRTEI);
      basename_data = b_basename->data;
      p1 = c_obj->NameInternal->size[1];
      for (itmp = 0; itmp < p1; itmp++) {
        basename_data[itmp] = c_obj->NameInternal->data[itmp];
      }
      b_bool = false;
      if (b_basename->size[1] == 5) {
        p1 = 0;
        do {
          exitg1 = 0;
          if (p1 < 5) {
            if (basename_data[p1] != b_cv[p1]) {
              exitg1 = 1;
            } else {
              p1++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        bid = p3 + 1;
        exitg2 = true;
      } else {
        p3++;
      }
    }
  }
  emxFree_char_T(&b_st, &b_basename);
  if (bid == -1) {
    b_st.site = &od_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI, "robotics:robotmanip:rigidbodytree:BodyNotFound",
        "robotics:robotmanip:rigidbodytree:BodyNotFound", 3, 4, 5, &b[0]);
  }
  st.site = &sf_emlrtRSI;
  for (itmp = 0; itmp < 3; itmp++) {
    p1 = itmp << 2;
    x[3 * itmp] = tform[p1];
    x[3 * itmp + 1] = tform[p1 + 1];
    x[3 * itmp + 2] = tform[p1 + 2];
  }
  p1 = 0;
  bid = 3;
  p3 = 6;
  absx11 = muDoubleScalarAbs(tform[0]);
  absx21 = muDoubleScalarAbs(tform[1]);
  absx31 = muDoubleScalarAbs(tform[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    bid = 0;
    x[0] = tform[1];
    x[1] = tform[0];
    x[3] = tform[5];
    x[4] = tform[4];
    x[6] = tform[9];
    x[7] = tform[8];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    x[0] = tform[2];
    x[2] = tform[0];
    x[3] = tform[6];
    x[5] = tform[4];
    x[6] = tform[10];
    x[8] = tform[8];
  }
  x[1] /= x[0];
  x[2] /= x[0];
  x[4] -= x[1] * x[3];
  x[5] -= x[2] * x[3];
  x[7] -= x[1] * x[6];
  x[8] -= x[2] * x[6];
  if (muDoubleScalarAbs(x[5]) > muDoubleScalarAbs(x[4])) {
    itmp = bid;
    bid = p3;
    p3 = itmp;
    absx11 = x[1];
    x[1] = x[2];
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }
  x[5] /= x[4];
  x[8] -= x[5] * x[7];
  absx11 = (x[1] * x[5] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  y[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  y[bid] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  y[bid + 1] = absx21;
  y[bid + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  y[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
  b_st.site = &kf_emlrtRSI;
  for (itmp = 0; itmp < 3; itmp++) {
    p1 = itmp << 2;
    x[3 * itmp] = tform[p1];
    x[3 * itmp + 1] = tform[p1 + 1];
    x[3 * itmp + 2] = tform[p1 + 2];
  }
  absx11 = b_norm(x);
  absx21 = b_norm(y);
  absx31 = 1.0 / (absx11 * absx21);
  if ((absx11 == 0.0) || (absx21 == 0.0) || (absx31 == 0.0)) {
    c_st.site = &lf_emlrtRSI;
    b_warning(&c_st);
  } else if (muDoubleScalarIsNaN(absx31) || (absx31 < 2.2204460492503131E-16)) {
    char_T str[14];
    c_st.site = &mf_emlrtRSI;
    b_y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(&c_st, 6, m, &rfmt[0]);
    emlrtAssign(&b_y, m);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(absx31);
    emlrtAssign(&c_y, m);
    d_st.site = &mq_emlrtRSI;
    emlrt_marshallIn(&d_st, b_sprintf(&d_st, b_y, c_y, &d_emlrtMCI),
                     "<output of sprintf>", str);
    c_st.site = &mf_emlrtRSI;
    c_warning(&c_st, str);
  }
  for (itmp = 0; itmp < 3; itmp++) {
    x[3 * itmp] = y[3 * itmp] - tform[itmp];
    p1 = 3 * itmp + 1;
    x[p1] = y[p1] - tform[itmp + 4];
    p1 = 3 * itmp + 2;
    x[p1] = y[p1] - tform[itmp + 8];
  }
  for (p3 = 0; p3 < 9; p3++) {
    y[p3] = muDoubleScalarAbs(x[p3]);
  }
  for (p1 = 0; p1 < 3; p1++) {
    absx21 = y[3 * p1];
    maxval[p1] = absx21;
    absx11 = y[3 * p1 + 1];
    if (muDoubleScalarIsNaN(absx11)) {
      b_bool = false;
    } else if (muDoubleScalarIsNaN(absx21)) {
      b_bool = true;
    } else {
      b_bool = (absx21 < absx11);
    }
    if (b_bool) {
      absx21 = absx11;
      maxval[p1] = absx11;
    }
    absx11 = y[3 * p1 + 2];
    if (muDoubleScalarIsNaN(absx11)) {
      b_bool = false;
    } else if (muDoubleScalarIsNaN(absx21)) {
      b_bool = true;
    } else {
      b_bool = (absx21 < absx11);
    }
    if (b_bool) {
      maxval[p1] = absx11;
    }
  }
  if (!muDoubleScalarIsNaN(maxval[0])) {
    p1 = 1;
  } else {
    p1 = 0;
    p3 = 2;
    exitg2 = false;
    while ((!exitg2) && (p3 < 4)) {
      if (!muDoubleScalarIsNaN(maxval[p3 - 1])) {
        p1 = p3;
        exitg2 = true;
      } else {
        p3++;
      }
    }
  }
  if (p1 == 0) {
    absx11 = maxval[0];
  } else {
    absx11 = maxval[p1 - 1];
    itmp = p1 + 1;
    for (p3 = itmp; p3 < 4; p3++) {
      absx21 = maxval[p3 - 1];
      if (absx11 < absx21) {
        absx11 = absx21;
      }
    }
  }
  guard1 = false;
  if (absx11 > 0.0001) {
    guard1 = true;
  } else {
    real_T d_y;
    absx11 = 3.3121686421112381E-170;
    absx21 = muDoubleScalarAbs(tform[3]);
    if (absx21 > 3.3121686421112381E-170) {
      d_y = 1.0;
      absx11 = absx21;
    } else {
      absx31 = absx21 / 3.3121686421112381E-170;
      d_y = absx31 * absx31;
    }
    absx21 = muDoubleScalarAbs(tform[7]);
    if (absx21 > absx11) {
      absx31 = absx11 / absx21;
      d_y = d_y * absx31 * absx31 + 1.0;
      absx11 = absx21;
    } else {
      absx31 = absx21 / absx11;
      d_y += absx31 * absx31;
    }
    absx21 = muDoubleScalarAbs(tform[11]);
    if (absx21 > absx11) {
      absx31 = absx11 / absx21;
      d_y = d_y * absx31 * absx31 + 1.0;
      absx11 = absx21;
    } else {
      absx31 = absx21 / absx11;
      d_y += absx31 * absx31;
    }
    absx21 = muDoubleScalarAbs(tform[15] - 1.0);
    if (absx21 > absx11) {
      absx31 = absx11 / absx21;
      d_y = d_y * absx31 * absx31 + 1.0;
      absx11 = absx21;
    } else {
      absx31 = absx21 / absx11;
      d_y += absx31 * absx31;
    }
    d_y = absx11 * muDoubleScalarSqrt(d_y);
    if (d_y > 1.0E-7) {
      guard1 = true;
    }
  }
  if (guard1) {
    st.site = &tf_emlrtRSI;
    b_st.site = &ne_emlrtRSI;
    d_warning(&b_st);
  }
  memset(&weightMatrix[0], 0, 36U * sizeof(real_T));
  for (p1 = 0; p1 < 6; p1++) {
    weightMatrix[p1 + 6 * p1] = dv[p1];
  }
  args = obj->Solver->ExtraArgs;
  for (itmp = 0; itmp < 36; itmp++) {
    args->WeightMatrix[itmp] = weightMatrix[itmp];
  }
  itmp = args->BodyName->size[0] * args->BodyName->size[1];
  args->BodyName->size[0] = 1;
  args->BodyName->size[1] = 5;
  emxEnsureCapacity_char_T(sp, args->BodyName, itmp, &gf_emlrtRTEI);
  for (itmp = 0; itmp < 5; itmp++) {
    args->BodyName->data[itmp] = b[itmp];
  }
  for (itmp = 0; itmp < 16; itmp++) {
    args->Tform[itmp] = tform[itmp];
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

inverseKinematics *c_inverseKinematics_inverseKine(const emlrtStack *sp,
                                                   inverseKinematics *obj,
                                                   b_rigidBodyTree *varargin_2)
{
  c_robotics_core_internal_Damped *iobj_0;
  c_robotics_manip_internal_Rigid *body;
  c_robotics_manip_internal_Rigid *iobj_1;
  c_robotics_manip_internal_Rigid *parent;
  d_robotics_manip_internal_Colli *iobj_2;
  d_robotics_manip_internal_Rigid *d_obj;
  e_robotics_manip_internal_Rigid *newRobotInternal;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  emxArray_char_T *b_basename;
  inverseKinematics *b_obj;
  inverseKinematics *c_obj;
  rigidBodyJoint *b_iobj_0;
  real_T g[3];
  real_T bid;
  int32_T i;
  int32_T loop_ub;
  char_T *basename_data;
  boolean_T exitg1;
  boolean_T p;
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
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  st.site = &bb_emlrtRSI;
  b_st.site = &eb_emlrtRSI;
  c_st.site = &fb_emlrtRSI;
  b_st.site = &eb_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  b_obj->isInitialized = 0;
  st.site = &bb_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  st.site = &cb_emlrtRSI;
  c_obj = b_obj;
  iobj_0 = &b_obj->_pobj6;
  b_st.site = &hb_emlrtRSI;
  c_st.site = &jb_emlrtRSI;
  d_st.site = &ib_emlrtRSI;
  b_obj->_pobj6.MaxNumIteration = 1500.0;
  b_obj->_pobj6.MaxTime = 10.0;
  b_obj->_pobj6.GradientTolerance = 1.0E-7;
  b_obj->_pobj6.SolutionTolerance = 1.0E-6;
  b_obj->_pobj6.ArmijoRuleBeta = 0.4;
  b_obj->_pobj6.ArmijoRuleSigma = 1.0E-5;
  b_obj->_pobj6.ConstraintsOn = true;
  b_obj->_pobj6.RandomRestart = true;
  b_obj->_pobj6.StepTolerance = 1.0E-14;
  b_obj->_pobj6.ConstraintMatrix->size[0] = 0;
  b_obj->_pobj6.ConstraintMatrix->size[1] = 0;
  b_obj->_pobj6.ConstraintBound->size[0] = 0;
  c_st.site = &kb_emlrtRSI;
  d_st.site = &mb_emlrtRSI;
  e_st.site = &nb_emlrtRSI;
  iobj_0->TimeObj.StartTime.tv_sec = 0.0;
  iobj_0->TimeObj.StartTime.tv_nsec = 0.0;
  c_st.site = &lb_emlrtRSI;
  d_st.site = &mb_emlrtRSI;
  e_st.site = &nb_emlrtRSI;
  iobj_0->TimeObjInternal.StartTime.tv_sec = 0.0;
  iobj_0->TimeObjInternal.StartTime.tv_nsec = 0.0;
  b_obj->_pobj6.matlabCodegenIsDeleted = false;
  c_obj->Solver = &b_obj->_pobj6;
  st.site = &db_emlrtRSI;
  c_obj = b_obj;
  b_iobj_0 = &b_obj->_pobj1[0];
  iobj_1 = &b_obj->_pobj2[0];
  iobj_2 = &b_obj->_pobj3[0];
  b_st.site = &fb_emlrtRSI;
  c_st.site = &ob_emlrtRSI;
  d_st.site = &ob_emlrtRSI;
  e_st.site = &pb_emlrtRSI;
  bid = varargin_2->TreeInternal->NumNonFixedBodies;
  if (bid == 0.0) {
    e_st.site = &qb_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &e_st, &d_emlrtRTEI,
        "robotics:robotmanip:inversekinematics:RigidBodyTreeFixed",
        "robotics:robotmanip:inversekinematics:RigidBodyTreeFixed", 0);
  }
  e_st.site = &rb_emlrtRSI;
  f_st.site = &tb_emlrtRSI;
  d_obj = varargin_2->TreeInternal;
  g_st.site = &ub_emlrtRSI;
  newRobotInternal = b_RigidBodyTree_RigidBodyTree(&g_st, &b_obj->_pobj4);
  emxInit_char_T(&f_st, &b_basename, &ad_emlrtRTEI);
  g_st.site = &vb_emlrtRSI;
  h_st.site = &lc_emlrtRSI;
  i = b_basename->size[0] * b_basename->size[1];
  b_basename->size[0] = 1;
  b_basename->size[1] = d_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(&h_st, b_basename, i, &wc_emlrtRTEI);
  basename_data = b_basename->data;
  loop_ub = d_obj->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    basename_data[i] = d_obj->Base.NameInternal->data[i];
  }
  g_st.site = &vb_emlrtRSI;
  h_st.site = &mc_emlrtRSI;
  i_st.site = &y_emlrtRSI;
  if (b_basename->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &i_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:rigidBodyTree:expectedNonempty", 3, 4, 8, "baseName");
  }
  h_st.site = &nc_emlrtRSI;
  bid = c_RigidBodyTree_findBodyIndexBy(&h_st, newRobotInternal, b_basename);
  if (!(bid == 0.0)) {
    if (bid < 0.0) {
      i = newRobotInternal->Base.NameInternal->size[0] *
          newRobotInternal->Base.NameInternal->size[1];
      newRobotInternal->Base.NameInternal->size[0] = 1;
      newRobotInternal->Base.NameInternal->size[1] = b_basename->size[1];
      emxEnsureCapacity_char_T(&g_st, newRobotInternal->Base.NameInternal, i,
                               &xc_emlrtRTEI);
      loop_ub = b_basename->size[1];
      for (i = 0; i < loop_ub; i++) {
        newRobotInternal->Base.NameInternal->data[i] = basename_data[i];
      }
    } else {
      h_st.site = &oc_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &h_st, &e_emlrtRTEI,
          "robotics:robotmanip:rigidbodytree:BaseNameCollision",
          "robotics:robotmanip:rigidbodytree:BaseNameCollision", 3, 4,
          b_basename->size[1], &basename_data[0]);
    }
  }
  g_st.site = &yb_emlrtRSI;
  newRobotInternal->Base.CollisionsInternal =
      CollisionSet_copy(&g_st, d_obj->Base.CollisionsInternal,
                        &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[0]);
  g_st.site = &wb_emlrtRSI;
  g[0] = d_obj->Gravity[0];
  g[1] = d_obj->Gravity[1];
  g[2] = d_obj->Gravity[2];
  h_st.site = &bd_emlrtRSI;
  i_st.site = &y_emlrtRSI;
  p = true;
  loop_ub = 0;
  exitg1 = false;
  while ((!exitg1) && (loop_ub < 3)) {
    if (!muDoubleScalarIsNaN(g[loop_ub])) {
      loop_ub++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &i_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:rigidBodyTree:expectedNonNaN", 3, 4, 7, "Gravity");
  }
  i_st.site = &y_emlrtRSI;
  p = true;
  loop_ub = 0;
  exitg1 = false;
  while ((!exitg1) && (loop_ub < 3)) {
    if ((!muDoubleScalarIsInf(g[loop_ub])) &&
        (!muDoubleScalarIsNaN(g[loop_ub]))) {
      loop_ub++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &i_st, &g_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:rigidBodyTree:expectedFinite", 3, 4, 7, "Gravity");
  }
  if (d_obj->NumBodies >= 1.0) {
    body = d_obj->Bodies[0];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &c_emlrtDCI, &f_st);
      }
      if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &h_emlrtBCI,
                                      &f_st);
      }
      parent = d_obj->Bodies[(int32_T)bid - 1];
    } else {
      parent = &d_obj->Base;
    }
    g_st.site = &xb_emlrtRSI;
    i = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(&g_st, b_basename, i, &yc_emlrtRTEI);
    basename_data = b_basename->data;
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      basename_data[i] = parent->NameInternal->data[i];
    }
    g_st.site = &xb_emlrtRSI;
    RigidBodyTree_addBody(&g_st, newRobotInternal, body, b_basename,
                          &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[1],
                          &(&(&(&(&(&b_iobj_0[0])[0])[0])[0])[0])[0],
                          &(&(&(&(&(&iobj_1[0])[0])[0])[0])[0])[0]);
  }
  if (d_obj->NumBodies >= 2.0) {
    body = d_obj->Bodies[1];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &c_emlrtDCI, &f_st);
      }
      if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &h_emlrtBCI,
                                      &f_st);
      }
      parent = d_obj->Bodies[(int32_T)bid - 1];
    } else {
      parent = &d_obj->Base;
    }
    g_st.site = &xb_emlrtRSI;
    i = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(&g_st, b_basename, i, &yc_emlrtRTEI);
    basename_data = b_basename->data;
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      basename_data[i] = parent->NameInternal->data[i];
    }
    g_st.site = &xb_emlrtRSI;
    RigidBodyTree_addBody(&g_st, newRobotInternal, body, b_basename,
                          &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[3],
                          &(&(&(&(&(&b_iobj_0[0])[0])[0])[0])[0])[2],
                          &(&(&(&(&(&iobj_1[0])[0])[0])[0])[0])[1]);
  }
  if (d_obj->NumBodies >= 3.0) {
    body = d_obj->Bodies[2];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &c_emlrtDCI, &f_st);
      }
      if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &h_emlrtBCI,
                                      &f_st);
      }
      parent = d_obj->Bodies[(int32_T)bid - 1];
    } else {
      parent = &d_obj->Base;
    }
    g_st.site = &xb_emlrtRSI;
    i = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(&g_st, b_basename, i, &yc_emlrtRTEI);
    basename_data = b_basename->data;
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      basename_data[i] = parent->NameInternal->data[i];
    }
    g_st.site = &xb_emlrtRSI;
    RigidBodyTree_addBody(&g_st, newRobotInternal, body, b_basename,
                          &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[5],
                          &(&(&(&(&(&b_iobj_0[0])[0])[0])[0])[0])[4],
                          &(&(&(&(&(&iobj_1[0])[0])[0])[0])[0])[2]);
  }
  if (d_obj->NumBodies >= 4.0) {
    body = d_obj->Bodies[3];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &c_emlrtDCI, &f_st);
      }
      if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &h_emlrtBCI,
                                      &f_st);
      }
      parent = d_obj->Bodies[(int32_T)bid - 1];
    } else {
      parent = &d_obj->Base;
    }
    g_st.site = &xb_emlrtRSI;
    i = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(&g_st, b_basename, i, &yc_emlrtRTEI);
    basename_data = b_basename->data;
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      basename_data[i] = parent->NameInternal->data[i];
    }
    g_st.site = &xb_emlrtRSI;
    RigidBodyTree_addBody(&g_st, newRobotInternal, body, b_basename,
                          &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[7],
                          &(&(&(&(&(&b_iobj_0[0])[0])[0])[0])[0])[6],
                          &(&(&(&(&(&iobj_1[0])[0])[0])[0])[0])[3]);
  }
  if (d_obj->NumBodies >= 5.0) {
    body = d_obj->Bodies[4];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &c_emlrtDCI, &f_st);
      }
      if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &h_emlrtBCI,
                                      &f_st);
      }
      parent = d_obj->Bodies[(int32_T)bid - 1];
    } else {
      parent = &d_obj->Base;
    }
    g_st.site = &xb_emlrtRSI;
    i = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(&g_st, b_basename, i, &yc_emlrtRTEI);
    basename_data = b_basename->data;
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      basename_data[i] = parent->NameInternal->data[i];
    }
    g_st.site = &xb_emlrtRSI;
    RigidBodyTree_addBody(&g_st, newRobotInternal, body, b_basename,
                          &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[9],
                          &(&(&(&(&(&b_iobj_0[0])[0])[0])[0])[0])[8],
                          &(&(&(&(&(&iobj_1[0])[0])[0])[0])[0])[4]);
  }
  if (d_obj->NumBodies >= 6.0) {
    body = d_obj->Bodies[5];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &c_emlrtDCI, &f_st);
      }
      if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &h_emlrtBCI,
                                      &f_st);
      }
      parent = d_obj->Bodies[(int32_T)bid - 1];
    } else {
      parent = &d_obj->Base;
    }
    g_st.site = &xb_emlrtRSI;
    i = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(&g_st, b_basename, i, &yc_emlrtRTEI);
    basename_data = b_basename->data;
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      basename_data[i] = parent->NameInternal->data[i];
    }
    g_st.site = &xb_emlrtRSI;
    RigidBodyTree_addBody(&g_st, newRobotInternal, body, b_basename,
                          &(&(&(&(&(&iobj_2[0])[0])[0])[0])[0])[11],
                          &(&(&(&(&(&b_iobj_0[0])[0])[0])[0])[0])[10],
                          &(&(&(&(&(&iobj_1[0])[0])[0])[0])[0])[5]);
  }
  emxFree_char_T(&f_st, &b_basename);
  f_st.site = &sb_emlrtRSI;
  g_st.site = &ab_emlrtRSI;
  h_st.site = &g_emlrtRSI;
  g_st.site = &oe_emlrtRSI;
  b_obj->_pobj5.TreeInternal =
      b_RigidBodyTree_RigidBodyTree(&g_st, &b_obj->_pobj5._pobj1);
  g_st.site = &pe_emlrtRSI;
  b_obj->_pobj5.TreeInternal->Base.CollisionsInternal =
      CollisionSet_CollisionSet(&g_st, &b_obj->_pobj5._pobj0, 10.0);
  b_obj->_pobj5.matlabCodegenIsDeleted = false;
  b_obj->_pobj5.TreeInternal = newRobotInternal;
  c_obj->RigidBodyTreeInternal = b_obj->_pobj5.TreeInternal;
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

void inverseKinematics_stepImpl(
    const emlrtStack *sp, inverseKinematics *obj, const real_T tform[16],
    const emxArray_struct_T *initialGuess, emxArray_struct_T *QSol,
    real_T *solutionInfo_Iterations, real_T *solutionInfo_NumRandomRestarts,
    real_T *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag,
    char_T solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  b_emxArray_struct_T *Q;
  b_struct_T *Q_data;
  c_emxArray_struct_T *r3;
  c_robotics_manip_internal_Rigid *body;
  d_struct_T *r4;
  e_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  emxArray_char_T *endEffectorName;
  emxArray_int32_T *r;
  emxArray_int32_T *r2;
  emxArray_real_T *bodyIndices;
  emxArray_real_T *iniGuessVec;
  emxArray_real_T *positionIndices;
  emxArray_real_T *positionMap;
  emxArray_real_T *qvSolRaw;
  emxArray_real_T *y;
  rigidBodyJoint *c_obj;
  struct_T *QSol_data;
  real_T bid;
  real_T dofMap_idx_1;
  real_T idxCount;
  real_T *bodyIndices_data;
  real_T *iniGuessVec_data;
  real_T *positionMap_data;
  real_T *qvSolRaw_data;
  real_T *y_data;
  int32_T b_iv[2];
  int32_T c_i;
  int32_T end;
  int32_T i;
  int32_T k;
  int32_T nm1d2;
  int32_T *r1;
  uint32_T b_i;
  char_T *endEffectorName_data;
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &nf_emlrtRSI;
  inverseKinematics_setPoseGoal(&st, obj, tform);
  st.site = &of_emlrtRSI;
  emxInit_real_T(&st, &iniGuessVec, 1, &af_emlrtRTEI);
  b_st.site = &uf_emlrtRSI;
  c_RigidBodyTree_validateConfigu(&b_st, obj->RigidBodyTreeInternal,
                                  initialGuess, iniGuessVec);
  iniGuessVec_data = iniGuessVec->data;
  emxInit_real_T(&st, &qvSolRaw, 1, &bf_emlrtRTEI);
  b_st.site = &vf_emlrtRSI;
  NLPSolverInterface_solve(&b_st, obj->Solver, iniGuessVec, qvSolRaw,
                           solutionInfo_Iterations,
                           solutionInfo_NumRandomRestarts,
                           solutionInfo_PoseErrorNorm, solutionInfo_ExitFlag,
                           solutionInfo_Status_data, solutionInfo_Status_size);
  qvSolRaw_data = qvSolRaw->data;
  b_st.site = &wf_emlrtRSI;
  b_obj = obj->RigidBodyTreeInternal;
  emxInit_char_T(&b_st, &endEffectorName, &ne_emlrtRTEI);
  i = endEffectorName->size[0] * endEffectorName->size[1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  emxEnsureCapacity_char_T(&b_st, endEffectorName, i, &ne_emlrtRTEI);
  endEffectorName_data = endEffectorName->data;
  nm1d2 = obj->Solver->ExtraArgs->BodyName->size[1];
  for (i = 0; i < nm1d2; i++) {
    endEffectorName_data[i] = obj->Solver->ExtraArgs->BodyName->data[i];
  }
  emxInit_real_T(&b_st, &bodyIndices, 1, &oe_emlrtRTEI);
  if (!(b_obj->NumBodies >= 0.0)) {
    emlrtNonNegativeCheckR2012b(b_obj->NumBodies, &pb_emlrtDCI, &b_st);
  }
  bid = b_obj->NumBodies;
  if (bid != (int32_T)muDoubleScalarFloor(bid)) {
    emlrtIntegerCheckR2012b(bid, &ob_emlrtDCI, &b_st);
  }
  i = bodyIndices->size[0];
  bodyIndices->size[0] = (int32_T)bid;
  emxEnsureCapacity_real_T(&b_st, bodyIndices, i, &oe_emlrtRTEI);
  bodyIndices_data = bodyIndices->data;
  if (!(b_obj->NumBodies >= 0.0)) {
    emlrtNonNegativeCheckR2012b(b_obj->NumBodies, &pb_emlrtDCI, &b_st);
  }
  bid = b_obj->NumBodies;
  if (bid != (int32_T)muDoubleScalarFloor(bid)) {
    emlrtIntegerCheckR2012b(bid, &ob_emlrtDCI, &b_st);
  }
  nm1d2 = (int32_T)bid;
  for (i = 0; i < nm1d2; i++) {
    bodyIndices_data[i] = 0.0;
  }
  c_st.site = &xp_emlrtRSI;
  d_st.site = &nd_emlrtRSI;
  bid = c_RigidBodyTree_findBodyIndexBy(&d_st, b_obj, endEffectorName);
  if (bid == -1.0) {
    d_st.site = &od_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &d_st, &e_emlrtRTEI, "robotics:robotmanip:rigidbodytree:BodyNotFound",
        "robotics:robotmanip:rigidbodytree:BodyNotFound", 3, 4,
        endEffectorName->size[1], &endEffectorName_data[0]);
  }
  if (bid == 0.0) {
    i = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    emxEnsureCapacity_real_T(&b_st, bodyIndices, i, &oe_emlrtRTEI);
    bodyIndices_data = bodyIndices->data;
    bodyIndices_data[0] = 0.0;
  } else {
    if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &vb_emlrtBCI,
                                    &b_st);
    }
    body = b_obj->Bodies[(int32_T)bid - 1];
    b_i = 1U;
    while (body->ParentIndex != 0.0) {
      if (((int32_T)b_i < 1) || ((int32_T)b_i > bodyIndices->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, bodyIndices->size[0],
                                      &wb_emlrtBCI, &b_st);
      }
      bodyIndices_data[(int32_T)b_i - 1] = body->Index;
      if (body->ParentIndex !=
          (int32_T)muDoubleScalarFloor(body->ParentIndex)) {
        emlrtIntegerCheckR2012b(body->ParentIndex, &qb_emlrtDCI, &b_st);
      }
      i = (int32_T)body->ParentIndex - 1;
      if ((i < 0) || (i > 5)) {
        emlrtDynamicBoundsCheckR2012b(i, 0, 5, &xb_emlrtBCI, &b_st);
      }
      body = b_obj->Bodies[i];
      b_i++;
    }
    if ((int32_T)(b_i - 1U) < 1) {
      i = 0;
    } else {
      if (bodyIndices->size[0] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, bodyIndices->size[0], &sb_emlrtBCI,
                                      &b_st);
      }
      if (((int32_T)(b_i - 1U) < 1) ||
          ((int32_T)(b_i - 1U) > bodyIndices->size[0])) {
        emlrtDynamicBoundsCheckR2012b(
            (int32_T)(b_i - 1U), 1, bodyIndices->size[0], &rb_emlrtBCI, &b_st);
      }
      i = (int32_T)(b_i - 1U);
    }
    b_iv[0] = 1;
    b_iv[1] = i;
    c_st.site = &yp_emlrtRSI;
    indexShapeCheck(&c_st, bodyIndices->size[0], b_iv);
    k = bodyIndices->size[0];
    bodyIndices->size[0] = i + 2;
    emxEnsureCapacity_real_T(&b_st, bodyIndices, k, &oe_emlrtRTEI);
    bodyIndices_data = bodyIndices->data;
    bodyIndices_data[i] = body->Index;
    bodyIndices_data[i + 1] = 0.0;
  }
  b_st.site = &xf_emlrtRSI;
  b_obj = obj->RigidBodyTreeInternal;
  end = bodyIndices->size[0] - 1;
  nm1d2 = 0;
  for (c_i = 0; c_i <= end; c_i++) {
    if (bodyIndices_data[c_i] != 0.0) {
      nm1d2++;
    }
  }
  emxInit_int32_T(&b_st, &r, 1, &cf_emlrtRTEI);
  i = r->size[0];
  r->size[0] = nm1d2;
  emxEnsureCapacity_int32_T(&b_st, r, i, &pe_emlrtRTEI);
  r1 = r->data;
  nm1d2 = 0;
  for (c_i = 0; c_i <= end; c_i++) {
    if (bodyIndices_data[c_i] != 0.0) {
      r1[nm1d2] = c_i + 1;
      nm1d2++;
    }
  }
  emxInit_real_T(&b_st, &positionMap, 2, &qe_emlrtRTEI);
  i = positionMap->size[0] * positionMap->size[1];
  positionMap->size[0] = r->size[0];
  positionMap->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, positionMap, i, &qe_emlrtRTEI);
  positionMap_data = positionMap->data;
  nm1d2 = r->size[0];
  for (i = 0; i < 2; i++) {
    for (k = 0; k < nm1d2; k++) {
      if (r1[k] > bodyIndices->size[0]) {
        emlrtDynamicBoundsCheckR2012b(r1[k], 1, bodyIndices->size[0],
                                      &ac_emlrtBCI, &b_st);
      }
      bid = bodyIndices_data[r1[k] - 1];
      if (bid != (int32_T)muDoubleScalarFloor(bid)) {
        emlrtIntegerCheckR2012b(bid, &rb_emlrtDCI, &b_st);
      }
      if (((int32_T)bid < 1) || ((int32_T)bid > 6)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid, 1, 6, &yb_emlrtBCI, &b_st);
      }
      positionMap_data[k + positionMap->size[0] * i] =
          b_obj->PositionDoFMap[((int32_T)bid + 6 * i) - 1];
    }
  }
  emxFree_int32_T(&b_st, &r);
  emxFree_real_T(&b_st, &bodyIndices);
  emxInit_real_T(&b_st, &positionIndices, 2, &re_emlrtRTEI);
  i = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[0] = 1;
  emxEnsureCapacity_real_T(&b_st, positionIndices, i, &re_emlrtRTEI);
  if (!(b_obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(b_obj->PositionNumber, &nb_emlrtDCI, &b_st);
  }
  bid = b_obj->PositionNumber;
  if (bid != (int32_T)muDoubleScalarFloor(bid)) {
    emlrtIntegerCheckR2012b(bid, &mb_emlrtDCI, &b_st);
  }
  i = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[1] = (int32_T)bid;
  emxEnsureCapacity_real_T(&b_st, positionIndices, i, &re_emlrtRTEI);
  bodyIndices_data = positionIndices->data;
  if (!(b_obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(b_obj->PositionNumber, &tb_emlrtDCI, &b_st);
  }
  bid = b_obj->PositionNumber;
  if (bid != (int32_T)muDoubleScalarFloor(bid)) {
    emlrtIntegerCheckR2012b(bid, &sb_emlrtDCI, &b_st);
  }
  nm1d2 = (int32_T)bid;
  for (i = 0; i < nm1d2; i++) {
    bodyIndices_data[i] = 0.0;
  }
  idxCount = 0.0;
  i = positionMap->size[0];
  emxInit_int32_T(&b_st, &r2, 2, &bf_emlrtRTEI);
  emxInit_real_T(&b_st, &y, 2, &df_emlrtRTEI);
  y_data = y->data;
  for (c_i = 0; c_i < i; c_i++) {
    real_T numPositions_tmp;
    if (c_i + 1 > positionMap->size[0]) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, positionMap->size[0],
                                    &qb_emlrtBCI, &b_st);
    }
    numPositions_tmp =
        positionMap_data[c_i + positionMap->size[0]] - positionMap_data[c_i];
    if (numPositions_tmp + 1.0 > 0.0) {
      if (numPositions_tmp + 1.0 < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else {
        k = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = (int32_T)((numPositions_tmp + 1.0) - 1.0) + 1;
        emxEnsureCapacity_real_T(&b_st, y, k, &te_emlrtRTEI);
        y_data = y->data;
        nm1d2 = (int32_T)((numPositions_tmp + 1.0) - 1.0);
        for (k = 0; k <= nm1d2; k++) {
          y_data[k] = (real_T)k + 1.0;
        }
      }
      k = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = y->size[1];
      emxEnsureCapacity_int32_T(&b_st, r2, k, &ue_emlrtRTEI);
      r1 = r2->data;
      nm1d2 = y->size[1];
      for (k = 0; k < nm1d2; k++) {
        bid = idxCount + y_data[k];
        if (bid != (int32_T)muDoubleScalarFloor(bid)) {
          emlrtIntegerCheckR2012b(bid, &vb_emlrtDCI, &b_st);
        }
        if (((int32_T)bid < 1) || ((int32_T)bid > positionIndices->size[1])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)bid, 1, positionIndices->size[1], &cc_emlrtBCI, &b_st);
        }
        r1[k] = (int32_T)bid;
      }
      c_st.site = &aq_emlrtRSI;
      bid = positionMap_data[c_i];
      dofMap_idx_1 = positionMap_data[c_i + positionMap->size[0]];
      d_st.site = &rm_emlrtRSI;
      if (muDoubleScalarIsNaN(bid) || muDoubleScalarIsNaN(dofMap_idx_1)) {
        k = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        emxEnsureCapacity_real_T(&d_st, y, k, &ve_emlrtRTEI);
        y_data = y->data;
        y_data[0] = rtNaN;
      } else if (positionMap_data[c_i + positionMap->size[0]] <
                 positionMap_data[c_i]) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else if ((muDoubleScalarIsInf(bid) ||
                  muDoubleScalarIsInf(dofMap_idx_1)) &&
                 (positionMap_data[c_i] ==
                  positionMap_data[c_i + positionMap->size[0]])) {
        k = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        emxEnsureCapacity_real_T(&d_st, y, k, &ve_emlrtRTEI);
        y_data = y->data;
        y_data[0] = rtNaN;
      } else if (muDoubleScalarFloor(bid) == positionMap_data[c_i]) {
        bid = positionMap_data[c_i];
        k = y->size[0] * y->size[1];
        y->size[0] = 1;
        nm1d2 = (int32_T)(positionMap_data[c_i + positionMap->size[0]] - bid);
        y->size[1] = nm1d2 + 1;
        emxEnsureCapacity_real_T(&d_st, y, k, &ve_emlrtRTEI);
        y_data = y->data;
        for (k = 0; k <= nm1d2; k++) {
          y_data[k] = bid + (real_T)k;
        }
      } else {
        real_T apnd;
        real_T cdiff;
        real_T ndbl;
        e_st.site = &bq_emlrtRSI;
        ndbl = muDoubleScalarFloor(numPositions_tmp + 0.5);
        apnd = positionMap_data[c_i] + ndbl;
        cdiff = apnd - positionMap_data[c_i + positionMap->size[0]];
        if (muDoubleScalarAbs(cdiff) <
            4.4408920985006262E-16 *
                muDoubleScalarMax(muDoubleScalarAbs(bid),
                                  muDoubleScalarAbs(dofMap_idx_1))) {
          ndbl++;
          apnd = positionMap_data[c_i + positionMap->size[0]];
        } else if (cdiff > 0.0) {
          apnd = positionMap_data[c_i] + (ndbl - 1.0);
        } else {
          ndbl++;
        }
        if (ndbl >= 0.0) {
          end = (int32_T)ndbl;
        } else {
          end = 0;
        }
        f_st.site = &cq_emlrtRSI;
        if (ndbl > 2.147483647E+9) {
          emlrtErrorWithMessageIdR2018a(&f_st, &s_emlrtRTEI,
                                        "Coder:MATLAB:pmaxsize",
                                        "Coder:MATLAB:pmaxsize", 0);
        }
        k = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = end;
        emxEnsureCapacity_real_T(&e_st, y, k, &xe_emlrtRTEI);
        y_data = y->data;
        if (end > 0) {
          y_data[0] = positionMap_data[c_i];
          if (end > 1) {
            y_data[end - 1] = apnd;
            nm1d2 = (end - 1) / 2;
            for (k = 0; k <= nm1d2 - 2; k++) {
              y_data[k + 1] = bid + ((real_T)k + 1.0);
              y_data[(end - k) - 2] = apnd - ((real_T)k + 1.0);
            }
            if (nm1d2 << 1 == end - 1) {
              y_data[nm1d2] = (positionMap_data[c_i] + apnd) / 2.0;
            } else {
              y_data[nm1d2] = positionMap_data[c_i] + (real_T)nm1d2;
              y_data[nm1d2 + 1] = apnd - (real_T)nm1d2;
            }
          }
        }
      }
      if (r2->size[1] != y->size[1]) {
        emlrtSubAssignSizeCheck1dR2017a(r2->size[1], y->size[1], &i_emlrtECI,
                                        &b_st);
      }
      nm1d2 = y->size[1];
      for (k = 0; k < nm1d2; k++) {
        bodyIndices_data[r1[k] - 1] = y_data[k];
      }
      idxCount += numPositions_tmp + 1.0;
    }
  }
  emxFree_real_T(&b_st, &y);
  emxFree_real_T(&b_st, &positionMap);
  if (idxCount < 1.0) {
    nm1d2 = 0;
  } else {
    if (positionIndices->size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, positionIndices->size[1],
                                    &pb_emlrtBCI, &b_st);
    }
    if (idxCount != (int32_T)muDoubleScalarFloor(idxCount)) {
      emlrtIntegerCheckR2012b(idxCount, &lb_emlrtDCI, &b_st);
    }
    if (((int32_T)idxCount < 1) ||
        ((int32_T)idxCount > positionIndices->size[1])) {
      emlrtDynamicBoundsCheckR2012b(
          (int32_T)idxCount, 1, positionIndices->size[1], &ob_emlrtBCI, &b_st);
    }
    nm1d2 = (int32_T)idxCount;
  }
  i = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[1] = nm1d2;
  emxEnsureCapacity_real_T(&b_st, positionIndices, i, &re_emlrtRTEI);
  bodyIndices_data = positionIndices->data;
  i = r2->size[0] * r2->size[1];
  r2->size[0] = 1;
  r2->size[1] = nm1d2;
  emxEnsureCapacity_int32_T(&st, r2, i, &se_emlrtRTEI);
  r1 = r2->data;
  for (i = 0; i < nm1d2; i++) {
    if (bodyIndices_data[i] !=
        (int32_T)muDoubleScalarFloor(bodyIndices_data[i])) {
      emlrtIntegerCheckR2012b(bodyIndices_data[i], &ub_emlrtDCI, &st);
    }
    k = (int32_T)bodyIndices_data[i];
    if ((k < 1) || (k > iniGuessVec->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)bodyIndices_data[i], 1,
                                    iniGuessVec->size[0], &bc_emlrtBCI, &st);
    }
    r1[i] = k;
  }
  for (i = 0; i < nm1d2; i++) {
    k = (int32_T)bodyIndices_data[i];
    if ((k < 1) || (k > qvSolRaw->size[0])) {
      emlrtDynamicBoundsCheckR2012b(k, 1, qvSolRaw->size[0], &dc_emlrtBCI, &st);
    }
  }
  if (r2->size[1] != nm1d2) {
    emlrtSubAssignSizeCheck1dR2017a(r2->size[1], nm1d2, &j_emlrtECI, &st);
  }
  for (i = 0; i < nm1d2; i++) {
    iniGuessVec_data[r1[i] - 1] =
        qvSolRaw_data[(int32_T)bodyIndices_data[i] - 1];
  }
  emxFree_int32_T(&st, &r2);
  emxFree_real_T(&st, &qvSolRaw);
  emxFree_real_T(&st, &positionIndices);
  emxInit_struct_T1(&st, &Q, &ef_emlrtRTEI);
  b_st.site = &yf_emlrtRSI;
  b_obj = obj->RigidBodyTreeInternal;
  emxInit_struct_T2(&b_st, &r3, &ff_emlrtRTEI);
  c_st.site = &eq_emlrtRSI;
  repmat(&c_st, b_obj->NumNonFixedBodies, r3);
  r4 = r3->data;
  i = Q->size[0] * Q->size[1];
  Q->size[0] = 1;
  Q->size[1] = r3->size[1];
  emxEnsureCapacity_struct_T(&b_st, Q, i, &xb_emlrtRTEI);
  Q_data = Q->data;
  i = r3->size[1] - 1;
  for (k = 0; k <= i; k++) {
    Q_data[k].JointName.size[0] = 1;
    Q_data[k].JointName.size[1] = 0;
    Q_data[k].JointPosition.size[0] = 1;
    Q_data[k].JointPosition.size[1] = 1;
    Q_data[k].JointPosition.data[0] = r4[k].JointPosition;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  }
  emxFree_struct_T2(&b_st, &r3);
  b_i = 1U;
  bid = b_obj->NumBodies;
  i = (int32_T)bid;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, bid, mxDOUBLE_CLASS, (int32_T)bid,
                                &t_emlrtRTEI, &b_st);
  for (c_i = 0; c_i < i; c_i++) {
    boolean_T b_bool;
    if (c_i > 5) {
      emlrtDynamicBoundsCheckR2012b(c_i, 0, 5, &ec_emlrtBCI, &b_st);
    }
    body = b_obj->Bodies[c_i];
    k = endEffectorName->size[0] * endEffectorName->size[1];
    endEffectorName->size[0] = 1;
    endEffectorName->size[1] = body->JointInternal->Type->size[1];
    emxEnsureCapacity_char_T(&b_st, endEffectorName, k, &we_emlrtRTEI);
    endEffectorName_data = endEffectorName->data;
    nm1d2 = body->JointInternal->Type->size[1];
    for (k = 0; k < nm1d2; k++) {
      endEffectorName_data[k] = body->JointInternal->Type->data[k];
    }
    b_bool = false;
    if (endEffectorName->size[1] == 5) {
      nm1d2 = 0;
      int32_T exitg1;
      do {
        exitg1 = 0;
        if (nm1d2 < 5) {
          if (endEffectorName_data[nm1d2] != b_cv[nm1d2]) {
            exitg1 = 1;
          } else {
            nm1d2++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (!b_bool) {
      if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
        emlrtIntegerCheckR2012b(body->Index, &kb_emlrtDCI, &b_st);
      }
      nm1d2 = (int32_T)body->Index;
      if ((nm1d2 < 1) || (nm1d2 > 6)) {
        emlrtDynamicBoundsCheckR2012b(nm1d2, 1, 6, &nb_emlrtBCI, &b_st);
      }
      bid = b_obj->PositionDoFMap[nm1d2 - 1];
      dofMap_idx_1 = b_obj->PositionDoFMap[nm1d2 + 5];
      c_st.site = &fq_emlrtRSI;
      c_obj = body->JointInternal;
      k = endEffectorName->size[0] * endEffectorName->size[1];
      endEffectorName->size[0] = 1;
      endEffectorName->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(&c_st, endEffectorName, k, &ye_emlrtRTEI);
      endEffectorName_data = endEffectorName->data;
      nm1d2 = c_obj->NameInternal->size[1];
      for (k = 0; k < nm1d2; k++) {
        endEffectorName_data[k] = c_obj->NameInternal->data[k];
      }
      if (endEffectorName->size[1] > 200) {
        emlrtDimSizeGeqCheckR2012b(200, endEffectorName->size[1], &h_emlrtECI,
                                   &b_st);
      }
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &tb_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)b_i - 1].JointName.size[0] = 1;
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &tb_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)b_i - 1].JointName.size[1] = endEffectorName->size[1];
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &tb_emlrtBCI,
                                      &b_st);
      }
      nm1d2 = endEffectorName->size[1];
      for (k = 0; k < nm1d2; k++) {
        if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1],
                                        &fc_emlrtBCI, &b_st);
        }
        Q_data[(int32_T)b_i - 1].JointName.data[k] = endEffectorName_data[k];
      }
      if (bid > dofMap_idx_1) {
        k = 0;
        end = 0;
      } else {
        if (bid != (int32_T)muDoubleScalarFloor(bid)) {
          emlrtIntegerCheckR2012b(bid, &jb_emlrtDCI, &b_st);
        }
        if (((int32_T)bid < 1) || ((int32_T)bid > iniGuessVec->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)bid, 1, iniGuessVec->size[0],
                                        &mb_emlrtBCI, &b_st);
        }
        k = (int32_T)bid - 1;
        if (dofMap_idx_1 != (int32_T)muDoubleScalarFloor(dofMap_idx_1)) {
          emlrtIntegerCheckR2012b(dofMap_idx_1, &ib_emlrtDCI, &b_st);
        }
        if (((int32_T)dofMap_idx_1 < 1) ||
            ((int32_T)dofMap_idx_1 > iniGuessVec->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)dofMap_idx_1, 1,
                                        iniGuessVec->size[0], &lb_emlrtBCI,
                                        &b_st);
        }
        end = (int32_T)dofMap_idx_1;
      }
      b_iv[0] = 1;
      nm1d2 = end - k;
      b_iv[1] = nm1d2;
      c_st.site = &gq_emlrtRSI;
      indexShapeCheck(&c_st, iniGuessVec->size[0], b_iv);
      if (nm1d2 != 1) {
        emlrtDimSizeEqCheckR2012b(1, nm1d2, &g_emlrtECI, &b_st);
      }
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &ub_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)b_i - 1].JointPosition.size[0] = 1;
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &ub_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)b_i - 1].JointPosition.size[1] = 1;
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &ub_emlrtBCI,
                                      &b_st);
      }
      if (((int32_T)b_i < 1) || ((int32_T)b_i > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, Q->size[1], &gc_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)b_i - 1].JointPosition.data[0] = iniGuessVec_data[k];
      b_i++;
    }
  }
  emxFree_char_T(&b_st, &endEffectorName);
  emxFree_real_T(&b_st, &iniGuessVec);
  i = QSol->size[0] * QSol->size[1];
  QSol->size[0] = 1;
  QSol->size[1] = Q->size[1];
  emxEnsureCapacity_struct_T1(&st, QSol, i, &yb_emlrtRTEI);
  QSol_data = QSol->data;
  i = Q->size[1] - 1;
  for (k = 0; k <= i; k++) {
    QSol_data[k].JointName.size[0] = 1;
    QSol_data[k].JointName.size[1] = Q_data[k].JointName.size[1];
    nm1d2 = Q_data[k].JointName.size[1];
    for (end = 0; end < nm1d2; end++) {
      QSol_data[k].JointName.data[end] = Q_data[k].JointName.data[end];
    }
    QSol_data[k].JointPosition.size[0] = 1;
    QSol_data[k].JointPosition.size[1] = 1;
    QSol_data[k].JointPosition.data[0] = Q_data[k].JointPosition.data[0];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  emxFree_struct_T1(&st, &Q);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (inverseKinematics.c) */
