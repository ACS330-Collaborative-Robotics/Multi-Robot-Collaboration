/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RigidBodyTree.c
 *
 * Code generation for function 'RigidBodyTree'
 *
 */

/* Include files */
#include "RigidBodyTree.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "all.h"
#include "find.h"
#include "indexShapeCheck.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "mtimes.h"
#include "normalizeRows.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "strcmp.h"
#include "warning.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = {
    111,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    194,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    184,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    187,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    203,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    1283,                                             /* lineNo */
    "RigidBodyTree/defaultInitializeBodiesCellArray", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    158,                   /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

static emlrtRSInfo ac_emlrtRSI = {
    167,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bc_emlrtRSI = {
    163,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo cc_emlrtRSI = {
    160,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    169,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI = {
    47,                                    /* lineNo */
    "VisualizationInfo/VisualizationInfo", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/VisualizationInfo.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI = {
    47,                                                /* lineNo */
    "FastVisualizationHelper/FastVisualizationHelper", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/FastVisualizationHelper.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI = {
    1311,                           /* lineNo */
    "RigidBodyTree/clearAllBodies", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI = {
    1293,                           /* lineNo */
    "RigidBodyTree/clearAllBodies", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI = {
    1330,                                /* lineNo */
    "RigidBodyTree/findBodyIndexByName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo cd_emlrtRSI = {
    232,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo dd_emlrtRSI = {
    236,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ed_emlrtRSI = {
    240,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo fd_emlrtRSI = {
    243,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo gd_emlrtRSI = {
    245,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo hd_emlrtRSI = {
    285,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo id_emlrtRSI = {
    287,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo jd_emlrtRSI = {
    288,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo kd_emlrtRSI = {
    295,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ld_emlrtRSI = {
    296,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo md_emlrtRSI = {
    256,                     /* lineNo */
    "RigidBodyTree/addBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo qd_emlrtRSI = {
    1353,                                     /* lineNo */
    "RigidBodyTree/findBodyIndexByJointName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo rd_emlrtRSI = {
    1357,                                     /* lineNo */
    "RigidBodyTree/findBodyIndexByJointName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo hf_emlrtRSI = {
    2217,                                    /* lineNo */
    "RigidBodyTree/get.JointPositionLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ag_emlrtRSI = {
    1597,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bg_emlrtRSI = {
    1599,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo cg_emlrtRSI = {
    1602,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo dg_emlrtRSI = {
    1608,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo eg_emlrtRSI = {
    1611,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo fg_emlrtRSI = {
    1613,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo gg_emlrtRSI = {
    1538,                                  /* lineNo */
    "RigidBodyTree/validateConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo hg_emlrtRSI = {
    1548,                                  /* lineNo */
    "RigidBodyTree/validateConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ig_emlrtRSI = {
    1552,                                  /* lineNo */
    "RigidBodyTree/validateConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo jg_emlrtRSI = {
    1559,                                  /* lineNo */
    "RigidBodyTree/validateConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo kg_emlrtRSI = {
    1563,                                  /* lineNo */
    "RigidBodyTree/validateConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo yi_emlrtRSI = {
    1870,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo aj_emlrtRSI = {
    1858,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bj_emlrtRSI = {
    1836,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo cj_emlrtRSI = {
    1833,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo dj_emlrtRSI = {
    1800,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ej_emlrtRSI = {
    1787,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo fj_emlrtRSI = {
    1786,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo gj_emlrtRSI = {
    1783,                                        /* lineNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo hj_emlrtRSI = {
    1910,                                  /* lineNo */
    "RigidBodyTree/kinematicPathInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ij_emlrtRSI = {
    1911,                                  /* lineNo */
    "RigidBodyTree/kinematicPathInternal", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtDCInfo d_emlrtDCI = {
    70,             /* lineNo */
    17,             /* colNo */
    "randomString", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/VisualizationInfo.m", /* pName */
    1                                /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    1,              /* iFirst */
    62,             /* iLast */
    70,             /* lineNo */
    17,             /* colNo */
    "",             /* aName */
    "randomString", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/VisualizationInfo.m", /* pName */
    0                                /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    266,                     /* lineNo */
    24,                      /* colNo */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    275,                     /* lineNo */
    36,                      /* colNo */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    1,                       /* iFirst */
    6,                       /* iLast */
    275,                     /* lineNo */
    36,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = {
    287,                     /* lineNo */
    36,                      /* colNo */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    1,                       /* iFirst */
    6,                       /* iLast */
    287,                     /* lineNo */
    36,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = {
    288,                     /* lineNo */
    36,                      /* colNo */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
    1,                       /* iFirst */
    6,                       /* iLast */
    288,                     /* lineNo */
    36,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtDCInfo i_emlrtDCI = {
    290,                     /* lineNo */
    36,                      /* colNo */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = {
    1,                       /* iFirst */
    6,                       /* iLast */
    290,                     /* lineNo */
    36,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = {
    291,                     /* lineNo */
    36,                      /* colNo */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    1,                       /* iFirst */
    6,                       /* iLast */
    291,                     /* lineNo */
    36,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = {
    0,                       /* iFirst */
    5,                       /* iLast */
    266,                     /* lineNo */
    24,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = {
    0,                       /* iFirst */
    5,                       /* iLast */
    271,                     /* lineNo */
    37,                      /* colNo */
    "",                      /* aName */
    "RigidBodyTree/addBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo j_emlrtRTEI = {
    1356,                                     /* lineNo */
    21,                                       /* colNo */
    "RigidBodyTree/findBodyIndexByJointName", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo t_emlrtBCI = {
    0,                                        /* iFirst */
    5,                                        /* iLast */
    1357,                                     /* lineNo */
    38,                                       /* colNo */
    "",                                       /* aName */
    "RigidBodyTree/findBodyIndexByJointName", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo r_emlrtRTEI = {
    2213,                                    /* lineNo */
    21,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtDCInfo cb_emlrtDCI = {
    2217,                                    /* lineNo */
    28,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    2217,                                    /* lineNo */
    28,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo db_emlrtDCI = {
    2217,                                    /* lineNo */
    30,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    2217,                                    /* lineNo */
    30,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo f_emlrtECI = {
    -1,                                      /* nDims */
    2217,                                    /* lineNo */
    21,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtDCInfo eb_emlrtDCI = {
    2211,                                    /* lineNo */
    28,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo fb_emlrtDCI = {
    2211,                                    /* lineNo */
    28,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo gb_emlrtDCI = {
    2211,                                    /* lineNo */
    13,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo hb_emlrtDCI = {
    2211,                                    /* lineNo */
    13,                                      /* colNo */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = {
    0,                                       /* iFirst */
    5,                                       /* iLast */
    2214,                                    /* lineNo */
    35,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTree/get.JointPositionLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo k_emlrtECI = {
    1,                                               /* nDims */
    1600,                                            /* lineNo */
    20,                                              /* colNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtECInfo l_emlrtECI = {
    1,                                               /* nDims */
    1601,                                            /* lineNo */
    20,                                              /* colNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtECInfo m_emlrtECI = {
    -1,                                              /* nDims */
    1609,                                            /* lineNo */
    21,                                              /* colNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtECInfo n_emlrtECI = {
    -1,                                              /* nDims */
    1612,                                            /* lineNo */
    21,                                              /* colNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo u_emlrtRTEI = {
    1545,                                  /* lineNo */
    25,                                    /* colNo */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo hc_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1562,                                  /* lineNo */
    32,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo wb_emlrtDCI = {
    1567,                                  /* lineNo */
    30,                                    /* colNo */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo ic_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1567,                                  /* lineNo */
    30,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo xb_emlrtDCI = {
    1567,                                  /* lineNo */
    37,                                    /* colNo */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo jc_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1567,                                  /* lineNo */
    37,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo o_emlrtECI = {
    -1,                                    /* nDims */
    1567,                                  /* lineNo */
    25,                                    /* colNo */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo v_emlrtRTEI = {
    10,              /* lineNo */
    23,              /* colNo */
    "validatenumel", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatenumel.m" /* pName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    18,              /* lineNo */
    23,              /* colNo */
    "validatenumel", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatenumel.m" /* pName */
};

static emlrtDCInfo yb_emlrtDCI = {
    1597,                                            /* lineNo */
    13,                                              /* colNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo ac_emlrtDCI = {
    1597,                                            /* lineNo */
    13,                                              /* colNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo kc_emlrtBCI = {
    0,                                     /* iFirst */
    5,                                     /* iLast */
    1546,                                  /* lineNo */
    39,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo lc_emlrtBCI = {
    -1,                                              /* iFirst */
    -1,                                              /* iLast */
    1609,                                            /* lineNo */
    26,                                              /* colNo */
    "",                                              /* aName */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo mc_emlrtBCI = {
    -1,                                              /* iFirst */
    -1,                                              /* iLast */
    1609,                                            /* lineNo */
    63,                                              /* colNo */
    "",                                              /* aName */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo nc_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1552,                                  /* lineNo */
    41,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/validateConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo oc_emlrtBCI = {
    -1,                                              /* iFirst */
    -1,                                              /* iLast */
    1612,                                            /* lineNo */
    26,                                              /* colNo */
    "",                                              /* aName */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo pc_emlrtBCI = {
    -1,                                              /* iFirst */
    -1,                                              /* iLast */
    1612,                                            /* lineNo */
    63,                                              /* colNo */
    "",                                              /* aName */
    "RigidBodyTree/validateConfigurationWithLimits", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo sb_emlrtECI = {
    -1,                                          /* nDims */
    1860,                                        /* lineNo */
    29,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo id_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1860,                                        /* lineNo */
    43,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo hc_emlrtDCI = {
    1860,                                        /* lineNo */
    43,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo jd_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1860,                                        /* lineNo */
    35,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo ic_emlrtDCI = {
    1860,                                        /* lineNo */
    35,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo kd_emlrtBCI = {
    1,                                           /* iFirst */
    6,                                           /* iLast */
    1838,                                        /* lineNo */
    55,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo jc_emlrtDCI = {
    1838,                                        /* lineNo */
    55,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo ld_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1836,                                        /* lineNo */
    72,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo kc_emlrtDCI = {
    1836,                                        /* lineNo */
    72,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo md_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1836,                                        /* lineNo */
    64,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo lc_emlrtDCI = {
    1836,                                        /* lineNo */
    64,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo nd_emlrtBCI = {
    1,                                           /* iFirst */
    6,                                           /* iLast */
    1835,                                        /* lineNo */
    51,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo mc_emlrtDCI = {
    1835,                                        /* lineNo */
    51,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo od_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1921,                                  /* lineNo */
    43,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/kinematicPathInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo pd_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1923,                                  /* lineNo */
    34,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/kinematicPathInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo nc_emlrtDCI = {
    1803,                                        /* lineNo */
    32,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo oc_emlrtDCI = {
    1803,                                        /* lineNo */
    32,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo qd_emlrtBCI = {
    0,                                           /* iFirst */
    5,                                           /* iLast */
    1793,                                        /* lineNo */
    40,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo rd_emlrtBCI = {
    0,                                           /* iFirst */
    5,                                           /* iLast */
    1798,                                        /* lineNo */
    40,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo sd_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1915,                                  /* lineNo */
    37,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/kinematicPathInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo td_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1915,                                  /* lineNo */
    64,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/kinematicPathInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ud_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1920,                                  /* lineNo */
    52,                                    /* colNo */
    "",                                    /* aName */
    "RigidBodyTree/kinematicPathInternal", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo pc_emlrtDCI = {
    1803,                                        /* lineNo */
    17,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo qc_emlrtDCI = {
    1803,                                        /* lineNo */
    17,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo vd_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1805,                                        /* lineNo */
    45,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo wd_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1810,                                        /* lineNo */
    45,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo xd_emlrtBCI = {
    0,                                           /* iFirst */
    5,                                           /* iLast */
    1806,                                        /* lineNo */
    50,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo rc_emlrtDCI = {
    1806,                                        /* lineNo */
    50,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo yd_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1806,                                        /* lineNo */
    71,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ae_emlrtBCI = {
    0,                                           /* iFirst */
    5,                                           /* iLast */
    1811,                                        /* lineNo */
    47,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo sc_emlrtDCI = {
    1811,                                        /* lineNo */
    47,                                          /* colNo */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo be_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1811,                                        /* lineNo */
    68,                                          /* colNo */
    "",                                          /* aName */
    "RigidBodyTree/efficientFKAndJacobianForIK", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ce_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    1944,                            /* lineNo */
    33,                              /* colNo */
    "",                              /* aName */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo de_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    1944,                            /* lineNo */
    31,                              /* colNo */
    "",                              /* aName */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ee_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    1934,                            /* lineNo */
    21,                              /* colNo */
    "",                              /* aName */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo tc_emlrtDCI = {
    1932,                            /* lineNo */
    31,                              /* colNo */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo uc_emlrtDCI = {
    1932,                            /* lineNo */
    31,                              /* colNo */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo vc_emlrtDCI = {
    1932,                            /* lineNo */
    13,                              /* colNo */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo wc_emlrtDCI = {
    1932,                            /* lineNo */
    13,                              /* colNo */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = {
    0,                               /* iFirst */
    5,                               /* iLast */
    1936,                            /* lineNo */
    35,                              /* colNo */
    "",                              /* aName */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo xc_emlrtDCI = {
    1936,                            /* lineNo */
    35,                              /* colNo */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo ge_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    1941,                            /* lineNo */
    25,                              /* colNo */
    "",                              /* aName */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo he_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    1937,                            /* lineNo */
    25,                              /* colNo */
    "",                              /* aName */
    "RigidBodyTree/ancestorIndices", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo sc_emlrtRTEI = {
    157,         /* lineNo */
    21,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = {
    95,          /* lineNo */
    17,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

static emlrtRTEInfo bd_emlrtRTEI = {
    1333,            /* lineNo */
    23,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo dd_emlrtRTEI = {
    285,             /* lineNo */
    24,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ed_emlrtRTEI = {
    232,             /* lineNo */
    43,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo fd_emlrtRTEI = {
    1357,            /* lineNo */
    27,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo je_emlrtRTEI = {
    2211,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ke_emlrtRTEI = {
    2215,            /* lineNo */
    28,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo le_emlrtRTEI = {
    2217,            /* lineNo */
    45,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo me_emlrtRTEI = {
    2209,            /* lineNo */
    27,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo if_emlrtRTEI = {
    1597,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo jf_emlrtRTEI = {
    1547,            /* lineNo */
    32,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo kf_emlrtRTEI = {
    1600,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo lf_emlrtRTEI = {
    1601,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo mf_emlrtRTEI = {
    1608,            /* lineNo */
    21,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo nf_emlrtRTEI = {
    1609,            /* lineNo */
    26,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo of_emlrtRTEI = {
    1611,            /* lineNo */
    21,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo pf_emlrtRTEI = {
    1612,            /* lineNo */
    26,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo qf_emlrtRTEI = {
    1599,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ri_emlrtRTEI = {
    1783,            /* lineNo */
    17,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo si_emlrtRTEI = {
    1800,            /* lineNo */
    17,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ti_emlrtRTEI = {
    1803,            /* lineNo */
    17,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ui_emlrtRTEI = {
    1870,            /* lineNo */
    25,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo vi_emlrtRTEI = {
    1832,            /* lineNo */
    31,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo
    wi_emlrtRTEI =
        {
            391,              /* lineNo */
            20,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo xi_emlrtRTEI = {
    1836,            /* lineNo */
    60,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo yi_emlrtRTEI = {
    1859,            /* lineNo */
    34,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo aj_emlrtRTEI = {
    1858,            /* lineNo */
    29,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo bj_emlrtRTEI = {
    1777,            /* lineNo */
    77,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo cj_emlrtRTEI = {
    1910,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo dj_emlrtRTEI = {
    1911,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ej_emlrtRTEI = {
    1932,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo fj_emlrtRTEI = {
    1944,            /* lineNo */
    13,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRSInfo rq_emlrtRSI = {
    1600,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo sq_emlrtRSI = {
    1601,                                            /* lineNo */
    "RigidBodyTree/validateConfigurationWithLimits", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

/* Function Declarations */
static void RigidBodyTree_ancestorIndices(const emlrtStack *sp,
                                          e_robotics_manip_internal_Rigid *obj,
                                          c_robotics_manip_internal_Rigid *body,
                                          emxArray_real_T *indices);

static void b_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real_T *in3);

static void binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                             const emxArray_real_T *in2,
                             const emxArray_real_T *in3);

static int32_T div_s32(const emlrtStack *sp, int32_T numerator,
                       int32_T denominator);

/* Function Definitions */
static void RigidBodyTree_ancestorIndices(const emlrtStack *sp,
                                          e_robotics_manip_internal_Rigid *obj,
                                          c_robotics_manip_internal_Rigid *body,
                                          emxArray_real_T *indices)
{
  real_T d;
  real_T *indices_data;
  int32_T i;
  int32_T loop_ub;
  uint32_T b_i;
  i = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  emxEnsureCapacity_real_T(sp, indices, i, &ej_emlrtRTEI);
  d = obj->NumBodies + 1.0;
  if (!(d >= 0.0)) {
    emlrtNonNegativeCheckR2012b(d, &uc_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (d != (int32_T)muDoubleScalarFloor(d)) {
    emlrtIntegerCheckR2012b(d, &tc_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = indices->size[0] * indices->size[1];
  indices->size[1] = (int32_T)d;
  emxEnsureCapacity_real_T(sp, indices, i, &ej_emlrtRTEI);
  indices_data = indices->data;
  d = obj->NumBodies + 1.0;
  if (!(d >= 0.0)) {
    emlrtNonNegativeCheckR2012b(d, &wc_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (d != (int32_T)muDoubleScalarFloor(d)) {
    emlrtIntegerCheckR2012b(d, &vc_emlrtDCI, (emlrtConstCTX)sp);
  }
  loop_ub = (int32_T)d;
  for (i = 0; i < loop_ub; i++) {
    indices_data[i] = 0.0;
  }
  b_i = 2U;
  if (indices->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, indices->size[1], &ee_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  indices_data[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    if (body->ParentIndex != (int32_T)muDoubleScalarFloor(body->ParentIndex)) {
      emlrtIntegerCheckR2012b(body->ParentIndex, &xc_emlrtDCI,
                              (emlrtConstCTX)sp);
    }
    i = (int32_T)body->ParentIndex - 1;
    if ((i < 0) || (i > 5)) {
      emlrtDynamicBoundsCheckR2012b(i, 0, 5, &fe_emlrtBCI, (emlrtConstCTX)sp);
    }
    body = obj->Bodies[i];
    if (((int32_T)b_i < 1) || ((int32_T)b_i > indices->size[1])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, indices->size[1],
                                    &he_emlrtBCI, (emlrtConstCTX)sp);
    }
    indices_data[(int32_T)b_i - 1] = body->Index;
    b_i++;
  }
  if (body->Index > 0.0) {
    if (((int32_T)b_i < 1) || ((int32_T)b_i > indices->size[1])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, indices->size[1],
                                    &ge_emlrtBCI, (emlrtConstCTX)sp);
    }
    indices_data[(int32_T)b_i - 1] = body->ParentIndex;
    b_i++;
  }
  if (indices->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, indices->size[1], &de_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (((int32_T)(b_i - 1U) < 1) || ((int32_T)(b_i - 1U) > indices->size[1])) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(b_i - 1U), 1, indices->size[1],
                                  &ce_emlrtBCI, (emlrtConstCTX)sp);
  }
  i = indices->size[0] * indices->size[1];
  indices->size[1] = (int32_T)(b_i - 1U);
  emxEnsureCapacity_real_T(sp, indices, i, &fj_emlrtRTEI);
}

static void b_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real_T *in3)
{
  const real_T *in2_data;
  const real_T *in3_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  boolean_T *in1_data;
  in3_data = in3->data;
  in2_data = in2->data;
  i = in1->size[0];
  if (in3->size[0] == 1) {
    in1->size[0] = in2->size[0];
  } else {
    in1->size[0] = in3->size[0];
  }
  emxEnsureCapacity_boolean_T(sp, in1, i, &kf_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] =
        (in2_data[i * stride_0_0] <=
         in3_data[i * stride_1_0 + in3->size[0]] + 4.4408920985006262E-16);
  }
}

static void binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                             const emxArray_real_T *in2,
                             const emxArray_real_T *in3)
{
  const real_T *in2_data;
  const real_T *in3_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  boolean_T *in1_data;
  in3_data = in3->data;
  in2_data = in2->data;
  i = in1->size[0];
  if (in3->size[0] == 1) {
    in1->size[0] = in2->size[0];
  } else {
    in1->size[0] = in3->size[0];
  }
  emxEnsureCapacity_boolean_T(sp, in1, i, &lf_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] >=
                   in3_data[i * stride_1_0] - 4.4408920985006262E-16);
  }
}

static int32_T div_s32(const emlrtStack *sp, int32_T numerator,
                       int32_T denominator)
{
  int32_T quotient;
  if (denominator == 0) {
    emlrtDivisionByZeroErrorR2012b(NULL, (emlrtConstCTX)sp);
  } else {
    uint32_T tempAbsQuotient;
    uint32_T u;
    if (numerator < 0) {
      tempAbsQuotient = ~(uint32_T)numerator + 1U;
    } else {
      tempAbsQuotient = (uint32_T)numerator;
    }
    if (denominator < 0) {
      u = ~(uint32_T)denominator + 1U;
    } else {
      u = (uint32_T)denominator;
    }
    tempAbsQuotient /= u;
    if ((numerator < 0) != (denominator < 0)) {
      quotient = -(int32_T)tempAbsQuotient;
    } else {
      quotient = (int32_T)tempAbsQuotient;
    }
  }
  return quotient;
}

d_robotics_manip_internal_Rigid *
RigidBodyTree_RigidBodyTree(const emlrtStack *sp,
                            d_robotics_manip_internal_Rigid *obj)
{
  static const real_T dv[16] = {1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0,  0.0,
                                0.0, 0.0, 1.0,  0.0, 0.0, 0.0, 0.13, 1.0};
  static const real_T dv1[16] = {1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0,    0.0,
                                 0.0, 0.0, 1.0,  0.0, 0.0, 0.0, 0.0625, 1.0};
  static const char_T cv3[9] = {'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k'};
  static const char_T b[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T c_jname[6] = {'j', 'o', 'i', 'n', 't', '2'};
  static const char_T jname[6] = {'j', 'o', 'i', 'n', 't', '1'};
  static const char_T b_cv[5] = {'l', 'i', 'n', 'k', '1'};
  static const char_T b_cv2[5] = {'l', 'i', 'n', 'k', '2'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  static const int8_T iv2[6] = {0, 0, -1, 0, 0, 0};
  static const int8_T iv3[6] = {0, 1, 0, 0, 0, 0};
  c_robotics_manip_internal_Rigid *d_obj;
  d_robotics_manip_internal_Colli *iobj_0;
  d_robotics_manip_internal_Rigid *b_obj;
  d_robotics_manip_internal_Rigid *c_obj;
  emlrtStack b_st;
  emlrtStack st;
  emxArray_char_T *b_jname;
  rigidBodyJoint *iobj_1;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T jointtype_tmp_data[20];
  char_T *jname_data;
  int8_T msubspace_data[36];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  st.site = &h_emlrtRSI;
  b_obj->NumBodies = 6.0;
  st.site = &j_emlrtRSI;
  c_obj = b_obj;
  iobj_0 = &b_obj->_pobj0[0];
  iobj_1 = &b_obj->_pobj1[0];
  b_st.site = &m_emlrtRSI;
  c_obj->Bodies[0] = RigidBody_RigidBody(&b_st, &(&b_obj->_pobj2[0])[0],
                                         &iobj_0[0], &(&b_obj->_pobj1[0])[0]);
  b_st.site = &m_emlrtRSI;
  c_obj->Bodies[1] = b_RigidBody_RigidBody(&b_st, &(&b_obj->_pobj2[0])[1],
                                           &iobj_0[1], &iobj_1[1]);
  b_st.site = &m_emlrtRSI;
  c_obj->Bodies[2] = c_RigidBody_RigidBody(&b_st, &(&b_obj->_pobj2[0])[2],
                                           &iobj_0[2], &iobj_1[2]);
  b_st.site = &m_emlrtRSI;
  c_obj->Bodies[3] = d_RigidBody_RigidBody(&b_st, &(&b_obj->_pobj2[0])[3],
                                           &iobj_0[3], &iobj_1[3]);
  b_st.site = &m_emlrtRSI;
  c_obj->Bodies[4] = e_RigidBody_RigidBody(&b_st, &(&b_obj->_pobj2[0])[4],
                                           &iobj_0[4], &iobj_1[4]);
  b_st.site = &m_emlrtRSI;
  c_obj->Bodies[5] = f_RigidBody_RigidBody(&b_st, &(&b_obj->_pobj2[0])[5],
                                           &iobj_0[5], &iobj_1[5]);
  st.site = &k_emlrtRSI;
  d_obj = &b_obj->_pobj2[6];
  iobj_0 = &b_obj->_pobj0[6];
  i = d_obj->NameInternal->size[0] * d_obj->NameInternal->size[1];
  d_obj->NameInternal->size[0] = 1;
  d_obj->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(&st, d_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    d_obj->NameInternal->data[i] = b_cv[i];
  }
  d_obj->ParentIndex = 0.0;
  d_obj->MassInternal = 0.0;
  d_obj->CenterOfMassInternal[0] = 0.0;
  d_obj->CenterOfMassInternal[1] = 0.0;
  d_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    d_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    d_obj->SpatialInertia[i] = 0.0;
  }
  b_st.site = &u_emlrtRSI;
  b_obj->_pobj1[6].InTree = false;
  for (i = 0; i < 16; i++) {
    b_obj->_pobj1[6].JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->_pobj1[6].ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->_pobj1[6].NameInternal->size[0] *
      b_obj->_pobj1[6].NameInternal->size[1];
  b_obj->_pobj1[6].NameInternal->size[0] = 1;
  b_obj->_pobj1[6].NameInternal->size[1] = 6;
  emxEnsureCapacity_char_T(&b_st, b_obj->_pobj1[6].NameInternal, i,
                           &jc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->_pobj1[6].NameInternal->data[i] = jname[i];
  }
  for (i = 0; i < 8; i++) {
    jointtype_tmp_data[i] = b[i];
  }
  i = b_obj->_pobj1[6].Type->size[0] * b_obj->_pobj1[6].Type->size[1];
  b_obj->_pobj1[6].Type->size[0] = 1;
  b_obj->_pobj1[6].Type->size[1] = 8;
  emxEnsureCapacity_char_T(&b_st, b_obj->_pobj1[6].Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    b_obj->_pobj1[6].Type->data[i] = jointtype_tmp_data[i];
  }
  emxInit_char_T(&b_st, &b_jname, &tc_emlrtRTEI);
  i = b_jname->size[0] * b_jname->size[1];
  b_jname->size[0] = 1;
  b_jname->size[1] = b_obj->_pobj1[6].Type->size[1];
  emxEnsureCapacity_char_T(&b_st, b_jname, i, &lc_emlrtRTEI);
  jname_data = b_jname->data;
  loop_ub = b_obj->_pobj1[6].Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    jname_data[i] = b_obj->_pobj1[6].Type->data[i];
  }
  b_bool = false;
  if (b_jname->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (jname_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (b_jname->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (jname_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->_pobj1[6].VelocityNumber = 1.0;
    b_obj->_pobj1[6].PositionNumber = 1.0;
    b_obj->_pobj1[6].JointAxisInternal[0] = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[1] = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->_pobj1[6].VelocityNumber = 1.0;
    b_obj->_pobj1[6].PositionNumber = 1.0;
    b_obj->_pobj1[6].JointAxisInternal[0] = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[1] = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->_pobj1[6].VelocityNumber = 0.0;
    b_obj->_pobj1[6].PositionNumber = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[0] = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[1] = 0.0;
    b_obj->_pobj1[6].JointAxisInternal[2] = 0.0;
    break;
  }
  i = b_obj->_pobj1[6].MotionSubspace->size[0] *
      b_obj->_pobj1[6].MotionSubspace->size[1];
  b_obj->_pobj1[6].MotionSubspace->size[0] = 6;
  b_obj->_pobj1[6].MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj1[6].MotionSubspace, i,
                           &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->_pobj1[6].MotionSubspace->data[i] = msubspace_data[i];
  }
  i = b_obj->_pobj1[6].PositionLimitsInternal->size[0] *
      b_obj->_pobj1[6].PositionLimitsInternal->size[1];
  b_obj->_pobj1[6].PositionLimitsInternal->size[0] = 1;
  b_obj->_pobj1[6].PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj1[6].PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    b_obj->_pobj1[6].PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = b_obj->_pobj1[6].HomePositionInternal->size[0];
  b_obj->_pobj1[6].HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj1[6].HomePositionInternal, i,
                           &oc_emlrtRTEI);
  b_obj->_pobj1[6].HomePositionInternal->data[0] = 0.0;
  d_obj->JointInternal = &b_obj->_pobj1[6];
  for (i = 0; i < 16; i++) {
    d_obj->JointInternal->JointToParentTransform[i] = dv[i];
  }
  for (i = 0; i < 16; i++) {
    d_obj->JointInternal->ChildToJointTransform[i] = iv[i];
  }
  i = d_obj->JointInternal->MotionSubspace->size[0] *
      d_obj->JointInternal->MotionSubspace->size[1];
  d_obj->JointInternal->MotionSubspace->size[0] = 6;
  d_obj->JointInternal->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, d_obj->JointInternal->MotionSubspace, i,
                           &pc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    d_obj->JointInternal->MotionSubspace->data[i] = iv2[i];
  }
  d_obj->JointInternal->InTree = true;
  i = d_obj->JointInternal->PositionLimitsInternal->size[0] *
      d_obj->JointInternal->PositionLimitsInternal->size[1];
  d_obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  d_obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, d_obj->JointInternal->PositionLimitsInternal, i,
                           &qc_emlrtRTEI);
  d_obj->JointInternal->PositionLimitsInternal->data[0] = -2.27;
  d_obj->JointInternal->PositionLimitsInternal->data[1] = 2.27;
  d_obj->JointInternal->JointAxisInternal[0] = 0.0;
  d_obj->JointInternal->JointAxisInternal[1] = 0.0;
  d_obj->JointInternal->JointAxisInternal[2] = -1.0;
  i = d_obj->JointInternal->HomePositionInternal->size[0];
  d_obj->JointInternal->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, d_obj->JointInternal->HomePositionInternal, i,
                           &rc_emlrtRTEI);
  d_obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  b_st.site = &v_emlrtRSI;
  d_obj->CollisionsInternal = CollisionSet_CollisionSet(&b_st, iobj_0, 0.0);
  d_obj->matlabCodegenIsDeleted = false;
  b_obj->Bodies[0] = d_obj;
  b_obj->Bodies[0]->Index = 1.0;
  st.site = &k_emlrtRSI;
  d_obj = &b_obj->_pobj2[7];
  iobj_0 = &b_obj->_pobj0[7];
  i = d_obj->NameInternal->size[0] * d_obj->NameInternal->size[1];
  d_obj->NameInternal->size[0] = 1;
  d_obj->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(&st, d_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    d_obj->NameInternal->data[i] = b_cv2[i];
  }
  d_obj->ParentIndex = 1.0;
  d_obj->MassInternal = 0.0;
  d_obj->CenterOfMassInternal[0] = 0.0;
  d_obj->CenterOfMassInternal[1] = 0.0;
  d_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    d_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    d_obj->SpatialInertia[i] = 0.0;
  }
  b_st.site = &u_emlrtRSI;
  b_obj->_pobj1[7].InTree = false;
  for (i = 0; i < 16; i++) {
    b_obj->_pobj1[7].JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->_pobj1[7].ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->_pobj1[7].NameInternal->size[0] *
      b_obj->_pobj1[7].NameInternal->size[1];
  b_obj->_pobj1[7].NameInternal->size[0] = 1;
  b_obj->_pobj1[7].NameInternal->size[1] = 6;
  emxEnsureCapacity_char_T(&b_st, b_obj->_pobj1[7].NameInternal, i,
                           &jc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->_pobj1[7].NameInternal->data[i] = c_jname[i];
  }
  i = b_obj->_pobj1[7].Type->size[0] * b_obj->_pobj1[7].Type->size[1];
  b_obj->_pobj1[7].Type->size[0] = 1;
  b_obj->_pobj1[7].Type->size[1] = 8;
  emxEnsureCapacity_char_T(&b_st, b_obj->_pobj1[7].Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    b_obj->_pobj1[7].Type->data[i] = jointtype_tmp_data[i];
  }
  i = b_jname->size[0] * b_jname->size[1];
  b_jname->size[0] = 1;
  b_jname->size[1] = b_obj->_pobj1[7].Type->size[1];
  emxEnsureCapacity_char_T(&b_st, b_jname, i, &lc_emlrtRTEI);
  jname_data = b_jname->data;
  loop_ub = b_obj->_pobj1[7].Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    jname_data[i] = b_obj->_pobj1[7].Type->data[i];
  }
  b_bool = false;
  if (b_jname->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (jname_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (b_jname->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (jname_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->_pobj1[7].VelocityNumber = 1.0;
    b_obj->_pobj1[7].PositionNumber = 1.0;
    b_obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->_pobj1[7].VelocityNumber = 1.0;
    b_obj->_pobj1[7].PositionNumber = 1.0;
    b_obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->_pobj1[7].VelocityNumber = 0.0;
    b_obj->_pobj1[7].PositionNumber = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    b_obj->_pobj1[7].JointAxisInternal[2] = 0.0;
    break;
  }
  i = b_obj->_pobj1[7].MotionSubspace->size[0] *
      b_obj->_pobj1[7].MotionSubspace->size[1];
  b_obj->_pobj1[7].MotionSubspace->size[0] = 6;
  b_obj->_pobj1[7].MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj1[7].MotionSubspace, i,
                           &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->_pobj1[7].MotionSubspace->data[i] = msubspace_data[i];
  }
  i = b_obj->_pobj1[7].PositionLimitsInternal->size[0] *
      b_obj->_pobj1[7].PositionLimitsInternal->size[1];
  b_obj->_pobj1[7].PositionLimitsInternal->size[0] = 1;
  b_obj->_pobj1[7].PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj1[7].PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    b_obj->_pobj1[7].PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = b_obj->_pobj1[7].HomePositionInternal->size[0];
  b_obj->_pobj1[7].HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj1[7].HomePositionInternal, i,
                           &oc_emlrtRTEI);
  b_obj->_pobj1[7].HomePositionInternal->data[0] = 0.0;
  d_obj->JointInternal = &b_obj->_pobj1[7];
  for (i = 0; i < 16; i++) {
    d_obj->JointInternal->JointToParentTransform[i] = dv1[i];
  }
  for (i = 0; i < 16; i++) {
    d_obj->JointInternal->ChildToJointTransform[i] = iv[i];
  }
  i = d_obj->JointInternal->MotionSubspace->size[0] *
      d_obj->JointInternal->MotionSubspace->size[1];
  d_obj->JointInternal->MotionSubspace->size[0] = 6;
  d_obj->JointInternal->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, d_obj->JointInternal->MotionSubspace, i,
                           &pc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    d_obj->JointInternal->MotionSubspace->data[i] = iv3[i];
  }
  d_obj->JointInternal->InTree = true;
  i = d_obj->JointInternal->PositionLimitsInternal->size[0] *
      d_obj->JointInternal->PositionLimitsInternal->size[1];
  d_obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  d_obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, d_obj->JointInternal->PositionLimitsInternal, i,
                           &qc_emlrtRTEI);
  d_obj->JointInternal->PositionLimitsInternal->data[0] = -0.87;
  d_obj->JointInternal->PositionLimitsInternal->data[1] = 1.05;
  d_obj->JointInternal->JointAxisInternal[0] = 0.0;
  d_obj->JointInternal->JointAxisInternal[1] = 1.0;
  d_obj->JointInternal->JointAxisInternal[2] = 0.0;
  i = d_obj->JointInternal->HomePositionInternal->size[0];
  d_obj->JointInternal->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, d_obj->JointInternal->HomePositionInternal, i,
                           &rc_emlrtRTEI);
  d_obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  b_st.site = &v_emlrtRSI;
  d_obj->CollisionsInternal = CollisionSet_CollisionSet(&b_st, iobj_0, 0.0);
  d_obj->matlabCodegenIsDeleted = false;
  b_obj->Bodies[1] = d_obj;
  b_obj->Bodies[1]->Index = 2.0;
  st.site = &k_emlrtRSI;
  b_obj->Bodies[2] = g_RigidBody_RigidBody(
      &st, &b_obj->_pobj2[8], &b_obj->_pobj0[8], &b_obj->_pobj1[8]);
  b_obj->Bodies[2]->Index = 3.0;
  st.site = &k_emlrtRSI;
  b_obj->Bodies[3] = h_RigidBody_RigidBody(
      &st, &b_obj->_pobj2[9], &b_obj->_pobj0[9], &b_obj->_pobj1[9]);
  b_obj->Bodies[3]->Index = 4.0;
  st.site = &k_emlrtRSI;
  b_obj->Bodies[4] = i_RigidBody_RigidBody(
      &st, &b_obj->_pobj2[10], &b_obj->_pobj0[10], &b_obj->_pobj1[10]);
  b_obj->Bodies[4]->Index = 5.0;
  st.site = &k_emlrtRSI;
  b_obj->Bodies[5] = j_RigidBody_RigidBody(
      &st, &b_obj->_pobj2[11], &b_obj->_pobj0[11], &b_obj->_pobj1[11]);
  b_obj->Bodies[5]->Index = 6.0;
  st.site = &i_emlrtRSI;
  b_obj->Gravity[0] = 0.0;
  b_obj->Gravity[1] = 0.0;
  b_obj->Gravity[2] = 0.0;
  b_obj->NumNonFixedBodies = 6.0;
  st.site = &l_emlrtRSI;
  d_obj = &b_obj->Base;
  iobj_0 = &b_obj->_pobj0[12];
  i = d_obj->NameInternal->size[0] * d_obj->NameInternal->size[1];
  d_obj->NameInternal->size[0] = 1;
  d_obj->NameInternal->size[1] = 9;
  emxEnsureCapacity_char_T(&st, d_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 9; i++) {
    d_obj->NameInternal->data[i] = cv3[i];
  }
  d_obj->ParentIndex = -1.0;
  d_obj->MassInternal = 0.0;
  d_obj->CenterOfMassInternal[0] = 0.0;
  d_obj->CenterOfMassInternal[1] = 0.0;
  d_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    d_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    d_obj->SpatialInertia[i] = 0.0;
  }
  i = b_jname->size[0] * b_jname->size[1];
  b_jname->size[0] = 1;
  b_jname->size[1] = d_obj->NameInternal->size[1] + 4;
  emxEnsureCapacity_char_T(&st, b_jname, i, &sc_emlrtRTEI);
  jname_data = b_jname->data;
  loop_ub = d_obj->NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    jname_data[i] = d_obj->NameInternal->data[i];
  }
  jname_data[d_obj->NameInternal->size[1]] = '_';
  jname_data[d_obj->NameInternal->size[1] + 1] = 'j';
  jname_data[d_obj->NameInternal->size[1] + 2] = 'n';
  jname_data[d_obj->NameInternal->size[1] + 3] = 't';
  b_st.site = &w_emlrtRSI;
  d_obj->JointInternal =
      rigidBodyJoint_rigidBodyJoint(&b_st, &b_obj->_pobj1[12], b_jname);
  emxFree_char_T(&st, &b_jname);
  b_st.site = &v_emlrtRSI;
  d_obj->CollisionsInternal = CollisionSet_CollisionSet(&b_st, iobj_0, 0.0);
  d_obj->matlabCodegenIsDeleted = false;
  b_obj->Base.Index = 0.0;
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

void RigidBodyTree_addBody(const emlrtStack *sp,
                           e_robotics_manip_internal_Rigid *obj,
                           c_robotics_manip_internal_Rigid *bodyin,
                           const emxArray_char_T *parentName,
                           d_robotics_manip_internal_Colli *iobj_0,
                           rigidBodyJoint *iobj_1,
                           c_robotics_manip_internal_Rigid *iobj_2)
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const char_T varargin_1[5] = {'J', 'o', 'i', 'n', 't'};
  c_robotics_manip_internal_Rigid *body;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_char_T *bname;
  emxArray_char_T *nm;
  rigidBodyJoint *jnt;
  real_T bid;
  real_T pid;
  int32_T b_i;
  int32_T bid2;
  int32_T i;
  int32_T loop_ub;
  const char_T *parentName_data;
  char_T *bname_data;
  boolean_T b_bool;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  parentName_data = parentName->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_char_T(sp, &bname, &ed_emlrtRTEI);
  st.site = &cd_emlrtRSI;
  i = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  emxEnsureCapacity_char_T(&st, bname, i, &yc_emlrtRTEI);
  bname_data = bname->data;
  loop_ub = bodyin->NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    bname_data[i] = bodyin->NameInternal->data[i];
  }
  st.site = &cd_emlrtRSI;
  bid = c_RigidBodyTree_findBodyIndexBy(&st, obj, bname);
  if (bid > -1.0) {
    st.site = &dd_emlrtRSI;
    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = bodyin->NameInternal->size[1];
    emxEnsureCapacity_char_T(&st, bname, i, &yc_emlrtRTEI);
    bname_data = bname->data;
    loop_ub = bodyin->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname_data[i] = bodyin->NameInternal->data[i];
    }
    st.site = &dd_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &st, &e_emlrtRTEI,
        "robotics:robotmanip:rigidbodytree:BodyNameCollision",
        "robotics:robotmanip:rigidbodytree:BodyNameCollision", 3, 4,
        bname->size[1], &bname_data[0]);
  }
  st.site = &ed_emlrtRSI;
  b_st.site = &nd_emlrtRSI;
  pid = c_RigidBodyTree_findBodyIndexBy(&b_st, obj, parentName);
  if (pid == -1.0) {
    b_st.site = &od_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI, "robotics:robotmanip:rigidbodytree:BodyNotFound",
        "robotics:robotmanip:rigidbodytree:BodyNotFound", 3, 4,
        parentName->size[1], &parentName_data[0]);
  }
  st.site = &fd_emlrtRSI;
  if (bodyin->Index == 0.0) {
    b_st.site = &pd_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI,
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
        &varargin_1[0]);
  }
  jnt = bodyin->JointInternal;
  st.site = &fd_emlrtRSI;
  i = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(&st, bname, i, &cd_emlrtRTEI);
  bname_data = bname->data;
  loop_ub = jnt->NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    bname_data[i] = jnt->NameInternal->data[i];
  }
  st.site = &fd_emlrtRSI;
  bid2 = -1;
  b_st.site = &qd_emlrtRSI;
  c_st.site = &y_emlrtRSI;
  if (bname->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:findBodyIndexByJointName:expectedNonempty", 3, 4, 9,
        "jointname");
  }
  bid = obj->NumBodies;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, bid, mxDOUBLE_CLASS, (int32_T)bid,
                                &j_emlrtRTEI, &st);
  b_i = 0;
  emxInit_char_T(&st, &nm, &fd_emlrtRTEI);
  exitg1 = false;
  while ((!exitg1) && (b_i <= (int32_T)bid - 1)) {
    b_st.site = &rd_emlrtRSI;
    if (b_i > 5) {
      emlrtDynamicBoundsCheckR2012b(6, 0, 5, &t_emlrtBCI, &b_st);
    }
    body = obj->Bodies[b_i];
    if (body->Index == 0.0) {
      c_st.site = &pd_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &c_st, &e_emlrtRTEI,
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
          &varargin_1[0]);
    }
    jnt = body->JointInternal;
    b_st.site = &rd_emlrtRSI;
    i = nm->size[0] * nm->size[1];
    nm->size[0] = 1;
    nm->size[1] = jnt->NameInternal->size[1];
    emxEnsureCapacity_char_T(&b_st, nm, i, &cd_emlrtRTEI);
    bname_data = nm->data;
    loop_ub = jnt->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname_data[i] = jnt->NameInternal->data[i];
    }
    b_st.site = &rd_emlrtRSI;
    if (b_strcmp(&b_st, nm, bname)) {
      bid2 = b_i + 1;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  emxFree_char_T(&st, &nm);
  if (bid2 > 0) {
    st.site = &gd_emlrtRSI;
    if (bodyin->Index == 0.0) {
      b_st.site = &pd_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &b_st, &e_emlrtRTEI,
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
          &varargin_1[0]);
    }
    jnt = bodyin->JointInternal;
    st.site = &gd_emlrtRSI;
    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = jnt->NameInternal->size[1];
    emxEnsureCapacity_char_T(&st, bname, i, &cd_emlrtRTEI);
    bname_data = bname->data;
    loop_ub = jnt->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname_data[i] = jnt->NameInternal->data[i];
    }
    st.site = &gd_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &st, &e_emlrtRTEI,
        "robotics:robotmanip:rigidbodytree:JointNameCollision",
        "robotics:robotmanip:rigidbodytree:JointNameCollision", 3, 4,
        bname->size[1], &bname_data[0]);
  }
  bid = obj->NumBodies + 1.0;
  st.site = &md_emlrtRSI;
  body = RigidBody_copy(&st, bodyin, &iobj_0[0], &iobj_1[0], iobj_2);
  if (bid != (int32_T)muDoubleScalarFloor(bid)) {
    emlrtIntegerCheckR2012b(bid, &e_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)bid - 1 < 0) || ((int32_T)bid - 1 > 5)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)bid - 1, 0, 5, &r_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  obj->Bodies[(int32_T)bid - 1] = body;
  body->Index = bid;
  body->ParentIndex = pid;
  if ((pid > 0.0) && (((int32_T)pid - 1 < 0) || ((int32_T)pid - 1 > 5))) {
    emlrtDynamicBoundsCheckR2012b((int32_T)pid - 1, 0, 5, &s_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
    emlrtIntegerCheckR2012b(body->Index, &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = (int32_T)body->Index;
  if ((i < 1) || (i > 6)) {
    emlrtDynamicBoundsCheckR2012b(i, 1, 6, &m_emlrtBCI, (emlrtConstCTX)sp);
  }
  body->JointInternal->InTree = true;
  obj->NumBodies++;
  st.site = &hd_emlrtRSI;
  if (body->Index == 0.0) {
    b_st.site = &pd_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI,
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
        &varargin_1[0]);
  }
  jnt = body->JointInternal;
  i = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  emxEnsureCapacity_char_T(sp, bname, i, &dd_emlrtRTEI);
  bname_data = bname->data;
  loop_ub = jnt->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    bname_data[i] = jnt->Type->data[i];
  }
  b_bool = false;
  if (bname->size[1] == 5) {
    loop_ub = 0;
    int32_T exitg2;
    do {
      exitg2 = 0;
      if (loop_ub < 5) {
        if (bname_data[loop_ub] != b_cv[loop_ub]) {
          exitg2 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
  emxFree_char_T(sp, &bname);
  if (!b_bool) {
    obj->NumNonFixedBodies++;
    st.site = &id_emlrtRSI;
    if (body->Index == 0.0) {
      b_st.site = &pd_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &b_st, &e_emlrtRTEI,
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
          &varargin_1[0]);
    }
    jnt = body->JointInternal;
    if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
      emlrtIntegerCheckR2012b(body->Index, &g_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)body->Index < 1) || ((int32_T)body->Index > 6)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)body->Index, 1, 6, &n_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = (int32_T)body->Index - 1;
    obj->PositionDoFMap[loop_ub] = obj->PositionNumber + 1.0;
    obj->PositionDoFMap[loop_ub + 6] =
        obj->PositionNumber + jnt->PositionNumber;
    st.site = &jd_emlrtRSI;
    if (body->Index == 0.0) {
      b_st.site = &pd_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &b_st, &e_emlrtRTEI,
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
          &varargin_1[0]);
    }
    jnt = body->JointInternal;
    if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
      emlrtIntegerCheckR2012b(body->Index, &h_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)body->Index < 1) || ((int32_T)body->Index > 6)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)body->Index, 1, 6, &o_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = (int32_T)body->Index - 1;
    obj->VelocityDoFMap[loop_ub] = obj->VelocityNumber + 1.0;
    obj->VelocityDoFMap[loop_ub + 6] =
        obj->VelocityNumber + jnt->VelocityNumber;
  } else {
    if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
      emlrtIntegerCheckR2012b(body->Index, &i_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)body->Index < 1) || ((int32_T)body->Index > 6)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)body->Index, 1, 6, &p_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = (int32_T)body->Index;
    obj->PositionDoFMap[loop_ub - 1] = 0.0;
    obj->PositionDoFMap[loop_ub + 5] = -1.0;
    if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
      emlrtIntegerCheckR2012b(body->Index, &j_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)body->Index < 1) || ((int32_T)body->Index > 6)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)body->Index, 1, 6, &q_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = (int32_T)body->Index;
    obj->VelocityDoFMap[loop_ub - 1] = 0.0;
    obj->VelocityDoFMap[loop_ub + 5] = -1.0;
  }
  st.site = &kd_emlrtRSI;
  if (body->Index == 0.0) {
    b_st.site = &pd_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI,
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
        &varargin_1[0]);
  }
  jnt = body->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  st.site = &ld_emlrtRSI;
  if (body->Index == 0.0) {
    b_st.site = &pd_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI,
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
        "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
        &varargin_1[0]);
  }
  jnt = body->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

e_robotics_manip_internal_Rigid *
b_RigidBodyTree_RigidBodyTree(const emlrtStack *sp,
                              e_robotics_manip_internal_Rigid *obj)
{
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T jname[8] = {'b', 'a', 's', 'e', '_', 'j', 'n', 't'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T iv2[12] = {0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *c_obj;
  d_robotics_manip_internal_Colli *iobj_0;
  e_robotics_manip_internal_Rigid *b_obj;
  e_robotics_manip_internal_Rigid *d_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  rigidBodyJoint *iobj_1;
  real_T poslim_data[12];
  real_T idx[5];
  real_T d;
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  st.site = &h_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  st.site = &cc_emlrtRSI;
  c_obj = &b_obj->Base;
  iobj_0 = &b_obj->_pobj1[0];
  b_st.site = &n_emlrtRSI;
  i = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
  c_obj->NameInternal->size[0] = 1;
  c_obj->NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(&st, c_obj->NameInternal, i, &uc_emlrtRTEI);
  c_obj->NameInternal->data[0] = 'b';
  c_obj->NameInternal->data[1] = 'a';
  c_obj->NameInternal->data[2] = 's';
  c_obj->NameInternal->data[3] = 'e';
  b_st.site = &o_emlrtRSI;
  b_obj->_pobj2[0].InTree = false;
  for (i = 0; i < 16; i++) {
    b_obj->_pobj2[0].JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->_pobj2[0].ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->_pobj2[0].NameInternal->size[0] *
      b_obj->_pobj2[0].NameInternal->size[1];
  b_obj->_pobj2[0].NameInternal->size[0] = 1;
  b_obj->_pobj2[0].NameInternal->size[1] = 8;
  emxEnsureCapacity_char_T(&b_st, b_obj->_pobj2[0].NameInternal, i,
                           &jc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    b_obj->_pobj2[0].NameInternal->data[i] = jname[i];
  }
  i = b_obj->_pobj2[0].Type->size[0] * b_obj->_pobj2[0].Type->size[1];
  b_obj->_pobj2[0].Type->size[0] = 1;
  b_obj->_pobj2[0].Type->size[1] = 5;
  emxEnsureCapacity_char_T(&b_st, b_obj->_pobj2[0].Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    b_obj->_pobj2[0].Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&b_st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->_pobj2[0].Type->size[1];
  emxEnsureCapacity_char_T(&b_st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = b_obj->_pobj2[0].Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = b_obj->_pobj2[0].Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&b_st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->_pobj2[0].VelocityNumber = 1.0;
    b_obj->_pobj2[0].PositionNumber = 1.0;
    b_obj->_pobj2[0].JointAxisInternal[0] = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[1] = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->_pobj2[0].VelocityNumber = 1.0;
    b_obj->_pobj2[0].PositionNumber = 1.0;
    b_obj->_pobj2[0].JointAxisInternal[0] = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[1] = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->_pobj2[0].VelocityNumber = 0.0;
    b_obj->_pobj2[0].PositionNumber = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[0] = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[1] = 0.0;
    b_obj->_pobj2[0].JointAxisInternal[2] = 0.0;
    break;
  }
  i = b_obj->_pobj2[0].MotionSubspace->size[0] *
      b_obj->_pobj2[0].MotionSubspace->size[1];
  b_obj->_pobj2[0].MotionSubspace->size[0] = 6;
  b_obj->_pobj2[0].MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj2[0].MotionSubspace, i,
                           &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->_pobj2[0].MotionSubspace->data[i] = msubspace_data[i];
  }
  i = b_obj->_pobj2[0].PositionLimitsInternal->size[0] *
      b_obj->_pobj2[0].PositionLimitsInternal->size[1];
  b_obj->_pobj2[0].PositionLimitsInternal->size[0] = 1;
  b_obj->_pobj2[0].PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj2[0].PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    b_obj->_pobj2[0].PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = b_obj->_pobj2[0].HomePositionInternal->size[0];
  b_obj->_pobj2[0].HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&b_st, b_obj->_pobj2[0].HomePositionInternal, i,
                           &oc_emlrtRTEI);
  b_obj->_pobj2[0].HomePositionInternal->data[0] = 0.0;
  c_obj->JointInternal = &b_obj->_pobj2[0];
  c_obj->Index = -1.0;
  c_obj->ParentIndex = -1.0;
  c_obj->MassInternal = 1.0;
  c_obj->CenterOfMassInternal[0] = 0.0;
  c_obj->CenterOfMassInternal[1] = 0.0;
  c_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    c_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    c_obj->SpatialInertia[i] = msubspace_data[i];
  }
  b_st.site = &p_emlrtRSI;
  c_obj->CollisionsInternal = CollisionSet_CollisionSet(&b_st, iobj_0, 0.0);
  c_obj->matlabCodegenIsDeleted = false;
  b_obj->Base.Index = 0.0;
  st.site = &bc_emlrtRSI;
  b_st.site = &ec_emlrtRSI;
  emlrtRandu(&idx[0], 5);
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    idx[loop_ub] = muDoubleScalarFloor(idx[loop_ub] * 62.0) + 1.0;
  }
  for (i = 0; i < 5; i++) {
    d = idx[i];
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &d_emlrtDCI, &b_st);
    }
    if (((int32_T)d < 1) || ((int32_T)d > 62)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 62, &i_emlrtBCI, &b_st);
    }
  }
  st.site = &ac_emlrtRSI;
  b_st.site = &ic_emlrtRSI;
  st.site = &dc_emlrtRSI;
  d_obj = b_obj;
  iobj_0 = &b_obj->_pobj1[1];
  iobj_1 = &b_obj->_pobj2[1];
  b_st.site = &kc_emlrtRSI;
  c_st.site = &m_emlrtRSI;
  d_obj->Bodies[0] =
      RigidBody_RigidBody(&c_st, &(&(&b_obj->_pobj0[0])[0])[0],
                          &(&iobj_0[0])[0], &(&(&b_obj->_pobj2[1])[0])[0]);
  c_st.site = &m_emlrtRSI;
  d_obj->Bodies[1] = b_RigidBody_RigidBody(&c_st, &(&(&b_obj->_pobj0[0])[0])[1],
                                           &(&iobj_0[0])[1], &(&iobj_1[0])[1]);
  c_st.site = &m_emlrtRSI;
  d_obj->Bodies[2] = c_RigidBody_RigidBody(&c_st, &(&(&b_obj->_pobj0[0])[0])[2],
                                           &(&iobj_0[0])[2], &(&iobj_1[0])[2]);
  c_st.site = &m_emlrtRSI;
  d_obj->Bodies[3] = d_RigidBody_RigidBody(&c_st, &(&(&b_obj->_pobj0[0])[0])[3],
                                           &(&iobj_0[0])[3], &(&iobj_1[0])[3]);
  c_st.site = &m_emlrtRSI;
  d_obj->Bodies[4] = e_RigidBody_RigidBody(&c_st, &(&(&b_obj->_pobj0[0])[0])[4],
                                           &(&iobj_0[0])[4], &(&iobj_1[0])[4]);
  c_st.site = &m_emlrtRSI;
  d_obj->Bodies[5] = f_RigidBody_RigidBody(&c_st, &(&(&b_obj->_pobj0[0])[0])[5],
                                           &(&iobj_0[0])[5], &(&iobj_1[0])[5]);
  d_obj->NumBodies = 0.0;
  d_obj->NumNonFixedBodies = 0.0;
  d_obj->PositionNumber = 0.0;
  d_obj->VelocityNumber = 0.0;
  b_st.site = &jc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  emlrtRandu(&idx[0], 5);
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    idx[loop_ub] = muDoubleScalarFloor(idx[loop_ub] * 62.0) + 1.0;
  }
  for (i = 0; i < 5; i++) {
    d = idx[i];
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &d_emlrtDCI, &c_st);
    }
    if (((int32_T)d < 1) || ((int32_T)d > 62)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 62, &i_emlrtBCI, &c_st);
    }
  }
  for (i = 0; i < 12; i++) {
    d_obj->PositionDoFMap[i] = iv2[i];
  }
  for (i = 0; i < 12; i++) {
    d_obj->VelocityDoFMap[i] = iv2[i];
  }
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

void c_RigidBodyTree_efficientFKAndJ(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj,
                                     const emxArray_real_T *qv,
                                     const emxArray_char_T *body1Name,
                                     real_T T_data[], int32_T T_size[2],
                                     emxArray_real_T *Jac)
{
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  c_robotics_manip_internal_Rigid *body1;
  c_robotics_manip_internal_Rigid *body2;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_char_T *body2Name;
  emxArray_real_T *JacSlice;
  emxArray_real_T *ancestorIndices1;
  emxArray_real_T *ancestorIndices2;
  emxArray_real_T *b;
  emxArray_real_T *b_qv;
  emxArray_real_T *kinematicPathIndices;
  rigidBodyJoint *joint;
  real_T Tj1[36];
  real_T T1[16];
  real_T Tj[16];
  real_T b_Tj1[16];
  real_T R[9];
  const real_T *qv_data;
  real_T bid1;
  real_T bid2;
  real_T *ancestorIndices1_data;
  real_T *ancestorIndices2_data;
  real_T *kinematicPathIndices_data;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  int32_T minPathLength;
  const char_T *body1Name_data;
  char_T *body2Name_data;
  boolean_T exitg1;
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
  body1Name_data = body1Name->data;
  qv_data = qv->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &gj_emlrtRSI;
  emxInit_char_T(&st, &body2Name, &bj_emlrtRTEI);
  i = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(&st, body2Name, i, &ri_emlrtRTEI);
  body2Name_data = body2Name->data;
  loop_ub = obj->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    body2Name_data[i] = obj->Base.NameInternal->data[i];
  }
  st.site = &fj_emlrtRSI;
  b_st.site = &nd_emlrtRSI;
  bid1 = c_RigidBodyTree_findBodyIndexBy(&b_st, obj, body1Name);
  if (bid1 == -1.0) {
    b_st.site = &od_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI, "robotics:robotmanip:rigidbodytree:BodyNotFound",
        "robotics:robotmanip:rigidbodytree:BodyNotFound", 3, 4,
        body1Name->size[1], &body1Name_data[0]);
  }
  st.site = &ej_emlrtRSI;
  b_st.site = &nd_emlrtRSI;
  bid2 = c_RigidBodyTree_findBodyIndexBy(&b_st, obj, body2Name);
  if (bid2 == -1.0) {
    b_st.site = &od_emlrtRSI;
    emlrtErrorWithMessageIdR2018a(
        &b_st, &e_emlrtRTEI, "robotics:robotmanip:rigidbodytree:BodyNotFound",
        "robotics:robotmanip:rigidbodytree:BodyNotFound", 3, 4,
        body2Name->size[1], &body2Name_data[0]);
  }
  if (bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    if (((int32_T)bid1 - 1 < 0) || ((int32_T)bid1 - 1 > 5)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)bid1 - 1, 0, 5, &qd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    body1 = obj->Bodies[(int32_T)bid1 - 1];
  }
  if (bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    if (((int32_T)bid2 - 1 < 0) || ((int32_T)bid2 - 1 > 5)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)bid2 - 1, 0, 5, &rd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    body2 = obj->Bodies[(int32_T)bid2 - 1];
  }
  st.site = &dj_emlrtRSI;
  emxInit_real_T(&st, &ancestorIndices1, 2, &cj_emlrtRTEI);
  b_st.site = &hj_emlrtRSI;
  RigidBodyTree_ancestorIndices(&b_st, obj, body1, ancestorIndices1);
  ancestorIndices1_data = ancestorIndices1->data;
  emxInit_real_T(&st, &ancestorIndices2, 2, &dj_emlrtRTEI);
  b_st.site = &ij_emlrtRSI;
  RigidBodyTree_ancestorIndices(&b_st, obj, body2, ancestorIndices2);
  ancestorIndices2_data = ancestorIndices2->data;
  minPathLength = (int32_T)muDoubleScalarMin(ancestorIndices1->size[1],
                                             ancestorIndices2->size[1]);
  b_i = 1;
  exitg1 = false;
  while ((!exitg1) && (b_i - 1 <= minPathLength - 2)) {
    i = ancestorIndices1->size[1] - b_i;
    if ((i < 1) || (i > ancestorIndices1->size[1])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, ancestorIndices1->size[1],
                                    &sd_emlrtBCI, &st);
    }
    i1 = ancestorIndices2->size[1] - b_i;
    if ((i1 < 1) || (i1 > ancestorIndices2->size[1])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, ancestorIndices2->size[1],
                                    &td_emlrtBCI, &st);
    }
    if (ancestorIndices1_data[i - 1] != ancestorIndices2_data[i1 - 1]) {
      minPathLength = b_i;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  i = (ancestorIndices1->size[1] - minPathLength) + 1;
  if ((i < 1) || (i > ancestorIndices1->size[1])) {
    emlrtDynamicBoundsCheckR2012b(i, 1, ancestorIndices1->size[1], &ud_emlrtBCI,
                                  &st);
  }
  loop_ub = ancestorIndices1->size[1] - minPathLength;
  if (loop_ub < 1) {
    loop_ub = 0;
  } else if (loop_ub > ancestorIndices1->size[1]) {
    emlrtDynamicBoundsCheckR2012b(loop_ub, 1, ancestorIndices1->size[1],
                                  &od_emlrtBCI, &st);
  }
  i1 = ancestorIndices2->size[1] - minPathLength;
  if (i1 < 1) {
    i1 = 0;
    i2 = 1;
    i3 = -1;
  } else {
    if (i1 > ancestorIndices2->size[1]) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, ancestorIndices2->size[1],
                                    &pd_emlrtBCI, &st);
    }
    i1--;
    i2 = -1;
    i3 = 0;
  }
  emxInit_real_T(&st, &kinematicPathIndices, 2, &si_emlrtRTEI);
  minPathLength = kinematicPathIndices->size[0] * kinematicPathIndices->size[1];
  kinematicPathIndices->size[0] = 1;
  b_st.site = &dj_emlrtRSI;
  b_loop_ub = div_s32(&b_st, i3 - i1, i2);
  kinematicPathIndices->size[1] = (loop_ub + b_loop_ub) + 2;
  emxEnsureCapacity_real_T(&st, kinematicPathIndices, minPathLength,
                           &si_emlrtRTEI);
  kinematicPathIndices_data = kinematicPathIndices->data;
  for (i3 = 0; i3 < loop_ub; i3++) {
    kinematicPathIndices_data[i3] = ancestorIndices1_data[i3];
  }
  kinematicPathIndices_data[loop_ub] = ancestorIndices1_data[i - 1];
  emxFree_real_T(&st, &ancestorIndices1);
  for (i = 0; i <= b_loop_ub; i++) {
    kinematicPathIndices_data[(i + loop_ub) + 1] =
        ancestorIndices2_data[i1 + i2 * i];
  }
  emxFree_real_T(&st, &ancestorIndices2);
  memset(&T1[0], 0, 16U * sizeof(real_T));
  T1[0] = 1.0;
  T1[5] = 1.0;
  T1[10] = 1.0;
  T1[15] = 1.0;
  i = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  emxEnsureCapacity_real_T(sp, Jac, i, &ti_emlrtRTEI);
  if (!(obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(obj->PositionNumber, &oc_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  bid1 = obj->PositionNumber;
  if (bid1 != (int32_T)muDoubleScalarFloor(bid1)) {
    emlrtIntegerCheckR2012b(bid1, &nc_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = Jac->size[0] * Jac->size[1];
  Jac->size[1] = (int32_T)bid1;
  emxEnsureCapacity_real_T(sp, Jac, i, &ti_emlrtRTEI);
  ancestorIndices2_data = Jac->data;
  if (!(obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(obj->PositionNumber, &qc_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  bid1 = obj->PositionNumber;
  if (bid1 != (int32_T)muDoubleScalarFloor(bid1)) {
    emlrtIntegerCheckR2012b(bid1, &pc_emlrtDCI, (emlrtConstCTX)sp);
  }
  loop_ub = 6 * (int32_T)bid1;
  for (i = 0; i < loop_ub; i++) {
    ancestorIndices2_data[i] = 0.0;
  }
  i = kinematicPathIndices->size[1];
  emxInit_real_T(sp, &JacSlice, 2, &aj_emlrtRTEI);
  emxInit_real_T(sp, &b, 2, &yi_emlrtRTEI);
  emxInit_real_T(sp, &b_qv, 1, &xi_emlrtRTEI);
  for (b_i = 0; b_i <= i - 2; b_i++) {
    real_T Tc2p[16];
    real_T tempR[9];
    real_T b_tempR_tmp;
    real_T c_tempR_tmp;
    real_T tempR_tmp;
    int32_T exitg2;
    int32_T jointSign;
    boolean_T b_bool;
    boolean_T nextBodyIsParent;
    if (b_i + 1 > kinematicPathIndices->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, kinematicPathIndices->size[1],
                                    &vd_emlrtBCI, (emlrtConstCTX)sp);
    }
    if (kinematicPathIndices_data[b_i] != 0.0) {
      if (b_i + 1 > kinematicPathIndices->size[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, kinematicPathIndices->size[1],
                                      &yd_emlrtBCI, (emlrtConstCTX)sp);
      }
      bid1 = kinematicPathIndices_data[b_i];
      if (bid1 != (int32_T)muDoubleScalarFloor(bid1)) {
        emlrtIntegerCheckR2012b(bid1, &rc_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)bid1 - 1 < 0) || ((int32_T)bid1 - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid1 - 1, 0, 5, &xd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      body1 = obj->Bodies[(int32_T)bid1 - 1];
    } else {
      body1 = &obj->Base;
    }
    if (b_i + 2 > kinematicPathIndices->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, kinematicPathIndices->size[1],
                                    &wd_emlrtBCI, (emlrtConstCTX)sp);
    }
    if (kinematicPathIndices_data[b_i + 1] != 0.0) {
      if (b_i + 2 > kinematicPathIndices->size[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, kinematicPathIndices->size[1],
                                      &be_emlrtBCI, (emlrtConstCTX)sp);
      }
      bid1 = kinematicPathIndices_data[b_i + 1];
      if (bid1 != (int32_T)muDoubleScalarFloor(bid1)) {
        emlrtIntegerCheckR2012b(bid1, &sc_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)bid1 - 1 < 0) || ((int32_T)bid1 - 1 > 5)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)bid1 - 1, 0, 5, &ae_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      body2 = obj->Bodies[(int32_T)bid1 - 1];
    } else {
      body2 = &obj->Base;
    }
    nextBodyIsParent = (body2->Index == body1->ParentIndex);
    if (nextBodyIsParent) {
      body2 = body1;
      jointSign = 1;
    } else {
      jointSign = -1;
    }
    joint = body2->JointInternal;
    i1 = body2Name->size[0] * body2Name->size[1];
    body2Name->size[0] = 1;
    body2Name->size[1] = joint->Type->size[1];
    emxEnsureCapacity_char_T(sp, body2Name, i1, &vi_emlrtRTEI);
    body2Name_data = body2Name->data;
    loop_ub = joint->Type->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      body2Name_data[i1] = joint->Type->data[i1];
    }
    b_bool = false;
    if (body2Name->size[1] == 5) {
      minPathLength = 0;
      do {
        exitg2 = 0;
        if (minPathLength < 5) {
          if (body2Name_data[minPathLength] != b_cv[minPathLength]) {
            exitg2 = 1;
          } else {
            minPathLength++;
          }
        } else {
          b_bool = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }
    if (b_bool) {
      real_T T1j[16];
      real_T c_Tj1[16];
      st.site = &cj_emlrtRSI;
      for (i1 = 0; i1 < 16; i1++) {
        b_Tj1[i1] = joint->JointToParentTransform[i1];
      }
      b_st.site = &jj_emlrtRSI;
      i1 = body2Name->size[0] * body2Name->size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      emxEnsureCapacity_char_T(&b_st, body2Name, i1, &wi_emlrtRTEI);
      body2Name_data = body2Name->data;
      loop_ub = joint->Type->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        body2Name_data[i1] = joint->Type->data[i1];
      }
      b_bool = false;
      if (body2Name->size[1] == 5) {
        minPathLength = 0;
        do {
          exitg2 = 0;
          if (minPathLength < 5) {
            if (body2Name_data[minPathLength] != b_cv[minPathLength]) {
              exitg2 = 1;
            } else {
              minPathLength++;
            }
          } else {
            b_bool = true;
            exitg2 = 1;
          }
        } while (exitg2 == 0);
      }
      if (b_bool) {
        minPathLength = 0;
      } else {
        b_bool = false;
        if (body2Name->size[1] == 8) {
          minPathLength = 0;
          do {
            exitg2 = 0;
            if (minPathLength < 8) {
              if (body2Name_data[minPathLength] != b_cv1[minPathLength]) {
                exitg2 = 1;
              } else {
                minPathLength++;
              }
            } else {
              b_bool = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }
        if (b_bool) {
          minPathLength = 1;
        } else {
          minPathLength = -1;
        }
      }
      switch (minPathLength) {
      case 0:
        memset(&Tj[0], 0, 16U * sizeof(real_T));
        Tj[0] = 1.0;
        Tj[5] = 1.0;
        Tj[10] = 1.0;
        Tj[15] = 1.0;
        break;
      case 1: {
        real_T b_v[3];
        real_T v[3];
        real_T d_tempR_tmp;
        real_T e_tempR_tmp;
        real_T f_tempR_tmp;
        real_T qidx_idx_1;
        c_st.site = &kj_emlrtRSI;
        rigidBodyJoint_get_JointAxis(joint, v);
        c_st.site = &lj_emlrtRSI;
        d_st.site = &nj_emlrtRSI;
        e_st.site = &oj_emlrtRSI;
        normalizeRows(&e_st, v, b_v);
        bid1 = b_v[0] * b_v[0] * 0.0 + 1.0;
        tempR[0] = bid1;
        bid2 = b_v[0] * b_v[1] * 0.0;
        qidx_idx_1 = bid2 - b_v[2] * 0.0;
        tempR[1] = qidx_idx_1;
        tempR_tmp = b_v[0] * b_v[2] * 0.0;
        b_tempR_tmp = tempR_tmp + b_v[1] * 0.0;
        tempR[2] = b_tempR_tmp;
        bid2 += b_v[2] * 0.0;
        tempR[3] = bid2;
        c_tempR_tmp = b_v[1] * b_v[1] * 0.0 + 1.0;
        tempR[4] = c_tempR_tmp;
        d_tempR_tmp = b_v[1] * b_v[2] * 0.0;
        e_tempR_tmp = d_tempR_tmp - b_v[0] * 0.0;
        tempR[5] = e_tempR_tmp;
        tempR_tmp -= b_v[1] * 0.0;
        tempR[6] = tempR_tmp;
        d_tempR_tmp += b_v[0] * 0.0;
        tempR[7] = d_tempR_tmp;
        f_tempR_tmp = b_v[2] * b_v[2] * 0.0 + 1.0;
        tempR[8] = f_tempR_tmp;
        R[0] = bid1;
        R[1] = qidx_idx_1;
        R[2] = b_tempR_tmp;
        R[3] = bid2;
        R[4] = c_tempR_tmp;
        R[5] = e_tempR_tmp;
        R[6] = tempR_tmp;
        R[7] = d_tempR_tmp;
        R[8] = f_tempR_tmp;
        for (minPathLength = 0; minPathLength < 3; minPathLength++) {
          R[minPathLength] = tempR[3 * minPathLength];
          R[minPathLength + 3] = tempR[3 * minPathLength + 1];
          R[minPathLength + 6] = tempR[3 * minPathLength + 2];
        }
        memset(&Tj[0], 0, 16U * sizeof(real_T));
        for (i1 = 0; i1 < 3; i1++) {
          minPathLength = i1 << 2;
          Tj[minPathLength] = R[3 * i1];
          Tj[minPathLength + 1] = R[3 * i1 + 1];
          Tj[minPathLength + 2] = R[3 * i1 + 2];
        }
        Tj[15] = 1.0;
      } break;
      default: {
        real_T v[3];
        c_st.site = &mj_emlrtRSI;
        rigidBodyJoint_get_JointAxis(joint, v);
        memset(&R[0], 0, 9U * sizeof(real_T));
        R[0] = 1.0;
        R[4] = 1.0;
        R[8] = 1.0;
        for (i1 = 0; i1 < 3; i1++) {
          minPathLength = i1 << 2;
          Tj[minPathLength] = R[3 * i1];
          Tj[minPathLength + 1] = R[3 * i1 + 1];
          Tj[minPathLength + 2] = R[3 * i1 + 2];
          Tj[i1 + 12] = v[i1] * 0.0;
        }
        Tj[3] = 0.0;
        Tj[7] = 0.0;
        Tj[11] = 0.0;
        Tj[15] = 1.0;
      } break;
      }
      for (i1 = 0; i1 < 16; i1++) {
        T1j[i1] = joint->ChildToJointTransform[i1];
      }
      for (i1 = 0; i1 < 4; i1++) {
        bid1 = b_Tj1[i1];
        tempR_tmp = b_Tj1[i1 + 4];
        b_tempR_tmp = b_Tj1[i1 + 8];
        c_tempR_tmp = b_Tj1[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          c_Tj1[i1 + i3] = ((bid1 * Tj[i3] + tempR_tmp * Tj[i3 + 1]) +
                            b_tempR_tmp * Tj[i3 + 2]) +
                           c_tempR_tmp * Tj[i3 + 3];
        }
        bid1 = c_Tj1[i1];
        tempR_tmp = c_Tj1[i1 + 4];
        b_tempR_tmp = c_Tj1[i1 + 8];
        c_tempR_tmp = c_Tj1[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          Tc2p[i1 + i3] = ((bid1 * T1j[i3] + tempR_tmp * T1j[i3 + 1]) +
                           b_tempR_tmp * T1j[i3 + 2]) +
                          c_tempR_tmp * T1j[i3 + 3];
        }
      }
    } else {
      real_T T1j[16];
      real_T qidx_idx_1;
      if (body2->Index != (int32_T)muDoubleScalarFloor(body2->Index)) {
        emlrtIntegerCheckR2012b(body2->Index, &mc_emlrtDCI, (emlrtConstCTX)sp);
      }
      minPathLength = (int32_T)body2->Index;
      if ((minPathLength < 1) || (minPathLength > 6)) {
        emlrtDynamicBoundsCheckR2012b(minPathLength, 1, 6, &nd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      bid2 = obj->PositionDoFMap[minPathLength - 1];
      qidx_idx_1 = obj->PositionDoFMap[minPathLength + 5];
      if (bid2 > qidx_idx_1) {
        i1 = 0;
        i2 = 0;
      } else {
        if (bid2 != (int32_T)muDoubleScalarFloor(bid2)) {
          emlrtIntegerCheckR2012b(bid2, &lc_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)bid2 < 1) || ((int32_T)bid2 > qv->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)bid2, 1, qv->size[0],
                                        &md_emlrtBCI, (emlrtConstCTX)sp);
        }
        i1 = (int32_T)bid2 - 1;
        if (qidx_idx_1 != (int32_T)muDoubleScalarFloor(qidx_idx_1)) {
          emlrtIntegerCheckR2012b(qidx_idx_1, &kc_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)qidx_idx_1 < 1) || ((int32_T)qidx_idx_1 > qv->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)qidx_idx_1, 1, qv->size[0],
                                        &ld_emlrtBCI, (emlrtConstCTX)sp);
        }
        i2 = (int32_T)qidx_idx_1;
      }
      b_iv[0] = 1;
      loop_ub = i2 - i1;
      b_iv[1] = loop_ub;
      st.site = &bj_emlrtRSI;
      indexShapeCheck(&st, qv->size[0], b_iv);
      i2 = b_qv->size[0];
      b_qv->size[0] = loop_ub;
      emxEnsureCapacity_real_T(sp, b_qv, i2, &xi_emlrtRTEI);
      ancestorIndices1_data = b_qv->data;
      for (i2 = 0; i2 < loop_ub; i2++) {
        ancestorIndices1_data[i2] = qv_data[i1 + i2];
      }
      st.site = &bj_emlrtRSI;
      c_rigidBodyJoint_transformBodyT(&st, joint, b_qv, Tc2p);
      if (body2->Index != (int32_T)muDoubleScalarFloor(body2->Index)) {
        emlrtIntegerCheckR2012b(body2->Index, &jc_emlrtDCI, (emlrtConstCTX)sp);
      }
      minPathLength = (int32_T)body2->Index;
      if ((minPathLength < 1) || (minPathLength > 6)) {
        emlrtDynamicBoundsCheckR2012b(minPathLength, 1, 6, &kd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      bid2 = obj->VelocityDoFMap[minPathLength - 1];
      qidx_idx_1 = obj->VelocityDoFMap[minPathLength + 5];
      if (nextBodyIsParent) {
        for (i1 = 0; i1 < 16; i1++) {
          Tj[i1] = joint->ChildToJointTransform[i1];
        }
      } else {
        for (i1 = 0; i1 < 16; i1++) {
          b_Tj1[i1] = joint->JointToParentTransform[i1];
        }
        for (i1 = 0; i1 < 3; i1++) {
          R[3 * i1] = b_Tj1[i1];
          R[3 * i1 + 1] = b_Tj1[i1 + 4];
          R[3 * i1 + 2] = b_Tj1[i1 + 8];
        }
        for (i1 = 0; i1 < 9; i1++) {
          tempR[i1] = -R[i1];
        }
        bid1 = b_Tj1[12];
        tempR_tmp = b_Tj1[13];
        b_tempR_tmp = b_Tj1[14];
        for (i1 = 0; i1 < 3; i1++) {
          minPathLength = i1 << 2;
          Tj[minPathLength] = R[3 * i1];
          Tj[minPathLength + 1] = R[3 * i1 + 1];
          Tj[minPathLength + 2] = R[3 * i1 + 2];
          Tj[i1 + 12] = (tempR[i1] * bid1 + tempR[i1 + 3] * tempR_tmp) +
                        tempR[i1 + 6] * b_tempR_tmp;
        }
        Tj[3] = 0.0;
        Tj[7] = 0.0;
        Tj[11] = 0.0;
        Tj[15] = 1.0;
      }
      for (i1 = 0; i1 < 4; i1++) {
        bid1 = Tj[i1];
        tempR_tmp = Tj[i1 + 4];
        b_tempR_tmp = Tj[i1 + 8];
        c_tempR_tmp = Tj[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          T1j[i1 + i3] = ((bid1 * T1[i3] + tempR_tmp * T1[i3 + 1]) +
                          b_tempR_tmp * T1[i3 + 2]) +
                         c_tempR_tmp * T1[i3 + 3];
        }
      }
      for (i1 = 0; i1 < 3; i1++) {
        R[3 * i1] = T1j[i1];
        R[3 * i1 + 1] = T1j[i1 + 4];
        R[3 * i1 + 2] = T1j[i1 + 8];
      }
      for (i1 = 0; i1 < 9; i1++) {
        tempR[i1] = -R[i1];
      }
      bid1 = T1j[12];
      tempR_tmp = T1j[13];
      b_tempR_tmp = T1j[14];
      for (i1 = 0; i1 < 3; i1++) {
        minPathLength = i1 << 2;
        b_Tj1[minPathLength] = R[3 * i1];
        b_Tj1[minPathLength + 1] = R[3 * i1 + 1];
        b_Tj1[minPathLength + 2] = R[3 * i1 + 2];
        b_Tj1[i1 + 12] = (tempR[i1] * bid1 + tempR[i1 + 3] * tempR_tmp) +
                         tempR[i1 + 6] * b_tempR_tmp;
      }
      b_Tj1[3] = 0.0;
      b_Tj1[7] = 0.0;
      b_Tj1[11] = 0.0;
      b_Tj1[15] = 1.0;
      st.site = &aj_emlrtRSI;
      i1 = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      emxEnsureCapacity_real_T(&st, b, i1, &yi_emlrtRTEI);
      ancestorIndices1_data = b->data;
      loop_ub = 6 * joint->MotionSubspace->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        ancestorIndices1_data[i1] = joint->MotionSubspace->data[i1];
      }
      R[0] = 0.0;
      R[3] = -b_Tj1[14];
      R[6] = b_Tj1[13];
      R[1] = b_Tj1[14];
      R[4] = 0.0;
      R[7] = -b_Tj1[12];
      R[2] = -b_Tj1[13];
      R[5] = b_Tj1[12];
      R[8] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        bid1 = R[i1];
        tempR_tmp = R[i1 + 3];
        b_tempR_tmp = R[i1 + 6];
        for (i2 = 0; i2 < 3; i2++) {
          i3 = i2 << 2;
          tempR[i1 + 3 * i2] = (bid1 * b_Tj1[i3] + tempR_tmp * b_Tj1[i3 + 1]) +
                               b_tempR_tmp * b_Tj1[i3 + 2];
          Tj1[i2 + 6 * i1] = b_Tj1[i2 + (i1 << 2)];
          Tj1[i2 + 6 * (i1 + 3)] = 0.0;
        }
      }
      for (i1 = 0; i1 < 3; i1++) {
        Tj1[6 * i1 + 3] = tempR[3 * i1];
        minPathLength = i1 << 2;
        b_loop_ub = 6 * (i1 + 3);
        Tj1[b_loop_ub + 3] = b_Tj1[minPathLength];
        Tj1[6 * i1 + 4] = tempR[3 * i1 + 1];
        Tj1[b_loop_ub + 4] = b_Tj1[minPathLength + 1];
        Tj1[6 * i1 + 5] = tempR[3 * i1 + 2];
        Tj1[b_loop_ub + 5] = b_Tj1[minPathLength + 2];
      }
      b_st.site = &fk_emlrtRSI;
      mtimes(&b_st, Tj1, b, JacSlice);
      loop_ub = 6 * JacSlice->size[1];
      i1 = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      emxEnsureCapacity_real_T(sp, JacSlice, i1, &aj_emlrtRTEI);
      ancestorIndices1_data = JacSlice->data;
      for (i1 = 0; i1 < loop_ub; i1++) {
        ancestorIndices1_data[i1] *= (real_T)jointSign;
      }
      if (bid2 > qidx_idx_1) {
        i1 = 0;
        i2 = 0;
      } else {
        if (bid2 != (int32_T)muDoubleScalarFloor(bid2)) {
          emlrtIntegerCheckR2012b(bid2, &ic_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)bid2 < 1) || ((int32_T)bid2 > Jac->size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)bid2, 1, Jac->size[1],
                                        &jd_emlrtBCI, (emlrtConstCTX)sp);
        }
        i1 = (int32_T)bid2 - 1;
        if (qidx_idx_1 != (int32_T)muDoubleScalarFloor(qidx_idx_1)) {
          emlrtIntegerCheckR2012b(qidx_idx_1, &hc_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)qidx_idx_1 < 1) || ((int32_T)qidx_idx_1 > Jac->size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)qidx_idx_1, 1, Jac->size[1],
                                        &id_emlrtBCI, (emlrtConstCTX)sp);
        }
        i2 = (int32_T)qidx_idx_1;
      }
      b_iv[0] = 6;
      b_iv[1] = i2 - i1;
      emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &JacSlice->size[0], 2,
                                    &sb_emlrtECI, (emlrtCTX)sp);
      loop_ub = JacSlice->size[1];
      for (i2 = 0; i2 < loop_ub; i2++) {
        for (i3 = 0; i3 < 6; i3++) {
          ancestorIndices2_data[i3 + 6 * (i1 + i2)] =
              ancestorIndices1_data[i3 + 6 * i2];
        }
      }
    }
    if (nextBodyIsParent) {
      for (i1 = 0; i1 < 4; i1++) {
        bid1 = Tc2p[i1];
        tempR_tmp = Tc2p[i1 + 4];
        b_tempR_tmp = Tc2p[i1 + 8];
        c_tempR_tmp = Tc2p[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          b_Tj1[i1 + i3] = ((bid1 * T1[i3] + tempR_tmp * T1[i3 + 1]) +
                            b_tempR_tmp * T1[i3 + 2]) +
                           c_tempR_tmp * T1[i3 + 3];
        }
      }
      memcpy(&T1[0], &b_Tj1[0], 16U * sizeof(real_T));
    } else {
      for (i1 = 0; i1 < 3; i1++) {
        R[3 * i1] = Tc2p[i1];
        R[3 * i1 + 1] = Tc2p[i1 + 4];
        R[3 * i1 + 2] = Tc2p[i1 + 8];
      }
      for (i1 = 0; i1 < 9; i1++) {
        tempR[i1] = -R[i1];
      }
      bid1 = Tc2p[12];
      tempR_tmp = Tc2p[13];
      b_tempR_tmp = Tc2p[14];
      for (i1 = 0; i1 < 3; i1++) {
        minPathLength = i1 << 2;
        b_Tj1[minPathLength] = R[3 * i1];
        b_Tj1[minPathLength + 1] = R[3 * i1 + 1];
        b_Tj1[minPathLength + 2] = R[3 * i1 + 2];
        b_Tj1[i1 + 12] = (tempR[i1] * bid1 + tempR[i1 + 3] * tempR_tmp) +
                         tempR[i1 + 6] * b_tempR_tmp;
      }
      b_Tj1[3] = 0.0;
      b_Tj1[7] = 0.0;
      b_Tj1[11] = 0.0;
      b_Tj1[15] = 1.0;
      for (i1 = 0; i1 < 4; i1++) {
        bid1 = b_Tj1[i1];
        tempR_tmp = b_Tj1[i1 + 4];
        b_tempR_tmp = b_Tj1[i1 + 8];
        c_tempR_tmp = b_Tj1[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          Tj[i1 + i3] = ((bid1 * T1[i3] + tempR_tmp * T1[i3 + 1]) +
                         b_tempR_tmp * T1[i3 + 2]) +
                        c_tempR_tmp * T1[i3 + 3];
        }
      }
      memcpy(&T1[0], &Tj[0], 16U * sizeof(real_T));
    }
  }
  emxFree_real_T(sp, &b_qv);
  emxFree_real_T(sp, &b);
  emxFree_char_T(sp, &body2Name);
  emxFree_real_T(sp, &kinematicPathIndices);
  st.site = &yi_emlrtRSI;
  for (i = 0; i < 3; i++) {
    i1 = i << 2;
    bid1 = T1[i1];
    Tj1[6 * i] = bid1;
    minPathLength = 6 * (i + 3);
    Tj1[minPathLength] = 0.0;
    Tj1[6 * i + 3] = 0.0;
    Tj1[minPathLength + 3] = bid1;
    bid1 = T1[i1 + 1];
    Tj1[6 * i + 1] = bid1;
    Tj1[minPathLength + 1] = 0.0;
    Tj1[6 * i + 4] = 0.0;
    Tj1[minPathLength + 4] = bid1;
    bid1 = T1[i1 + 2];
    Tj1[6 * i + 2] = bid1;
    Tj1[minPathLength + 2] = 0.0;
    Tj1[6 * i + 5] = 0.0;
    Tj1[minPathLength + 5] = bid1;
  }
  i = JacSlice->size[0] * JacSlice->size[1];
  JacSlice->size[0] = 6;
  JacSlice->size[1] = Jac->size[1];
  emxEnsureCapacity_real_T(&st, JacSlice, i, &ui_emlrtRTEI);
  ancestorIndices1_data = JacSlice->data;
  loop_ub = Jac->size[0] * Jac->size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    ancestorIndices1_data[i] = ancestorIndices2_data[i];
  }
  b_st.site = &fk_emlrtRSI;
  mtimes(&b_st, Tj1, JacSlice, Jac);
  emxFree_real_T(&st, &JacSlice);
  T_size[0] = 4;
  T_size[1] = 4;
  memcpy(&T_data[0], &T1[0], 16U * sizeof(real_T));
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

real_T c_RigidBodyTree_findBodyIndexBy(const emlrtStack *sp,
                                       e_robotics_manip_internal_Rigid *obj,
                                       const emxArray_char_T *bodyname)
{
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack st;
  emxArray_char_T *bname;
  real_T bid;
  int32_T i;
  int32_T loop_ub;
  char_T *bname_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  bid = -1.0;
  st.site = &pc_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  if (bodyname->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:findBodyIndexByName:expectedNonempty", 3, 4, 8, "bodyname");
  }
  emxInit_char_T(sp, &bname, &bd_emlrtRTEI);
  st.site = &qc_emlrtRSI;
  i = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(&st, bname, i, &yc_emlrtRTEI);
  bname_data = bname->data;
  loop_ub = obj->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    bname_data[i] = obj->Base.NameInternal->data[i];
  }
  st.site = &qc_emlrtRSI;
  if (b_strcmp(&st, bname, bodyname)) {
    bid = 0.0;
  } else {
    real_T d;
    int32_T b_i;
    boolean_T exitg1;
    d = obj->NumBodies;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                  &h_emlrtRTEI, (emlrtConstCTX)sp);
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i <= (int32_T)d - 1)) {
      st.site = &rc_emlrtRSI;
      if (b_i > 5) {
        emlrtDynamicBoundsCheckR2012b(6, 0, 5, &j_emlrtBCI, &st);
      }
      b_obj = obj->Bodies[b_i];
      i = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = b_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(&st, bname, i, &yc_emlrtRTEI);
      bname_data = bname->data;
      loop_ub = b_obj->NameInternal->size[1];
      for (i = 0; i < loop_ub; i++) {
        bname_data[i] = b_obj->NameInternal->data[i];
      }
      st.site = &rc_emlrtRSI;
      if (b_strcmp(&st, bname, bodyname)) {
        bid = (real_T)b_i + 1.0;
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }
  emxFree_char_T(sp, &bname);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return bid;
}

void c_RigidBodyTree_get_JointPositi(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj,
                                     emxArray_real_T *limits)
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  c_robotics_manip_internal_Rigid *body;
  emlrtStack st;
  emxArray_char_T *a;
  emxArray_real_T *r;
  rigidBodyJoint *b_obj;
  real_T k;
  real_T pnum;
  real_T *limits_data;
  real_T *r1;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  char_T *a_data;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (!(obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(obj->PositionNumber, &fb_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  pnum = obj->PositionNumber;
  if (pnum != (int32_T)muDoubleScalarFloor(pnum)) {
    emlrtIntegerCheckR2012b(pnum, &eb_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = limits->size[0] * limits->size[1];
  limits->size[0] = (int32_T)pnum;
  limits->size[1] = 2;
  emxEnsureCapacity_real_T(sp, limits, i, &je_emlrtRTEI);
  limits_data = limits->data;
  if (!(obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(obj->PositionNumber, &hb_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  pnum = obj->PositionNumber;
  if (pnum != (int32_T)muDoubleScalarFloor(pnum)) {
    emlrtIntegerCheckR2012b(pnum, &gb_emlrtDCI, (emlrtConstCTX)sp);
  }
  loop_ub = (int32_T)pnum << 1;
  for (i = 0; i < loop_ub; i++) {
    limits_data[i] = 0.0;
  }
  k = 1.0;
  pnum = obj->NumBodies;
  i = (int32_T)pnum;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, pnum, mxDOUBLE_CLASS, (int32_T)pnum,
                                &r_emlrtRTEI, (emlrtConstCTX)sp);
  emxInit_real_T(sp, &r, 2, &me_emlrtRTEI);
  emxInit_char_T(sp, &a, &ke_emlrtRTEI);
  for (b_i = 0; b_i < i; b_i++) {
    boolean_T b_bool;
    if (b_i > 5) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, 5, &kb_emlrtBCI, (emlrtConstCTX)sp);
    }
    body = obj->Bodies[b_i];
    i1 = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    emxEnsureCapacity_char_T(sp, a, i1, &ke_emlrtRTEI);
    a_data = a->data;
    loop_ub = body->JointInternal->Type->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      a_data[i1] = body->JointInternal->Type->data[i1];
    }
    b_bool = false;
    if (a->size[1] == 5) {
      loop_ub = 0;
      int32_T exitg1;
      do {
        exitg1 = 0;
        if (loop_ub < 5) {
          if (a_data[loop_ub] != b_cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (!b_bool) {
      pnum = body->JointInternal->PositionNumber;
      pnum += k;
      if (k > pnum - 1.0) {
        i1 = 0;
        i2 = 0;
      } else {
        if (k != (int32_T)muDoubleScalarFloor(k)) {
          emlrtIntegerCheckR2012b(k, &cb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)k < 1) || ((int32_T)k > limits->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, limits->size[0],
                                        &ib_emlrtBCI, (emlrtConstCTX)sp);
        }
        i1 = (int32_T)k - 1;
        if (pnum - 1.0 != (int32_T)muDoubleScalarFloor(pnum - 1.0)) {
          emlrtIntegerCheckR2012b(pnum - 1.0, &db_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)(pnum - 1.0) < 1) ||
            ((int32_T)(pnum - 1.0) > limits->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)(pnum - 1.0), 1,
                                        limits->size[0], &jb_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        i2 = (int32_T)(pnum - 1.0);
      }
      st.site = &hf_emlrtRSI;
      b_obj = body->JointInternal;
      i3 = r->size[0] * r->size[1];
      r->size[0] = b_obj->PositionLimitsInternal->size[0];
      r->size[1] = 2;
      emxEnsureCapacity_real_T(&st, r, i3, &le_emlrtRTEI);
      r1 = r->data;
      loop_ub = b_obj->PositionLimitsInternal->size[0] * 2;
      for (i3 = 0; i3 < loop_ub; i3++) {
        r1[i3] = b_obj->PositionLimitsInternal->data[i3];
      }
      b_iv[0] = i2 - i1;
      b_iv[1] = 2;
      emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &r->size[0], 2, &f_emlrtECI,
                                    (emlrtCTX)sp);
      loop_ub = r->size[0];
      for (i2 = 0; i2 < 2; i2++) {
        for (i3 = 0; i3 < loop_ub; i3++) {
          limits_data[(i1 + i3) + limits->size[0] * i2] =
              r1[i3 + r->size[0] * i2];
        }
      }
      k = pnum;
    }
  }
  emxFree_char_T(sp, &a);
  emxFree_real_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_RigidBodyTree_validateConfigu(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj,
                                     const emxArray_struct_T *Q,
                                     emxArray_real_T *qvec)
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const char_T varargin_1[5] = {'J', 'o', 'i', 'n', 't'};
  c_robotics_manip_internal_Rigid *body;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T *lbOK;
  emxArray_boolean_T *ubOK;
  emxArray_char_T b_Q;
  emxArray_char_T *a;
  emxArray_int32_T *ii;
  emxArray_int32_T *indicesUpperBoundViolation;
  emxArray_real_T *limits;
  rigidBodyJoint *jnt;
  const struct_T *Q_data;
  real_T idx_idx_0;
  real_T *limits_data;
  real_T *qvec_data;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T *ii_data;
  int32_T *indicesUpperBoundViolation_data;
  char_T *a_data;
  boolean_T *lbOK_data;
  boolean_T *ubOK_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  Q_data = Q->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ag_emlrtRSI;
  idx_idx_0 = obj->NumNonFixedBodies;
  b_st.site = &gg_emlrtRSI;
  c_st.site = &y_emlrtRSI;
  if ((!(muDoubleScalarFloor(idx_idx_0) == idx_idx_0)) ||
      muDoubleScalarIsInf(idx_idx_0) || (!(idx_idx_0 >= 0.0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &v_emlrtRTEI,
                                  "Coder:toolbox:ValidateattributesBadNumel",
                                  "MATLAB:validateattributes:badnumel", 0);
  }
  if (!(Q->size[1] == idx_idx_0)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &w_emlrtRTEI, "Coder:toolbox:ValidateattributesincorrectNumel",
        "MATLAB:validateConfiguration:incorrectNumel", 5, 4, 5, "input", 6,
        idx_idx_0);
  }
  if (!(obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(obj->PositionNumber, &ac_emlrtDCI, &st);
  }
  idx_idx_0 = obj->PositionNumber;
  if (idx_idx_0 != (int32_T)muDoubleScalarFloor(idx_idx_0)) {
    emlrtIntegerCheckR2012b(idx_idx_0, &yb_emlrtDCI, &st);
  }
  i = qvec->size[0];
  qvec->size[0] = (int32_T)idx_idx_0;
  emxEnsureCapacity_real_T(&st, qvec, i, &if_emlrtRTEI);
  qvec_data = qvec->data;
  if (!(obj->PositionNumber >= 0.0)) {
    emlrtNonNegativeCheckR2012b(obj->PositionNumber, &ac_emlrtDCI, &st);
  }
  idx_idx_0 = obj->PositionNumber;
  if (idx_idx_0 != (int32_T)muDoubleScalarFloor(idx_idx_0)) {
    emlrtIntegerCheckR2012b(idx_idx_0, &yb_emlrtDCI, &st);
  }
  loop_ub = (int32_T)idx_idx_0;
  for (i = 0; i < loop_ub; i++) {
    qvec_data[i] = 0.0;
  }
  idx_idx_0 = obj->NumBodies;
  i = (int32_T)idx_idx_0;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, idx_idx_0, mxDOUBLE_CLASS,
                                (int32_T)idx_idx_0, &u_emlrtRTEI, &st);
  emxInit_char_T(&st, &a, &jf_emlrtRTEI);
  for (b_i = 0; b_i < i; b_i++) {
    boolean_T b_bool;
    if (b_i > 5) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, 5, &kc_emlrtBCI, &st);
    }
    body = obj->Bodies[b_i];
    i1 = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    emxEnsureCapacity_char_T(&st, a, i1, &jf_emlrtRTEI);
    a_data = a->data;
    loop_ub = body->JointInternal->Type->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      a_data[i1] = body->JointInternal->Type->data[i1];
    }
    b_bool = false;
    if (a->size[1] == 5) {
      loop_ub = 0;
      int32_T exitg1;
      do {
        exitg1 = 0;
        if (loop_ub < 5) {
          if (a_data[loop_ub] != b_cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (!b_bool) {
      real_T idx_idx_1;
      int32_T idx;
      int32_T j;
      boolean_T exitg2;
      b_st.site = &hg_emlrtRSI;
      if (body->Index == 0.0) {
        c_st.site = &pd_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &c_st, &e_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      jnt = body->JointInternal;
      idx = -2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j <= Q->size[1] - 1)) {
        b_st.site = &ig_emlrtRSI;
        i1 = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = jnt->NameInternal->size[1];
        emxEnsureCapacity_char_T(&b_st, a, i1, &cd_emlrtRTEI);
        a_data = a->data;
        loop_ub = jnt->NameInternal->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          a_data[i1] = jnt->NameInternal->data[i1];
        }
        if (j + 1 > Q->size[1]) {
          emlrtDynamicBoundsCheckR2012b(j + 1, 1, Q->size[1], &nc_emlrtBCI,
                                        &st);
        }
        b_Q.data = (char_T *)&Q_data[j].JointName.data[0];
        b_Q.size = (int32_T *)&Q_data[j].JointName.size[0];
        b_Q.allocatedSize = 200;
        b_Q.numDimensions = 2;
        b_Q.canFreeData = false;
        b_st.site = &ig_emlrtRSI;
        if (b_strcmp(&b_st, &b_Q, a)) {
          idx = j;
          exitg2 = true;
        } else {
          j++;
        }
      }
      if (idx + 1 == -1) {
        b_st.site = &jg_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(&b_st, &d_emlrtRTEI,
                                      "robotics:robotmanip:rigidbodytree:"
                                      "ConfigStructArrayInvalidJointNames",
                                      "robotics:robotmanip:rigidbodytree:"
                                      "ConfigStructArrayInvalidJointNames",
                                      0);
      }
      if ((idx + 1 < 1) || (idx + 1 > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, Q->size[1], &hc_emlrtBCI,
                                      &st);
      }
      idx_idx_0 = jnt->PositionNumber;
      b_st.site = &kg_emlrtRSI;
      c_st.site = &y_emlrtRSI;
      b_bool = muDoubleScalarIsNaN(Q_data[idx].JointPosition.data[0]);
      if (b_bool) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &f_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedNonNaN",
            "MATLAB:validateConfiguration:expectedNonNaN", 3, 4, 5, "input");
      }
      c_st.site = &y_emlrtRSI;
      if (muDoubleScalarIsInf(Q_data[idx].JointPosition.data[0]) || b_bool) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &g_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedFinite",
            "MATLAB:validateConfiguration:expectedFinite", 3, 4, 5, "input");
      }
      c_st.site = &y_emlrtRSI;
      if ((!(muDoubleScalarFloor(idx_idx_0) == idx_idx_0)) ||
          muDoubleScalarIsInf(idx_idx_0) || (!(idx_idx_0 >= 0.0))) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &v_emlrtRTEI, "Coder:toolbox:ValidateattributesBadNumel",
            "MATLAB:validateattributes:badnumel", 0);
      }
      if (!(idx_idx_0 == 1.0)) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &w_emlrtRTEI,
            "Coder:toolbox:ValidateattributesincorrectNumel",
            "MATLAB:validateConfiguration:incorrectNumel", 5, 4, 5, "input", 6,
            idx_idx_0);
      }
      idx_idx_0 = obj->PositionDoFMap[b_i];
      idx_idx_1 = obj->PositionDoFMap[b_i + 6];
      if (idx_idx_0 > idx_idx_1) {
        i1 = 0;
        loop_ub = 0;
      } else {
        if (idx_idx_0 != (int32_T)muDoubleScalarFloor(idx_idx_0)) {
          emlrtIntegerCheckR2012b(idx_idx_0, &wb_emlrtDCI, &st);
        }
        if (((int32_T)idx_idx_0 < 1) || ((int32_T)idx_idx_0 > qvec->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)idx_idx_0, 1, qvec->size[0],
                                        &ic_emlrtBCI, &st);
        }
        i1 = (int32_T)idx_idx_0 - 1;
        if (idx_idx_1 != (int32_T)muDoubleScalarFloor(idx_idx_1)) {
          emlrtIntegerCheckR2012b(idx_idx_1, &xb_emlrtDCI, &st);
        }
        if (((int32_T)idx_idx_1 < 1) || ((int32_T)idx_idx_1 > qvec->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)idx_idx_1, 1, qvec->size[0],
                                        &jc_emlrtBCI, &st);
        }
        loop_ub = (int32_T)idx_idx_1;
      }
      loop_ub -= i1;
      if (loop_ub != 1) {
        emlrtSubAssignSizeCheck1dR2017a(loop_ub, 1, &o_emlrtECI, &st);
      }
      qvec_data[i1] = Q_data[idx].JointPosition.data[0];
    }
  }
  emxFree_char_T(&st, &a);
  emxInit_real_T(sp, &limits, 2, &qf_emlrtRTEI);
  st.site = &bg_emlrtRSI;
  c_RigidBodyTree_get_JointPositi(&st, obj, limits);
  limits_data = limits->data;
  if ((qvec->size[0] != limits->size[0]) &&
      ((qvec->size[0] != 1) && (limits->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(qvec->size[0], limits->size[0], &k_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  emxInit_boolean_T(sp, &ubOK, &kf_emlrtRTEI);
  if (qvec->size[0] == limits->size[0]) {
    i = ubOK->size[0];
    ubOK->size[0] = qvec->size[0];
    emxEnsureCapacity_boolean_T(sp, ubOK, i, &kf_emlrtRTEI);
    ubOK_data = ubOK->data;
    loop_ub = qvec->size[0];
    for (i = 0; i < loop_ub; i++) {
      ubOK_data[i] = (qvec_data[i] <= limits_data[i + limits->size[0]] +
                                          4.4408920985006262E-16);
    }
  } else {
    st.site = &rq_emlrtRSI;
    b_binary_expand_op(&st, ubOK, qvec, limits);
    ubOK_data = ubOK->data;
  }
  if ((qvec->size[0] != limits->size[0]) &&
      ((qvec->size[0] != 1) && (limits->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(qvec->size[0], limits->size[0], &l_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  emxInit_boolean_T(sp, &lbOK, &lf_emlrtRTEI);
  if (qvec->size[0] == limits->size[0]) {
    i = lbOK->size[0];
    lbOK->size[0] = qvec->size[0];
    emxEnsureCapacity_boolean_T(sp, lbOK, i, &lf_emlrtRTEI);
    lbOK_data = lbOK->data;
    loop_ub = qvec->size[0];
    for (i = 0; i < loop_ub; i++) {
      lbOK_data[i] = (qvec_data[i] >= limits_data[i] - 4.4408920985006262E-16);
    }
  } else {
    st.site = &sq_emlrtRSI;
    binary_expand_op(&st, lbOK, qvec, limits);
    lbOK_data = lbOK->data;
  }
  emxInit_int32_T(sp, &indicesUpperBoundViolation, 1, &mf_emlrtRTEI);
  emxInit_int32_T(sp, &ii, 1, &rf_emlrtRTEI);
  st.site = &cg_emlrtRSI;
  if ((!all(&st, ubOK)) || (!all(&st, lbOK))) {
    st.site = &dg_emlrtRSI;
    loop_ub = ubOK->size[0];
    for (i = 0; i < loop_ub; i++) {
      ubOK_data[i] = !ubOK_data[i];
    }
    b_st.site = &ng_emlrtRSI;
    eml_find(&b_st, ubOK, ii);
    ii_data = ii->data;
    i = indicesUpperBoundViolation->size[0];
    indicesUpperBoundViolation->size[0] = ii->size[0];
    emxEnsureCapacity_int32_T(&st, indicesUpperBoundViolation, i,
                              &mf_emlrtRTEI);
    indicesUpperBoundViolation_data = indicesUpperBoundViolation->data;
    loop_ub = ii->size[0];
    for (i = 0; i < loop_ub; i++) {
      indicesUpperBoundViolation_data[i] = ii_data[i];
    }
    i = ii->size[0];
    ii->size[0] = indicesUpperBoundViolation->size[0];
    emxEnsureCapacity_int32_T(sp, ii, i, &nf_emlrtRTEI);
    ii_data = ii->data;
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i = 0; i < loop_ub; i++) {
      if ((indicesUpperBoundViolation_data[i] < 1) ||
          (indicesUpperBoundViolation_data[i] > qvec->size[0])) {
        emlrtDynamicBoundsCheckR2012b(indicesUpperBoundViolation_data[i], 1,
                                      qvec->size[0], &lc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      ii_data[i] = indicesUpperBoundViolation_data[i];
    }
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i = 0; i < loop_ub; i++) {
      i1 = indicesUpperBoundViolation_data[i];
      if ((i1 < 1) || (i1 > limits->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, limits->size[0], &mc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
    }
    if (ii->size[0] != indicesUpperBoundViolation->size[0]) {
      emlrtSubAssignSizeCheck1dR2017a(ii->size[0],
                                      indicesUpperBoundViolation->size[0],
                                      &m_emlrtECI, (emlrtConstCTX)sp);
    }
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i = 0; i < loop_ub; i++) {
      qvec_data[ii_data[i] - 1] =
          limits_data[(indicesUpperBoundViolation_data[i] + limits->size[0]) -
                      1];
    }
    st.site = &eg_emlrtRSI;
    loop_ub = lbOK->size[0];
    for (i = 0; i < loop_ub; i++) {
      lbOK_data[i] = !lbOK_data[i];
    }
    b_st.site = &ng_emlrtRSI;
    eml_find(&b_st, lbOK, ii);
    ii_data = ii->data;
    i = indicesUpperBoundViolation->size[0];
    indicesUpperBoundViolation->size[0] = ii->size[0];
    emxEnsureCapacity_int32_T(&st, indicesUpperBoundViolation, i,
                              &of_emlrtRTEI);
    indicesUpperBoundViolation_data = indicesUpperBoundViolation->data;
    loop_ub = ii->size[0];
    for (i = 0; i < loop_ub; i++) {
      indicesUpperBoundViolation_data[i] = ii_data[i];
    }
    i = ii->size[0];
    ii->size[0] = indicesUpperBoundViolation->size[0];
    emxEnsureCapacity_int32_T(sp, ii, i, &pf_emlrtRTEI);
    ii_data = ii->data;
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i = 0; i < loop_ub; i++) {
      if ((indicesUpperBoundViolation_data[i] < 1) ||
          (indicesUpperBoundViolation_data[i] > qvec->size[0])) {
        emlrtDynamicBoundsCheckR2012b(indicesUpperBoundViolation_data[i], 1,
                                      qvec->size[0], &oc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      ii_data[i] = indicesUpperBoundViolation_data[i];
    }
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i = 0; i < loop_ub; i++) {
      i1 = indicesUpperBoundViolation_data[i];
      if ((i1 < 1) || (i1 > limits->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, limits->size[0], &pc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
    }
    if (ii->size[0] != indicesUpperBoundViolation->size[0]) {
      emlrtSubAssignSizeCheck1dR2017a(ii->size[0],
                                      indicesUpperBoundViolation->size[0],
                                      &n_emlrtECI, (emlrtConstCTX)sp);
    }
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i = 0; i < loop_ub; i++) {
      qvec_data[ii_data[i] - 1] =
          limits_data[indicesUpperBoundViolation_data[i] - 1];
    }
    st.site = &fg_emlrtRSI;
    b_st.site = &ne_emlrtRSI;
    e_warning(&b_st);
  }
  emxFree_int32_T(sp, &ii);
  emxFree_int32_T(sp, &indicesUpperBoundViolation);
  emxFree_boolean_T(sp, &lbOK);
  emxFree_boolean_T(sp, &ubOK);
  emxFree_real_T(sp, &limits);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (RigidBodyTree.c) */
