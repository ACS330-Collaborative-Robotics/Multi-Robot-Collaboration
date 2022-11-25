/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_data.c
 *
 * Code generation for function 'inverse_kinematics_data'
 *
 */

/* Include files */
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                              /* bFirstTime */
    false,                                             /* bInitialized */
    131627U,                                           /* fVersionInfo */
    NULL,                                              /* fErrorFunction */
    "inverse_kinematics",                              /* fFunctionName */
    NULL,                                              /* fRTCallStack */
    false,                                             /* bDebugMode */
    {497771824U, 2490594622U, 369384464U, 266412326U}, /* fSigWrd */
    NULL                                               /* fSigMem */
};

emlrtRSInfo g_emlrtRSI = {
    1,                               /* lineNo */
    "InternalAccess/InternalAccess", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/InternalAccess.m" /* pathName */
};

emlrtRSInfo n_emlrtRSI = {
    71,                    /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

emlrtRSInfo o_emlrtRSI = {
    96,                    /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

emlrtRSInfo p_emlrtRSI = {
    106,                   /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

emlrtRSInfo
    q_emlrtRSI =
        {
            140,                             /* lineNo */
            "rigidBodyJoint/rigidBodyJoint", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

emlrtRSInfo u_emlrtRSI = {
    145,                   /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

emlrtRSInfo v_emlrtRSI = {
    163,                   /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

emlrtRSInfo
    x_emlrtRSI =
        {
            147,                             /* lineNo */
            "rigidBodyJoint/rigidBodyJoint", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

emlrtRSInfo y_emlrtRSI =
    {
        93,                   /* lineNo */
        "validateattributes", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/lang/"
        "validateattributes.m" /* pathName */
};

emlrtRSInfo gb_emlrtRSI = {
    1,                           /* lineNo */
    "SystemCore/parenReference", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/system/coder/+matlab/+system/"
    "+coder/SystemCore.p" /* pathName */
};

emlrtRSInfo qc_emlrtRSI = {
    1333,                                /* lineNo */
    "RigidBodyTree/findBodyIndexByName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

emlrtRSInfo rc_emlrtRSI = {
    1339,                                /* lineNo */
    "RigidBodyTree/findBodyIndexByName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

emlrtRSInfo sc_emlrtRSI = {
    91,       /* lineNo */
    "strcmp", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/strcmp.m" /* pathName
                                                                          */
};

emlrtRSInfo tc_emlrtRSI = {
    167,          /* lineNo */
    "loc_strcmp", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/strcmp.m" /* pathName
                                                                          */
};

emlrtRSInfo vc_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/eml/"
    "eml_int_forloop_overflow_check.m" /* pathName */
};

emlrtRSInfo nd_emlrtRSI = {
    1624,                                  /* lineNo */
    "RigidBodyTree/validateInputBodyName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

emlrtRSInfo od_emlrtRSI = {
    1628,                                  /* lineNo */
    "RigidBodyTree/validateInputBodyName", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

emlrtRSInfo pd_emlrtRSI = {
    422,                   /* lineNo */
    "RigidBody/get.Joint", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

emlrtRSInfo ne_emlrtRSI = {
    21,        /* lineNo */
    "warning", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/warning.m" /* pathName */
};

emlrtRSInfo lf_emlrtRSI = {
    42,                                                            /* lineNo */
    "checkcond",                                                   /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

emlrtRSInfo mf_emlrtRSI = {
    46,                                                            /* lineNo */
    "checkcond",                                                   /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName
                                                                    */
};

emlrtRSInfo mg_emlrtRSI = {
    143,        /* lineNo */
    "allOrAny", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/allOrAny.m" /* pathName
                                                                            */
};

emlrtRSInfo ng_emlrtRSI = {
    39,                                                            /* lineNo */
    "find",                                                        /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/find.m" /* pathName
                                                                    */
};

emlrtRSInfo xg_emlrtRSI = {
    83,                         /* lineNo */
    "SystemTimeProvider/reset", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/SystemTimeProvider.m" /* pathName */
};

emlrtRSInfo yg_emlrtRSI = {
    84,                         /* lineNo */
    "SystemTimeProvider/reset", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/SystemTimeProvider.m" /* pathName */
};

emlrtRSInfo bh_emlrtRSI = {
    7,         /* lineNo */
    "getTime", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "+time/getTime.m" /* pathName */
};

emlrtRSInfo ch_emlrtRSI = {
    21,                     /* lineNo */
    "CoderTimeAPI/getTime", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "+time/CoderTimeAPI.m" /* pathName */
};

emlrtRSInfo dh_emlrtRSI = {
    148,                                  /* lineNo */
    "CoderTimeAPI/callEMLRTClockGettime", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "+time/CoderTimeAPI.m" /* pathName */
};

emlrtRSInfo
    jj_emlrtRSI =
        {
            511,                                    /* lineNo */
            "rigidBodyJoint/transformBodyToParent", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

emlrtRSInfo
    kj_emlrtRSI =
        {
            395,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

emlrtRSInfo
    lj_emlrtRSI =
        {
            396,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

emlrtRSInfo
    mj_emlrtRSI =
        {
            398,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

emlrtRSInfo nj_emlrtRSI =
    {
        23,            /* lineNo */
        "axang2tform", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/"
        "axang2tform.m" /* pathName */
};

emlrtRSInfo oj_emlrtRSI =
    {
        39,           /* lineNo */
        "axang2rotm", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/"
        "axang2rotm.m" /* pathName */
};

emlrtRSInfo ek_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pathName */
};

emlrtRSInfo fk_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pathName */
};

emlrtRSInfo gk_emlrtRSI =
    {
        142,      /* lineNo */
        "mtimes", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+blas/"
        "mtimes.m" /* pathName */
};

emlrtRSInfo hk_emlrtRSI =
    {
        178,           /* lineNo */
        "mtimes_blas", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+blas/"
        "mtimes.m" /* pathName */
};

emlrtRSInfo am_emlrtRSI = {
    64,                                  /* lineNo */
    "SystemTimeProvider/getElapsedTime", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/SystemTimeProvider.m" /* pathName */
};

emlrtRSInfo bm_emlrtRSI = {
    66,                                  /* lineNo */
    "SystemTimeProvider/getElapsedTime", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/SystemTimeProvider.m" /* pathName */
};

emlrtRSInfo rm_emlrtRSI = {
    28,                                                           /* lineNo */
    "colon",                                                      /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

emlrtMCInfo d_emlrtMCI = {
    53,        /* lineNo */
    19,        /* colNo */
    "flt2str", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/flt2str.m" /* pName
                                                                           */
};

emlrtRTEInfo c_emlrtRTEI = {
    13,                 /* lineNo */
    37,                 /* colNo */
    "validatenonempty", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatenonempty.m" /* pName */
};

emlrtRTEInfo d_emlrtRTEI = {
    21,      /* lineNo */
    9,       /* colNo */
    "error", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/error.m" /* pName */
};

emlrtRTEInfo e_emlrtRTEI = {
    28,      /* lineNo */
    9,       /* colNo */
    "error", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/error.m" /* pName */
};

emlrtRTEInfo f_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatenonnan", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatenonnan.m" /* pName */
};

emlrtRTEInfo g_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatefinite", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatefinite.m" /* pName */
};

emlrtRTEInfo h_emlrtRTEI = {
    1338,                                /* lineNo */
    21,                                  /* colNo */
    "RigidBodyTree/findBodyIndexByName", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

emlrtBCInfo j_emlrtBCI = {
    0,                                   /* iFirst */
    5,                                   /* iLast */
    1339,                                /* lineNo */
    38,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/findBodyIndexByName", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

emlrtRTEInfo n_emlrtRTEI = {
    64,                   /* lineNo */
    15,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "assertValidSizeArg.m" /* pName */
};

emlrtRTEInfo o_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "assertValidSizeArg.m" /* pName */
};

emlrtRTEInfo bb_emlrtRTEI = {
    64,                                  /* lineNo */
    13,                                  /* colNo */
    "SystemTimeProvider/getElapsedTime", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/SystemTimeProvider.m" /* pName */
};

emlrtRTEInfo db_emlrtRTEI = {
    159,                    /* lineNo */
    13,                     /* colNo */
    "coderTimeCheckStatus", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/coder/coder/lib/+coder/+internal/"
    "+time/CoderTimeAPI.m" /* pName */
};

emlrtRTEInfo hb_emlrtRTEI = {
    13,                                                            /* lineNo */
    9,                                                             /* colNo */
    "sqrt",                                                        /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elfun/sqrt.m" /* pName */
};

emlrtRTEInfo jb_emlrtRTEI =
    {
        138,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pName */
};

emlrtRTEInfo kb_emlrtRTEI =
    {
        133,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/ops/"
        "eml_mtimes_helper.m" /* pName */
};

emlrtRTEInfo ob_emlrtRTEI = {
    44,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "infocheck.m" /* pName */
};

emlrtRTEInfo pb_emlrtRTEI = {
    47,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "infocheck.m" /* pName */
};

emlrtRTEInfo xb_emlrtRTEI = {
    652,             /* lineNo */
    17,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

emlrtRTEInfo yb_emlrtRTEI = {
    450,             /* lineNo */
    13,              /* colNo */
    "rigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pName
                                                                            */
};

emlrtRTEInfo ic_emlrtRTEI = {
    125,         /* lineNo */
    17,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

emlrtRTEInfo
    jc_emlrtRTEI =
        {
            151,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo
    kc_emlrtRTEI =
        {
            162,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo
    lc_emlrtRTEI =
        {
            168,              /* lineNo */
            20,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo
    mc_emlrtRTEI =
        {
            192,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo
    nc_emlrtRTEI =
        {
            193,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo
    oc_emlrtRTEI =
        {
            194,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo pc_emlrtRTEI = {
    149,         /* lineNo */
    21,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

emlrtRTEInfo qc_emlrtRTEI = {
    152,         /* lineNo */
    21,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

emlrtRTEInfo rc_emlrtRTEI = {
    154,         /* lineNo */
    21,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

emlrtRTEInfo uc_emlrtRTEI = {
    93,          /* lineNo */
    17,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

emlrtRTEInfo yc_emlrtRTEI = {
    443,         /* lineNo */
    13,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

emlrtRTEInfo
    cd_emlrtRTEI =
        {
            236,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

emlrtRTEInfo rf_emlrtRTEI = {
    31,                                                            /* lineNo */
    6,                                                             /* colNo */
    "find",                                                        /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

emlrtRTEInfo pi_emlrtRTEI =
    {
        218,      /* lineNo */
        20,       /* colNo */
        "mtimes", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/+blas/"
        "mtimes.m" /* pName */
};

const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

const char_T cv[9] = {'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c'};

const char_T cv1[26] = {'e', 'm', 'l', 'r', 't', 'C', 'l', 'o', 'c',
                        'k', 'G', 'e', 't', 't', 'i', 'm', 'e', 'M',
                        'o', 'n', 'o', 't', 'o', 'n', 'i', 'c'};

const char_T cv2[19] = {'L', 'A', 'P', 'A', 'C', 'K', 'E', '_', 'd', 'g',
                        'e', 't', 'r', 'f', '_', 'w', 'o', 'r', 'k'};

emlrtRSInfo mq_emlrtRSI = {
    53,        /* lineNo */
    "flt2str", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/flt2str.m" /* pathName
                                                                           */
};

covrtInstance emlrtCoverageInstance;

/* End of code generation (inverse_kinematics_data.c) */
