/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SystemCore.c
 *
 * Code generation for function 'SystemCore'
 *
 */

/* Include files */
#include "SystemCore.h"
#include "RigidBodyTree.h"
#include "eye.h"
#include "inverseKinematics.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ve_emlrtRSI =
    {
        118,                           /* lineNo */
        "inverseKinematics/setupImpl", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo we_emlrtRSI =
    {
        121,                           /* lineNo */
        "inverseKinematics/setupImpl", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo xe_emlrtRSI =
    {
        124,                           /* lineNo */
        "inverseKinematics/setupImpl", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo ye_emlrtRSI =
    {
        508,                                 /* lineNo */
        "inverseKinematics/assembleProblem", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo af_emlrtRSI =
    {
        511,                                 /* lineNo */
        "inverseKinematics/assembleProblem", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo bf_emlrtRSI =
    {
        512,                                 /* lineNo */
        "inverseKinematics/assembleProblem", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo cf_emlrtRSI =
    {
        513,                                 /* lineNo */
        "inverseKinematics/assembleProblem", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo df_emlrtRSI =
    {
        514,                                 /* lineNo */
        "inverseKinematics/assembleProblem", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRSInfo if_emlrtRSI =
    {
        130,                                /* lineNo */
        "inverseKinematics/setupExtraArgs", /* fcnName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pathName */
};

static emlrtRTEInfo p_emlrtRTEI = {
    1,                           /* lineNo */
    1,                           /* colNo */
    "SystemCore/parenReference", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/system/coder/+matlab/+system/"
    "+coder/SystemCore.p" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI =
    {
        507,                                 /* lineNo */
        21,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtBCInfo u_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        513,                                 /* lineNo */
        49,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtBCInfo v_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        514,                                 /* lineNo */
        52,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo l_emlrtDCI =
    {
        511,                                 /* lineNo */
        23,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo w_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        511,                                 /* lineNo */
        23,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo m_emlrtDCI =
    {
        511,                                 /* lineNo */
        25,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo x_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        511,                                 /* lineNo */
        25,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo n_emlrtDCI =
    {
        511,                                 /* lineNo */
        35,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo y_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        511,                                 /* lineNo */
        35,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo o_emlrtDCI =
    {
        511,                                 /* lineNo */
        37,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        511,                                 /* lineNo */
        37,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtECInfo c_emlrtECI =
    {
        -1,                                  /* nDims */
        511,                                 /* lineNo */
        21,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtDCInfo p_emlrtDCI =
    {
        512,                                 /* lineNo */
        23,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        512,                                 /* lineNo */
        23,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo q_emlrtDCI =
    {
        512,                                 /* lineNo */
        25,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        512,                                 /* lineNo */
        25,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo r_emlrtDCI =
    {
        512,                                 /* lineNo */
        35,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo db_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        512,                                 /* lineNo */
        35,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo s_emlrtDCI =
    {
        512,                                 /* lineNo */
        42,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        512,                                 /* lineNo */
        42,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtECInfo d_emlrtECI =
    {
        -1,                                  /* nDims */
        512,                                 /* lineNo */
        21,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtECInfo e_emlrtECI =
    {
        1,                                  /* nDims */
        146,                                /* lineNo */
        13,                                 /* colNo */
        "inverseKinematics/setupExtraArgs", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtDCInfo t_emlrtDCI =
    {
        502,                                 /* lineNo */
        23,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtDCInfo u_emlrtDCI =
    {
        502,                                 /* lineNo */
        23,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        4                      /* checkKind */
};

static emlrtDCInfo v_emlrtDCI =
    {
        502,                                 /* lineNo */
        26,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtDCInfo w_emlrtDCI =
    {
        502,                                 /* lineNo */
        13,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtDCInfo x_emlrtDCI =
    {
        503,                                 /* lineNo */
        13,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI =
    {
        0,                                   /* iFirst */
        5,                                   /* iLast */
        508,                                 /* lineNo */
        58,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo y_emlrtDCI =
    {
        146,                                /* lineNo */
        23,                                 /* colNo */
        "inverseKinematics/setupExtraArgs", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtDCInfo ab_emlrtDCI =
    {
        146,                                /* lineNo */
        23,                                 /* colNo */
        "inverseKinematics/setupExtraArgs", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        4                      /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        513,                                 /* lineNo */
        23,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtDCInfo bb_emlrtDCI =
    {
        513,                                 /* lineNo */
        23,                                  /* colNo */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        1                      /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI =
    {
        -1,                                  /* iFirst */
        -1,                                  /* iLast */
        514,                                 /* lineNo */
        23,                                  /* colNo */
        "",                                  /* aName */
        "inverseKinematics/assembleProblem", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m", /* pName */
        0                      /* checkKind */
};

static emlrtRTEInfo vd_emlrtRTEI =
    {
        502,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo wd_emlrtRTEI =
    {
        503,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo xd_emlrtRTEI =
    {
        520,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo yd_emlrtRTEI =
    {
        521,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo ae_emlrtRTEI =
    {
        510,                 /* lineNo */
        28,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo be_emlrtRTEI =
    {
        142,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo ce_emlrtRTEI =
    {
        146,                 /* lineNo */
        23,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo de_emlrtRTEI =
    {
        147,                 /* lineNo */
        13,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo ee_emlrtRTEI =
    {
        513,                 /* lineNo */
        28,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo fe_emlrtRTEI =
    {
        514,                 /* lineNo */
        31,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

static emlrtRTEInfo ge_emlrtRTEI = {
    1,            /* lineNo */
    1,            /* colNo */
    "SystemCore", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/system/coder/+matlab/+system/"
    "+coder/SystemCore.p" /* pName */
};

static emlrtRTEInfo he_emlrtRTEI =
    {
        512,                 /* lineNo */
        57,                  /* colNo */
        "inverseKinematics", /* fName */
        "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
        "inverseKinematics.m" /* pName */
};

/* Function Definitions */
void SystemCore_step(
    const emlrtStack *sp, inverseKinematics *obj, const real_T varargin_2[16],
    const emxArray_struct_T *varargin_4, emxArray_struct_T *varargout_1,
    real_T *varargout_2_Iterations, real_T *varargout_2_NumRandomRestarts,
    real_T *varargout_2_PoseErrorNorm, real_T *varargout_2_ExitFlag,
    char_T varargout_2_Status_data[], int32_T varargout_2_Status_size[2])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const char_T varargin_1[5] = {'J', 'o', 'i', 'n', 't'};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  emxArray_char_T *a;
  emxArray_real_T *A;
  emxArray_real_T *b;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  rigidBodyJoint *joint;
  real_T *A_data;
  real_T *b_data;
  real_T *r2;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i4;
  char_T *a_data;
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
  if (obj->isInitialized == 2) {
    emlrtErrorWithMessageIdR2018a(
        sp, &p_emlrtRTEI, "MATLAB:system:methodCalledWhenReleasedCodegen",
        "MATLAB:system:methodCalledWhenReleasedCodegen", 3, 4, 4, "step");
  }
  if (obj->isInitialized != 1) {
    real_T d;
    real_T d1;
    real_T m;
    real_T n;
    int32_T loop_ub;
    int8_T b_I[16];
    st.site = &gb_emlrtRSI;
    b_st.site = &gb_emlrtRSI;
    obj->isSetupComplete = false;
    if (obj->isInitialized != 0) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &p_emlrtRTEI,
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen",
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen", 3, 4, 5,
          "setup");
    }
    obj->isInitialized = 1;
    c_st.site = &gb_emlrtRSI;
    d_st.site = &ve_emlrtRSI;
    n = obj->RigidBodyTreeInternal->PositionNumber;
    emxInit_real_T(&d_st, &A, 2, &vd_emlrtRTEI);
    if (!(n >= 0.0)) {
      emlrtNonNegativeCheckR2012b(n, &u_emlrtDCI, &d_st);
    }
    d = (int32_T)muDoubleScalarFloor(n);
    if (n != d) {
      emlrtIntegerCheckR2012b(n, &t_emlrtDCI, &d_st);
    }
    i = A->size[0] * A->size[1];
    A->size[0] = (int32_T)n;
    emxEnsureCapacity_real_T(&d_st, A, i, &vd_emlrtRTEI);
    d1 = 2.0 * n;
    m = (int32_T)muDoubleScalarFloor(d1);
    if (d1 != m) {
      emlrtIntegerCheckR2012b(d1, &v_emlrtDCI, &d_st);
    }
    i = A->size[0] * A->size[1];
    loop_ub = (int32_T)d1;
    A->size[1] = (int32_T)d1;
    emxEnsureCapacity_real_T(&d_st, A, i, &vd_emlrtRTEI);
    A_data = A->data;
    if (n != d) {
      emlrtIntegerCheckR2012b(n, &w_emlrtDCI, &d_st);
    }
    if (d1 != m) {
      emlrtIntegerCheckR2012b(d1, &w_emlrtDCI, &d_st);
    }
    b_loop_ub = (int32_T)n * (int32_T)d1;
    for (i = 0; i < b_loop_ub; i++) {
      A_data[i] = 0.0;
    }
    emxInit_real_T(&d_st, &b, 1, &wd_emlrtRTEI);
    if (d1 != m) {
      emlrtIntegerCheckR2012b(d1, &x_emlrtDCI, &d_st);
    }
    i = b->size[0];
    b->size[0] = (int32_T)d1;
    emxEnsureCapacity_real_T(&d_st, b, i, &wd_emlrtRTEI);
    b_data = b->data;
    if (d1 != m) {
      emlrtIntegerCheckR2012b(d1, &x_emlrtDCI, &d_st);
    }
    for (i = 0; i < loop_ub; i++) {
      b_data[i] = 0.0;
    }
    n = 1.0;
    m = 1.0;
    d = obj->RigidBodyTreeInternal->NumBodies;
    i = (int32_T)d;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                  &q_emlrtRTEI, &d_st);
    emxInit_real_T(&d_st, &r, 2, &ge_emlrtRTEI);
    emxInit_real_T(&d_st, &r1, 2, &he_emlrtRTEI);
    emxInit_char_T(&d_st, &a, &ae_emlrtRTEI);
    for (b_i = 0; b_i < i; b_i++) {
      real_T pnum;
      boolean_T b_bool;
      e_st.site = &ye_emlrtRSI;
      if (b_i > 5) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, 5, &fb_emlrtBCI, &e_st);
      }
      b_obj = obj->RigidBodyTreeInternal->Bodies[b_i];
      if (b_obj->Index == 0.0) {
        f_st.site = &pd_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &f_st, &e_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      joint = b_obj->JointInternal;
      pnum = joint->PositionNumber;
      i1 = a->size[0] * a->size[1];
      a->size[0] = 1;
      a->size[1] = joint->Type->size[1];
      emxEnsureCapacity_char_T(&d_st, a, i1, &ae_emlrtRTEI);
      a_data = a->data;
      loop_ub = joint->Type->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        a_data[i1] = joint->Type->data[i1];
      }
      b_bool = false;
      if (a->size[1] == 5) {
        b_loop_ub = 0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (b_loop_ub < 5) {
            if (a_data[b_loop_ub] != b_cv[b_loop_ub]) {
              exitg1 = 1;
            } else {
              b_loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (!b_bool) {
        int32_T i3;
        d = (n + pnum) - 1.0;
        if (n > d) {
          i1 = 0;
          i2 = 0;
        } else {
          if (n != (int32_T)muDoubleScalarFloor(n)) {
            emlrtIntegerCheckR2012b(n, &l_emlrtDCI, &d_st);
          }
          if (((int32_T)n < 1) || ((int32_T)n > A->size[0])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)n, 1, A->size[0],
                                          &w_emlrtBCI, &d_st);
          }
          i1 = (int32_T)n - 1;
          if (d != (int32_T)muDoubleScalarFloor(d)) {
            emlrtIntegerCheckR2012b(d, &m_emlrtDCI, &d_st);
          }
          if (((int32_T)d < 1) || ((int32_T)d > A->size[0])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, A->size[0],
                                          &x_emlrtBCI, &d_st);
          }
          i2 = (int32_T)d;
        }
        d1 = m + pnum;
        if (m > d1 - 1.0) {
          i3 = 0;
          i4 = 0;
        } else {
          if (m != (int32_T)muDoubleScalarFloor(m)) {
            emlrtIntegerCheckR2012b(m, &n_emlrtDCI, &d_st);
          }
          if (((int32_T)m < 1) || ((int32_T)m > A->size[1])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)m, 1, A->size[1],
                                          &y_emlrtBCI, &d_st);
          }
          i3 = (int32_T)m - 1;
          if (d1 - 1.0 != (int32_T)muDoubleScalarFloor(d1 - 1.0)) {
            emlrtIntegerCheckR2012b(d1 - 1.0, &o_emlrtDCI, &d_st);
          }
          if (((int32_T)(d1 - 1.0) < 1) || ((int32_T)(d1 - 1.0) > A->size[1])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(d1 - 1.0), 1, A->size[1],
                                          &ab_emlrtBCI, &d_st);
          }
          i4 = (int32_T)(d1 - 1.0);
        }
        e_st.site = &af_emlrtRSI;
        eye(&e_st, pnum, r1);
        r2 = r1->data;
        b_iv[0] = i2 - i1;
        b_iv[1] = i4 - i3;
        emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &r1->size[0], 2, &c_emlrtECI,
                                      &d_st);
        loop_ub = r1->size[1];
        for (i2 = 0; i2 < loop_ub; i2++) {
          b_loop_ub = r1->size[0];
          for (i4 = 0; i4 < b_loop_ub; i4++) {
            A_data[(i1 + i4) + A->size[0] * (i3 + i2)] =
                r2[i4 + r1->size[0] * i2];
          }
        }
        if (n > d) {
          i1 = 0;
          i2 = 0;
        } else {
          if (n != (int32_T)muDoubleScalarFloor(n)) {
            emlrtIntegerCheckR2012b(n, &p_emlrtDCI, &d_st);
          }
          if (((int32_T)n < 1) || ((int32_T)n > A->size[0])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)n, 1, A->size[0],
                                          &bb_emlrtBCI, &d_st);
          }
          i1 = (int32_T)n - 1;
          if (d != (int32_T)muDoubleScalarFloor(d)) {
            emlrtIntegerCheckR2012b(d, &q_emlrtDCI, &d_st);
          }
          if (((int32_T)d < 1) || ((int32_T)d > A->size[0])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, A->size[0],
                                          &cb_emlrtBCI, &d_st);
          }
          i2 = (int32_T)d;
        }
        d = m + 2.0 * pnum;
        if (d1 > d - 1.0) {
          i3 = 0;
          i4 = 0;
        } else {
          if (d1 != (int32_T)muDoubleScalarFloor(d1)) {
            emlrtIntegerCheckR2012b(d1, &r_emlrtDCI, &d_st);
          }
          if (((int32_T)d1 < 1) || ((int32_T)d1 > A->size[1])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, A->size[1],
                                          &db_emlrtBCI, &d_st);
          }
          i3 = (int32_T)d1 - 1;
          if (d - 1.0 != (int32_T)muDoubleScalarFloor(d - 1.0)) {
            emlrtIntegerCheckR2012b(d - 1.0, &s_emlrtDCI, &d_st);
          }
          if (((int32_T)(d - 1.0) < 1) || ((int32_T)(d - 1.0) > A->size[1])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(d - 1.0), 1, A->size[1],
                                          &eb_emlrtBCI, &d_st);
          }
          i4 = (int32_T)(d - 1.0);
        }
        e_st.site = &bf_emlrtRSI;
        eye(&e_st, pnum, r1);
        r2 = r1->data;
        loop_ub = r1->size[0] * r1->size[1];
        for (b_loop_ub = 0; b_loop_ub < loop_ub; b_loop_ub++) {
          r2[b_loop_ub] = -r2[b_loop_ub];
        }
        b_iv[0] = i2 - i1;
        b_iv[1] = i4 - i3;
        emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &r1->size[0], 2, &d_emlrtECI,
                                      &d_st);
        loop_ub = r1->size[1];
        for (i2 = 0; i2 < loop_ub; i2++) {
          b_loop_ub = r1->size[0];
          for (i4 = 0; i4 < b_loop_ub; i4++) {
            A_data[(i1 + i4) + A->size[0] * (i3 + i2)] =
                r2[i4 + r1->size[0] * i2];
          }
        }
        e_st.site = &cf_emlrtRSI;
        i1 = r->size[0] * r->size[1];
        r->size[0] = joint->PositionLimitsInternal->size[0];
        r->size[1] = 2;
        emxEnsureCapacity_real_T(&e_st, r, i1, &ee_emlrtRTEI);
        r2 = r->data;
        loop_ub = joint->PositionLimitsInternal->size[0] * 2;
        for (i1 = 0; i1 < loop_ub; i1++) {
          r2[i1] = joint->PositionLimitsInternal->data[i1];
        }
        i1 = r->size[0] << 1;
        if (i1 < 2) {
          emlrtDynamicBoundsCheckR2012b(2, 1, i1, &u_emlrtBCI, &d_st);
        }
        if (m != (int32_T)muDoubleScalarFloor(m)) {
          emlrtIntegerCheckR2012b(m, &bb_emlrtDCI, &d_st);
        }
        if (((int32_T)m < 1) || ((int32_T)m > b->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)m, 1, b->size[0], &gb_emlrtBCI,
                                        &d_st);
        }
        b_data[(int32_T)m - 1] = r2[1];
        e_st.site = &df_emlrtRSI;
        i1 = r->size[0] * r->size[1];
        r->size[0] = joint->PositionLimitsInternal->size[0];
        r->size[1] = 2;
        emxEnsureCapacity_real_T(&e_st, r, i1, &fe_emlrtRTEI);
        r2 = r->data;
        loop_ub = joint->PositionLimitsInternal->size[0] * 2;
        for (i1 = 0; i1 < loop_ub; i1++) {
          r2[i1] = joint->PositionLimitsInternal->data[i1];
        }
        i1 = r->size[0] << 1;
        if (i1 < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, i1, &v_emlrtBCI, &d_st);
        }
        if (((int32_T)((uint32_T)m + 1U) < 1) ||
            ((int32_T)((uint32_T)m + 1U) > b->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)m + 1U), 1,
                                        b->size[0], &hb_emlrtBCI, &d_st);
        }
        b_data[(int32_T)(uint32_T)m] = -r2[0];
        m = d;
      }
      n += pnum;
    }
    emxFree_char_T(&d_st, &a);
    emxFree_real_T(&d_st, &r1);
    emxFree_real_T(&d_st, &r);
    loop_ub = A->size[0] * A->size[1];
    i = obj->Solver->ConstraintMatrix->size[0] *
        obj->Solver->ConstraintMatrix->size[1];
    obj->Solver->ConstraintMatrix->size[0] = A->size[0];
    obj->Solver->ConstraintMatrix->size[1] = A->size[1];
    emxEnsureCapacity_real_T(&d_st, obj->Solver->ConstraintMatrix, i,
                             &xd_emlrtRTEI);
    for (i = 0; i < loop_ub; i++) {
      obj->Solver->ConstraintMatrix->data[i] = A_data[i];
    }
    emxFree_real_T(&d_st, &A);
    i = obj->Solver->ConstraintBound->size[0];
    obj->Solver->ConstraintBound->size[0] = b->size[0];
    emxEnsureCapacity_real_T(&d_st, obj->Solver->ConstraintBound, i,
                             &yd_emlrtRTEI);
    loop_ub = b->size[0];
    for (i = 0; i < loop_ub; i++) {
      obj->Solver->ConstraintBound->data[i] = b_data[i];
    }
    d_st.site = &we_emlrtRSI;
    c_RigidBodyTree_get_JointPositi(&d_st, obj->RigidBodyTreeInternal,
                                    obj->Limits);
    d_st.site = &xe_emlrtRSI;
    e_st.site = &if_emlrtRSI;
    obj->_pobj0.matlabCodegenIsDeleted = false;
    obj->Solver->ExtraArgs = &obj->_pobj0;
    for (i = 0; i < 36; i++) {
      obj->Solver->ExtraArgs->WeightMatrix[i] = 0.0;
    }
    obj->Solver->ExtraArgs->Robot = obj->RigidBodyTreeInternal;
    for (i = 0; i < 16; i++) {
      b_I[i] = 0;
    }
    b_I[0] = 1;
    b_I[5] = 1;
    b_I[10] = 1;
    b_I[15] = 1;
    for (i = 0; i < 16; i++) {
      obj->Solver->ExtraArgs->Tform[i] = b_I[i];
    }
    obj->Solver->ExtraArgs->BodyName->size[0] = 1;
    obj->Solver->ExtraArgs->BodyName->size[1] = 0;
    i = obj->Solver->ExtraArgs->ErrTemp->size[0];
    obj->Solver->ExtraArgs->ErrTemp->size[0] = 6;
    emxEnsureCapacity_real_T(&d_st, obj->Solver->ExtraArgs->ErrTemp, i,
                             &be_emlrtRTEI);
    for (i = 0; i < 6; i++) {
      obj->Solver->ExtraArgs->ErrTemp->data[i] = 0.0;
    }
    obj->Solver->ExtraArgs->CostTemp = 0.0;
    d = obj->RigidBodyTreeInternal->PositionNumber;
    if (!(d >= 0.0)) {
      emlrtNonNegativeCheckR2012b(d, &ab_emlrtDCI, &d_st);
    }
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &y_emlrtDCI, &d_st);
    }
    i = b->size[0];
    b->size[0] = (int32_T)d;
    emxEnsureCapacity_real_T(&d_st, b, i, &ce_emlrtRTEI);
    b_data = b->data;
    d = obj->RigidBodyTreeInternal->PositionNumber;
    if (!(d >= 0.0)) {
      emlrtNonNegativeCheckR2012b(d, &ab_emlrtDCI, &d_st);
    }
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &y_emlrtDCI, &d_st);
    }
    loop_ub = (int32_T)d;
    for (i = 0; i < loop_ub; i++) {
      b_data[i] = 0.0;
    }
    if (b->size[0] > 42) {
      emlrtDimSizeGeqCheckR2012b(42, b->size[0], &e_emlrtECI, &d_st);
    }
    i = obj->Solver->ExtraArgs->GradTemp->size[0];
    obj->Solver->ExtraArgs->GradTemp->size[0] = b->size[0];
    emxEnsureCapacity_real_T(&d_st, obj->Solver->ExtraArgs->GradTemp, i,
                             &de_emlrtRTEI);
    loop_ub = b->size[0];
    emxFree_real_T(&d_st, &b);
    for (i = 0; i < loop_ub; i++) {
      obj->Solver->ExtraArgs->GradTemp->data[i] = 0.0;
    }
    obj->isSetupComplete = true;
  }
  st.site = &gb_emlrtRSI;
  inverseKinematics_stepImpl(
      &st, obj, varargin_2, varargin_4, varargout_1, varargout_2_Iterations,
      varargout_2_NumRandomRestarts, varargout_2_PoseErrorNorm,
      varargout_2_ExitFlag, varargout_2_Status_data, varargout_2_Status_size);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (SystemCore.c) */
