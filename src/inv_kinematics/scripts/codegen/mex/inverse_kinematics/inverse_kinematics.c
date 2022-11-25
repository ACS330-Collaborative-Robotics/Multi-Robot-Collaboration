/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics.c
 *
 * Code generation for function 'inverse_kinematics'
 *
 */

/* Include files */
#include "inverse_kinematics.h"
#include "RigidBodyTree.h"
#include "SystemCore.h"
#include "handle.h"
#include "inverseKinematics.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_internal_types.h"
#include "inverse_kinematics_types.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Type Definitions */
#ifndef struct_emxArray_char_T_1x14
#define struct_emxArray_char_T_1x14
struct emxArray_char_T_1x14 {
  char_T data[14];
  int32_T size[2];
};
#endif /* struct_emxArray_char_T_1x14 */
#ifndef typedef_emxArray_char_T_1x14
#define typedef_emxArray_char_T_1x14
typedef struct emxArray_char_T_1x14 emxArray_char_T_1x14;
#endif /* typedef_emxArray_char_T_1x14 */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  real_T Iterations;
  real_T NumRandomRestarts;
  real_T PoseErrorNorm;
  real_T ExitFlag;
  emxArray_char_T_1x14 Status;
} c_struct_T;
#endif /* typedef_c_struct_T */

/* Variable Definitions */
static emlrtRSInfo
    emlrtRSI =
        {
            50,                   /* lineNo */
            "inverse_kinematics", /* fcnName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pathName */
};

static emlrtRSInfo
    b_emlrtRSI =
        {
            56,                   /* lineNo */
            "inverse_kinematics", /* fcnName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pathName */
};

static emlrtRSInfo
    c_emlrtRSI =
        {
            59,                   /* lineNo */
            "inverse_kinematics", /* fcnName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pathName */
};

static emlrtRSInfo
    d_emlrtRSI =
        {
            73,                   /* lineNo */
            "inverse_kinematics", /* fcnName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pathName */
};

static emlrtRSInfo
    e_emlrtRSI =
        {
            21,                   /* lineNo */
            "inverse_kinematics", /* fcnName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    187,           /* lineNo */
    "importrobot", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/importrobot.m" /* pathName
                                                                          */
};

static emlrtRSInfo qe_emlrtRSI = {
    450,                               /* lineNo */
    "rigidBodyTree/homeConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pathName
                                                                            */
};

static emlrtRSInfo re_emlrtRSI = {
    652,                               /* lineNo */
    "RigidBodyTree/homeConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo se_emlrtRSI = {
    660,                               /* lineNo */
    "RigidBodyTree/homeConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo te_emlrtRSI = {
    661,                               /* lineNo */
    "RigidBodyTree/homeConfiguration", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtMCInfo
    emlrtMCI =
        {
            62,                   /* lineNo */
            1,                    /* colNo */
            "inverse_kinematics", /* fName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pName */
};

static emlrtBCInfo
    emlrtBCI =
        {
            -1,                   /* iFirst */
            -1,                   /* iLast */
            70,                   /* lineNo */
            28,                   /* colNo */
            "configSoln",         /* aName */
            "inverse_kinematics", /* fName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m", /* pName */
            0                       /* checkKind */
};

static emlrtRTEInfo emlrtRTEI = {
    657,                               /* lineNo */
    25,                                /* colNo */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    2,                                 /* nDims */
    660,                               /* lineNo */
    25,                                /* colNo */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    1,                                 /* nDims */
    661,                               /* lineNo */
    25,                                /* colNo */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    660,                               /* lineNo */
    27,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    661,                               /* lineNo */
    27,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    0,                                 /* iFirst */
    5,                                 /* iLast */
    658,                               /* lineNo */
    39,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    660,                               /* lineNo */
    25,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    661,                               /* lineNo */
    25,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/homeConfiguration", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo ac_emlrtRTEI = {
    659,             /* lineNo */
    32,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo bc_emlrtRTEI = {
    660,             /* lineNo */
    42,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo cc_emlrtRTEI = {
    661,             /* lineNo */
    46,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo
    dc_emlrtRTEI =
        {
            1,                    /* lineNo */
            19,                   /* colNo */
            "inverse_kinematics", /* fName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pName */
};

static emlrtRTEInfo
    ec_emlrtRTEI =
        {
            50,                   /* lineNo */
            1,                    /* colNo */
            "inverse_kinematics", /* fName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pName */
};

static emlrtRTEInfo
    fc_emlrtRTEI =
        {
            56,                   /* lineNo */
            1,                    /* colNo */
            "inverse_kinematics", /* fName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pName */
};

static emlrtRTEInfo gc_emlrtRTEI = {
    450,             /* lineNo */
    17,              /* colNo */
    "rigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/rigidBodyTree.m" /* pName
                                                                            */
};

static emlrtRTEInfo hc_emlrtRTEI = {
    652,             /* lineNo */
    21,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRSInfo
    kq_emlrtRSI =
        {
            62,                   /* lineNo */
            "inverse_kinematics", /* fcnName */
            "/home/conor/catkin_ws/src/inv_kinematics/scripts/"
            "inverse_kinematics.m" /* pathName */
};

/* Function Declarations */
static void disp(const emlrtStack *sp, const mxArray *m, emlrtMCInfo *location);

/* Function Definitions */
static void disp(const emlrtStack *sp, const mxArray *m, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = m;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 1, &pArray, "disp", true,
                        location);
}

void inverse_kinematics(const emlrtStack *sp, real_T x, real_T y, real_T z,
                        real_T a, real_T b, real_T c, real_T joints[6])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  b_emxArray_struct_T *Q;
  b_rigidBodyTree mover6;
  b_struct_T *Q_data;
  c_emxArray_struct_T *r;
  c_robotics_manip_internal_Rigid *body;
  c_struct_T varargout_2;
  d_robotics_manip_internal_Rigid lobj_2;
  d_robotics_manip_internal_Rigid *varargin_1;
  d_struct_T *r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_char_T *b_a;
  emxArray_real_T *r2;
  emxArray_struct_T *initialGuess;
  emxArray_struct_T *varargout_1;
  inverseKinematics ik;
  const mxArray *b_y;
  const mxArray *m;
  rigidBodyJoint *obj;
  struct_T *initialGuess_data;
  real_T abc_tform[16];
  real_T c_a_tform_tmp[16];
  real_T d_a_tform_tmp[16];
  real_T a_tform_tmp;
  real_T b_a_tform_tmp;
  real_T b_b_tform_tmp;
  real_T b_c_tform_tmp;
  real_T b_tform_tmp;
  real_T c_tform_tmp;
  real_T *r3;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  uint32_T k;
  char_T *a_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  c_emxInitStruct_robotics_manip_(sp, &lobj_2, &dc_emlrtRTEI);
  emxInitStruct_inverseKinematics(sp, &ik, &ec_emlrtRTEI);
  for (i = 0; i < 13; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_2._pobj0[i],
                                      (void *)&h_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 13; i++) {
    lobj_2._pobj0[i].matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 7; i++) {
    emlrtPushHeapReferenceStackR2021a(
        (emlrtCTX)sp, true, &ik._pobj5._pobj1._pobj1[i],
        (void *)&h_handle_matlabCodegenDestructo, NULL, NULL, NULL);
  }
  for (i = 0; i < 7; i++) {
    ik._pobj5._pobj1._pobj1[i].matlabCodegenIsDeleted = true;
  }
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj5._pobj0,
                                    (void *)&h_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj5._pobj0.matlabCodegenIsDeleted = true;
  for (i = 0; i < 7; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj4._pobj1[i],
                                      (void *)&h_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 7; i++) {
    ik._pobj4._pobj1[i].matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 13; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj3[i],
                                      (void *)&h_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 13; i++) {
    ik._pobj3[i].matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 12; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_2._pobj2[i],
                                      (void *)&b_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 12; i++) {
    lobj_2._pobj2[i].matlabCodegenIsDeleted = true;
  }
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_2.Base,
                                    (void *)&b_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  lobj_2.Base.matlabCodegenIsDeleted = true;
  for (i = 0; i < 6; i++) {
    emlrtPushHeapReferenceStackR2021a(
        (emlrtCTX)sp, true, &ik._pobj5._pobj1._pobj0[i],
        (void *)&b_handle_matlabCodegenDestructo, NULL, NULL, NULL);
  }
  for (i = 0; i < 6; i++) {
    ik._pobj5._pobj1._pobj0[i].matlabCodegenIsDeleted = true;
  }
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj5._pobj1.Base,
                                    (void *)&b_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj5._pobj1.Base.matlabCodegenIsDeleted = true;
  for (i = 0; i < 6; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj4._pobj0[i],
                                      (void *)&b_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 6; i++) {
    ik._pobj4._pobj0[i].matlabCodegenIsDeleted = true;
  }
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj4.Base,
                                    (void *)&b_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj4.Base.matlabCodegenIsDeleted = true;
  for (i = 0; i < 6; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj2[i],
                                      (void *)&b_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 6; i++) {
    ik._pobj2[i].matlabCodegenIsDeleted = true;
  }
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_2,
                                    (void *)&handle_matlabCodegenDestructor,
                                    NULL, NULL, NULL);
  lobj_2.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &mover6,
                                    (void *)&c_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  mover6.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj5._pobj1,
                                    (void *)&g_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj5._pobj1.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj4,
                                    (void *)&g_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj4.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj0,
                                    (void *)&i_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj0.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj6,
                                    (void *)&e_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj6.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik,
                                    (void *)&d_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &ik._pobj5,
                                    (void *)&f_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  ik._pobj5.matlabCodegenIsDeleted = true;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 0U);
  /* INVERSE_KINEMATICS Inverse kinematics of 6DOF Mover6 Manipulator */
  /*    Function loads CPRMover6.urdf then computes the inverse kinematics using
   */
  /*    iterative method. */
  /*  */
  /*    INPUT: */
  /*    x - x coordinate relative to robot */
  /*    y - y coordinate relative to robot */
  /*    z - z coordinate relative to robot */
  /*    a - yaw */
  /*    b - pitch */
  /*    c - roll */
  /*  */
  /*    OUTPUT: */
  /*    Joint positions 1->6 starting from base_link, moving towards link6 */
  /*    j1 j2 j3 j4 j5 j6 */
  /*  */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   */
  /*  Import URDF and setup RigidBodyTree */
  st.site = &e_emlrtRSI;
  b_st.site = &f_emlrtRSI;
  c_st.site = &f_emlrtRSI;
  varargin_1 = RigidBodyTree_RigidBodyTree(&c_st, &lobj_2);
  mover6.TreeInternal = varargin_1;
  mover6.matlabCodegenIsDeleted = false;
  /*  Setup rotation matrices - source:
   * http://msl.cs.uiuc.edu/planning/node102.html */
  a_tform_tmp = muDoubleScalarSin(a);
  b_a_tform_tmp = muDoubleScalarCos(a);
  b_tform_tmp = muDoubleScalarSin(b);
  b_b_tform_tmp = muDoubleScalarCos(b);
  c_tform_tmp = muDoubleScalarSin(c);
  b_c_tform_tmp = muDoubleScalarCos(c);
  c_a_tform_tmp[0] = b_a_tform_tmp;
  c_a_tform_tmp[4] = -a_tform_tmp;
  c_a_tform_tmp[8] = 0.0;
  c_a_tform_tmp[12] = 0.0;
  c_a_tform_tmp[1] = a_tform_tmp;
  c_a_tform_tmp[5] = b_a_tform_tmp;
  c_a_tform_tmp[9] = 0.0;
  c_a_tform_tmp[13] = 0.0;
  abc_tform[0] = b_b_tform_tmp;
  abc_tform[4] = 0.0;
  abc_tform[8] = b_tform_tmp;
  abc_tform[12] = 0.0;
  abc_tform[2] = -b_tform_tmp;
  abc_tform[6] = 0.0;
  abc_tform[10] = b_b_tform_tmp;
  abc_tform[14] = 0.0;
  c_a_tform_tmp[2] = 0.0;
  c_a_tform_tmp[3] = 0.0;
  abc_tform[1] = 0.0;
  abc_tform[3] = 0.0;
  c_a_tform_tmp[6] = 0.0;
  c_a_tform_tmp[7] = 0.0;
  abc_tform[5] = 1.0;
  abc_tform[7] = 0.0;
  c_a_tform_tmp[10] = 1.0;
  c_a_tform_tmp[11] = 0.0;
  abc_tform[9] = 0.0;
  abc_tform[11] = 0.0;
  c_a_tform_tmp[14] = 0.0;
  c_a_tform_tmp[15] = 1.0;
  abc_tform[13] = 0.0;
  abc_tform[15] = 1.0;
  for (i = 0; i < 4; i++) {
    a_tform_tmp = c_a_tform_tmp[i];
    b_a_tform_tmp = c_a_tform_tmp[i + 4];
    b_tform_tmp = c_a_tform_tmp[i + 8];
    b_b_tform_tmp = c_a_tform_tmp[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      b_i = i1 << 2;
      d_a_tform_tmp[i + b_i] =
          ((a_tform_tmp * abc_tform[b_i] + b_a_tform_tmp * abc_tform[b_i + 1]) +
           b_tform_tmp * abc_tform[b_i + 2]) +
          b_b_tform_tmp * abc_tform[b_i + 3];
    }
  }
  c_a_tform_tmp[1] = 0.0;
  c_a_tform_tmp[5] = b_c_tform_tmp;
  c_a_tform_tmp[9] = -c_tform_tmp;
  c_a_tform_tmp[13] = 0.0;
  c_a_tform_tmp[2] = 0.0;
  c_a_tform_tmp[6] = c_tform_tmp;
  c_a_tform_tmp[10] = b_c_tform_tmp;
  c_a_tform_tmp[14] = 0.0;
  c_a_tform_tmp[0] = 1.0;
  c_a_tform_tmp[3] = 0.0;
  c_a_tform_tmp[4] = 0.0;
  c_a_tform_tmp[7] = 0.0;
  c_a_tform_tmp[8] = 0.0;
  c_a_tform_tmp[11] = 0.0;
  c_a_tform_tmp[12] = 0.0;
  c_a_tform_tmp[15] = 1.0;
  for (i = 0; i < 4; i++) {
    a_tform_tmp = d_a_tform_tmp[i];
    b_a_tform_tmp = d_a_tform_tmp[i + 4];
    b_tform_tmp = d_a_tform_tmp[i + 8];
    b_b_tform_tmp = d_a_tform_tmp[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      b_i = i1 << 2;
      abc_tform[i + b_i] = ((a_tform_tmp * c_a_tform_tmp[b_i] +
                             b_a_tform_tmp * c_a_tform_tmp[b_i + 1]) +
                            b_tform_tmp * c_a_tform_tmp[b_i + 2]) +
                           b_b_tform_tmp * c_a_tform_tmp[b_i + 3];
    }
  }
  /*  Add xyz transform to rotation matrix */
  abc_tform[12] = x;
  abc_tform[13] = y;
  abc_tform[14] = z;
  /*  Final 4x4 transform from base_link to link6 */
  /*  Initiate ik object */
  st.site = &emlrtRSI;
  c_inverseKinematics_inverseKine(&st, &ik, &mover6);
  /*  First three are orientation, Second three are xyz */
  /*  Start with initial guess of robot position */
  st.site = &b_emlrtRSI;
  emxInit_struct_T1(&st, &Q, &gc_emlrtRTEI);
  b_st.site = &qe_emlrtRSI;
  varargin_1 = mover6.TreeInternal;
  emxInit_struct_T2(&b_st, &r, &hc_emlrtRTEI);
  c_st.site = &re_emlrtRSI;
  repmat(&c_st, varargin_1->NumNonFixedBodies, r);
  r1 = r->data;
  i = Q->size[0] * Q->size[1];
  Q->size[0] = 1;
  Q->size[1] = r->size[1];
  emxEnsureCapacity_struct_T(&b_st, Q, i, &xb_emlrtRTEI);
  Q_data = Q->data;
  i = r->size[1] - 1;
  for (i1 = 0; i1 <= i; i1++) {
    Q_data[i1].JointName.size[0] = 1;
    Q_data[i1].JointName.size[1] = 0;
    Q_data[i1].JointPosition.size[0] = 1;
    Q_data[i1].JointPosition.size[1] = 1;
    Q_data[i1].JointPosition.data[0] = r1[i1].JointPosition;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  }
  emxFree_struct_T2(&b_st, &r);
  k = 1U;
  a_tform_tmp = varargin_1->NumBodies;
  i = (int32_T)a_tform_tmp;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, a_tform_tmp, mxDOUBLE_CLASS,
                                (int32_T)a_tform_tmp, &emlrtRTEI, &b_st);
  emxInit_real_T(&b_st, &r2, 1, &dc_emlrtRTEI);
  emxInit_char_T(&b_st, &b_a, &ac_emlrtRTEI);
  for (b_i = 0; b_i < i; b_i++) {
    boolean_T b_bool;
    if (b_i > 5) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, 5, &d_emlrtBCI, &b_st);
    }
    body = varargin_1->Bodies[b_i];
    i1 = b_a->size[0] * b_a->size[1];
    b_a->size[0] = 1;
    b_a->size[1] = body->JointInternal->Type->size[1];
    emxEnsureCapacity_char_T(&b_st, b_a, i1, &ac_emlrtRTEI);
    a_data = b_a->data;
    loop_ub = body->JointInternal->Type->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      a_data[i1] = body->JointInternal->Type->data[i1];
    }
    b_bool = false;
    if (b_a->size[1] == 5) {
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
      c_st.site = &se_emlrtRSI;
      obj = body->JointInternal;
      i1 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 1;
      b_a->size[1] = obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(&c_st, b_a, i1, &bc_emlrtRTEI);
      a_data = b_a->data;
      loop_ub = obj->NameInternal->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        a_data[i1] = obj->NameInternal->data[i1];
      }
      if (b_a->size[1] > 200) {
        emlrtDimSizeGeqCheckR2012b(200, b_a->size[1], &emlrtECI, &b_st);
      }
      if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &b_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)k - 1].JointName.size[0] = 1;
      if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &b_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)k - 1].JointName.size[1] = b_a->size[1];
      if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &b_emlrtBCI,
                                      &b_st);
      }
      loop_ub = b_a->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &e_emlrtBCI,
                                        &b_st);
        }
        Q_data[(int32_T)k - 1].JointName.data[i1] = a_data[i1];
      }
      c_st.site = &te_emlrtRSI;
      obj = body->JointInternal;
      i1 = r2->size[0];
      r2->size[0] = obj->HomePositionInternal->size[0];
      emxEnsureCapacity_real_T(&c_st, r2, i1, &cc_emlrtRTEI);
      r3 = r2->data;
      loop_ub = obj->HomePositionInternal->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        r3[i1] = obj->HomePositionInternal->data[i1];
      }
      if (r2->size[0] != 1) {
        emlrtDimSizeEqCheckR2012b(1, r2->size[0], &b_emlrtECI, &b_st);
      }
      if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &c_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)k - 1].JointPosition.size[0] = 1;
      if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &c_emlrtBCI,
                                      &b_st);
      }
      Q_data[(int32_T)k - 1].JointPosition.size[1] = 1;
      if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &c_emlrtBCI,
                                      &b_st);
      }
      loop_ub = r2->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        if (((int32_T)k < 1) || ((int32_T)k > Q->size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, Q->size[1], &f_emlrtBCI,
                                        &b_st);
        }
        Q_data[(int32_T)k - 1].JointPosition.data[i1] = r3[i1];
      }
      k++;
    }
  }
  emxFree_char_T(&b_st, &b_a);
  emxFree_real_T(&b_st, &r2);
  emxInit_struct_T(&st, &initialGuess, &fc_emlrtRTEI);
  i = initialGuess->size[0] * initialGuess->size[1];
  initialGuess->size[0] = 1;
  initialGuess->size[1] = Q->size[1];
  emxEnsureCapacity_struct_T1(&st, initialGuess, i, &yb_emlrtRTEI);
  initialGuess_data = initialGuess->data;
  i = Q->size[1] - 1;
  for (i1 = 0; i1 <= i; i1++) {
    initialGuess_data[i1].JointName.size[0] = 1;
    initialGuess_data[i1].JointName.size[1] = Q_data[i1].JointName.size[1];
    loop_ub = Q_data[i1].JointName.size[1];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      initialGuess_data[i1].JointName.data[b_i] =
          Q_data[i1].JointName.data[b_i];
    }
    initialGuess_data[i1].JointPosition.size[0] = 1;
    initialGuess_data[i1].JointPosition.size[1] = 1;
    initialGuess_data[i1].JointPosition.data[0] =
        Q_data[i1].JointPosition.data[0];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  emxFree_struct_T1(&st, &Q);
  /*  Iteratively improve robot position until it reaches optimal position */
  st.site = &c_emlrtRSI;
  emxInit_struct_T(&st, &varargout_1, &dc_emlrtRTEI);
  b_st.site = &gb_emlrtRSI;
  SystemCore_step(&b_st, &ik, abc_tform, initialGuess, varargout_1,
                  &varargout_2.Iterations, &varargout_2.NumRandomRestarts,
                  &varargout_2.PoseErrorNorm, &varargout_2.ExitFlag,
                  varargout_2.Status.data, varargout_2.Status.size);
  initialGuess_data = varargout_1->data;
  emxFree_struct_T(&st, &initialGuess);
  /*  Display ik status - "best available" or "success" */
  b_y = NULL;
  b_iv[0] = 1;
  b_iv[1] = varargout_2.Status.size[1];
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, varargout_2.Status.size[1], m,
                           &varargout_2.Status.data[0]);
  emlrtAssign(&b_y, m);
  st.site = &kq_emlrtRSI;
  disp(&st, b_y, &emlrtMCI);
  /*  Display plot of robot position */
  /* show(mover6, configSoln) */
  /*  Output final joint positions */
  for (b_i = 0; b_i < 6; b_i++) {
    covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 1U);
    if (b_i + 1 > varargout_1->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, varargout_1->size[1], &emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    joints[b_i] = initialGuess_data[b_i].JointPosition.data[0];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  emxFree_struct_T(sp, &varargout_1);
  covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 0);
  /*  for */
  st.site = &d_emlrtRSI;
  f_handle_matlabCodegenDestructo(&st, &ik._pobj5);
  st.site = &d_emlrtRSI;
  d_handle_matlabCodegenDestructo(&st, &ik);
  st.site = &d_emlrtRSI;
  e_handle_matlabCodegenDestructo(&st, &ik._pobj6);
  st.site = &d_emlrtRSI;
  i_handle_matlabCodegenDestructo(&st, &ik._pobj0);
  st.site = &d_emlrtRSI;
  g_handle_matlabCodegenDestructo(&st, &ik._pobj4);
  st.site = &d_emlrtRSI;
  g_handle_matlabCodegenDestructo(&st, &ik._pobj5._pobj1);
  st.site = &d_emlrtRSI;
  c_handle_matlabCodegenDestructo(&st, &mover6);
  st.site = &d_emlrtRSI;
  handle_matlabCodegenDestructor(&st, &lobj_2);
  for (i = 0; i < 6; i++) {
    st.site = &d_emlrtRSI;
    b_handle_matlabCodegenDestructo(&st, &ik._pobj2[i]);
  }
  st.site = &d_emlrtRSI;
  b_handle_matlabCodegenDestructo(&st, &ik._pobj4.Base);
  for (i = 0; i < 6; i++) {
    st.site = &d_emlrtRSI;
    b_handle_matlabCodegenDestructo(&st, &ik._pobj4._pobj0[i]);
  }
  st.site = &d_emlrtRSI;
  b_handle_matlabCodegenDestructo(&st, &ik._pobj5._pobj1.Base);
  for (i = 0; i < 6; i++) {
    st.site = &d_emlrtRSI;
    b_handle_matlabCodegenDestructo(&st, &ik._pobj5._pobj1._pobj0[i]);
  }
  st.site = &d_emlrtRSI;
  b_handle_matlabCodegenDestructo(&st, &lobj_2.Base);
  for (i = 0; i < 12; i++) {
    st.site = &d_emlrtRSI;
    b_handle_matlabCodegenDestructo(&st, &lobj_2._pobj2[i]);
  }
  for (i = 0; i < 13; i++) {
    st.site = &d_emlrtRSI;
    h_handle_matlabCodegenDestructo(&st, &ik._pobj3[i]);
  }
  for (i = 0; i < 7; i++) {
    st.site = &d_emlrtRSI;
    h_handle_matlabCodegenDestructo(&st, &ik._pobj4._pobj1[i]);
  }
  st.site = &d_emlrtRSI;
  h_handle_matlabCodegenDestructo(&st, &ik._pobj5._pobj0);
  for (i = 0; i < 7; i++) {
    st.site = &d_emlrtRSI;
    h_handle_matlabCodegenDestructo(&st, &ik._pobj5._pobj1._pobj1[i]);
  }
  for (i = 0; i < 13; i++) {
    st.site = &d_emlrtRSI;
    h_handle_matlabCodegenDestructo(&st, &lobj_2._pobj0[i]);
  }
  emxFreeStruct_inverseKinematics(sp, &ik);
  d_emxFreeStruct_robotics_manip_(sp, &lobj_2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (inverse_kinematics.c) */
