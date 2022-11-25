/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * NLPSolverInterface.c
 *
 * Code generation for function 'NLPSolverInterface'
 *
 */

/* Include files */
#include "NLPSolverInterface.h"
#include "DampedBFGSwGradientProjection.h"
#include "abs.h"
#include "all.h"
#include "any.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_internal_types.h"
#include "inverse_kinematics_types.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "emlrt.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo sg_emlrtRSI = {
    108,                        /* lineNo */
    "NLPSolverInterface/solve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pathName */
};

static emlrtRSInfo tg_emlrtRSI = {
    111,                        /* lineNo */
    "NLPSolverInterface/solve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pathName */
};

static emlrtRSInfo ug_emlrtRSI = {
    123,                        /* lineNo */
    "NLPSolverInterface/solve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pathName */
};

static emlrtRSInfo vg_emlrtRSI = {
    142,                        /* lineNo */
    "NLPSolverInterface/solve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pathName */
};

static emlrtRSInfo wg_emlrtRSI = {
    144,                        /* lineNo */
    "NLPSolverInterface/solve", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pathName */
};

static emlrtRSInfo lp_emlrtRSI = {
    48,                       /* lineNo */
    "IKHelpers/randomConfig", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/IKHelpers.m" /* pathName */
};

static emlrtRSInfo mp_emlrtRSI = {
    641,                                  /* lineNo */
    "RigidBodyTree/randomJointPositions", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo
    np_emlrtRSI =
        {
            523,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    op_emlrtRSI =
        {
            524,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    pp_emlrtRSI =
        {
            526,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    qp_emlrtRSI =
        {
            527,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    rp_emlrtRSI =
        {
            528,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    sp_emlrtRSI =
        {
            529,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    tp_emlrtRSI =
        {
            531,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtECInfo p_emlrtECI = {
    -1,                         /* nDims */
    143,                        /* lineNo */
    17,                         /* colNo */
    "NLPSolverInterface/solve", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = {
    638,                                  /* lineNo */
    21,                                   /* colNo */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtBCInfo qc_emlrtBCI = {
    1,                                    /* iFirst */
    6,                                    /* iLast */
    639,                                  /* lineNo */
    40,                                   /* colNo */
    "",                                   /* aName */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo bc_emlrtDCI = {
    642,                                  /* lineNo */
    23,                                   /* colNo */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo rc_emlrtBCI = {
    -1,                                   /* iFirst */
    -1,                                   /* iLast */
    642,                                  /* lineNo */
    23,                                   /* colNo */
    "",                                   /* aName */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo cc_emlrtDCI = {
    642,                                  /* lineNo */
    28,                                   /* colNo */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo sc_emlrtBCI = {
    -1,                                   /* iFirst */
    -1,                                   /* iLast */
    642,                                  /* lineNo */
    28,                                   /* colNo */
    "",                                   /* aName */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo q_emlrtECI = {
    -1,                                   /* nDims */
    642,                                  /* lineNo */
    21,                                   /* colNo */
    "RigidBodyTree/randomJointPositions", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtECInfo
    r_emlrtECI =
        {
            1,                               /* nDims */
            525,                             /* lineNo */
            40,                              /* colNo */
            "rigidBodyJoint/randomPosition", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtECInfo
    s_emlrtECI =
        {
            1,                               /* nDims */
            525,                             /* lineNo */
            35,                              /* colNo */
            "rigidBodyJoint/randomPosition", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtECInfo
    t_emlrtECI =
        {
            1,                               /* nDims */
            525,                             /* lineNo */
            30,                              /* colNo */
            "rigidBodyJoint/randomPosition", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtECInfo
    u_emlrtECI =
        {
            1,                               /* nDims */
            527,                             /* lineNo */
            30,                              /* colNo */
            "rigidBodyJoint/randomPosition", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtECInfo
    v_emlrtECI =
        {
            1,                               /* nDims */
            529,                             /* lineNo */
            30,                              /* colNo */
            "rigidBodyJoint/randomPosition", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtDCInfo
    dc_emlrtDCI =
        {
            517,                             /* lineNo */
            13,                              /* colNo */
            "rigidBodyJoint/randomPosition", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m", /* pName */
            1                   /* checkKind */
};

static emlrtDCInfo ec_emlrtDCI = {
    142,                        /* lineNo */
    17,                         /* colNo */
    "NLPSolverInterface/solve", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m", /* pName */
    1                                       /* checkKind */
};

static emlrtDCInfo fc_emlrtDCI = {
    142,                        /* lineNo */
    17,                         /* colNo */
    "NLPSolverInterface/solve", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m", /* pName */
    4                                       /* checkKind */
};

static emlrtDCInfo gc_emlrtDCI = {
    103,    /* lineNo */
    30,     /* colNo */
    "rand", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/randfun/rand.m", /* pName
                                                                       */
    4 /* checkKind */
};

static emlrtRTEInfo uf_emlrtRTEI = {
    104,                  /* lineNo */
    13,                   /* colNo */
    "NLPSolverInterface", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pName */
};

static emlrtRTEInfo vf_emlrtRTEI = {
    142,                  /* lineNo */
    17,                   /* colNo */
    "NLPSolverInterface", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pName */
};

static emlrtRTEInfo wf_emlrtRTEI = {
    143,                  /* lineNo */
    17,                   /* colNo */
    "NLPSolverInterface", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pName */
};

static emlrtRTEInfo
    xf_emlrtRTEI =
        {
            521,              /* lineNo */
            21,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo yf_emlrtRTEI = {
    641,             /* lineNo */
    21,              /* colNo */
    "RigidBodyTree", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo ag_emlrtRTEI = {
    148,                  /* lineNo */
    21,                   /* colNo */
    "NLPSolverInterface", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/shared/robotics/robotutils/+robotics/"
    "+core/+internal/NLPSolverInterface.m" /* pName */
};

static emlrtRTEInfo
    bg_emlrtRTEI =
        {
            522,              /* lineNo */
            21,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo cg_emlrtRTEI = {
    16,                                                             /* lineNo */
    13,                                                             /* colNo */
    "isinf",                                                        /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/isinf.m" /* pName */
};

static emlrtRTEInfo dg_emlrtRTEI = {
    16,                                                             /* lineNo */
    13,                                                             /* colNo */
    "isnan",                                                        /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/elmat/isnan.m" /* pName */
};

static emlrtRTEInfo eg_emlrtRTEI = {
    103,    /* lineNo */
    24,     /* colNo */
    "rand", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/randfun/rand.m" /* pName */
};

static emlrtRTEInfo fg_emlrtRTEI = {
    115,     /* lineNo */
    24,      /* colNo */
    "randn", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/lib/matlab/randfun/randn.m" /* pName
                                                                       */
};

static emlrtRTEInfo
    gg_emlrtRTEI =
        {
            524,              /* lineNo */
            25,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    hg_emlrtRTEI =
        {
            529,              /* lineNo */
            39,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRSInfo
    nq_emlrtRSI =
        {
            525,                             /* lineNo */
            "rigidBodyJoint/randomPosition", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

/* Function Definitions */
void NLPSolverInterface_solve(
    const emlrtStack *sp, c_robotics_core_internal_Damped *obj,
    const emxArray_real_T *seed, emxArray_real_T *xSol,
    real_T *solutionInfo_Iterations, real_T *solutionInfo_RRAttempts,
    real_T *solutionInfo_Error, real_T *solutionInfo_ExitFlag,
    char_T solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2])
{
  static const char_T b_cv[14] = {'b', 'e', 's', 't', ' ', 'a', 'v',
                                  'a', 'i', 'l', 'a', 'b', 'l', 'e'};
  static const char_T b_cv1[7] = {'s', 'u', 'c', 'c', 'e', 's', 's'};
  c_robotics_manip_internal_IKExt *args;
  e_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_boolean_T *b_r;
  emxArray_boolean_T *r1;
  emxArray_real_T *lb;
  emxArray_real_T *newseed;
  emxArray_real_T *r;
  emxArray_real_T *rn;
  emxArray_real_T *ub;
  rigidBodyJoint *c_obj;
  const real_T *seed_data;
  real_T err;
  real_T errPrev;
  real_T iter;
  real_T iterations;
  real_T rrAttempts;
  real_T tol;
  real_T *lb_data;
  real_T *newseed_data;
  real_T *rn_data;
  real_T *ub_data;
  real_T *xSol_data;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;
  boolean_T exitg1;
  boolean_T *r2;
  boolean_T *r3;
  c_robotics_core_internal_NLPSol exitFlag;
  c_robotics_core_internal_NLPSol exitFlagPrev;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  seed_data = seed->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  i = obj->SeedInternal->size[0];
  obj->SeedInternal->size[0] = seed->size[0];
  emxEnsureCapacity_real_T(sp, obj->SeedInternal, i, &uf_emlrtRTEI);
  loop_ub = seed->size[0];
  for (i = 0; i < loop_ub; i++) {
    obj->SeedInternal->data[i] = seed_data[i];
  }
  tol = obj->SolutionTolerance;
  st.site = &sg_emlrtRSI;
  b_st.site = &xg_emlrtRSI;
  obj->TimeObj.StartTime = tic(&b_st);
  b_st.site = &yg_emlrtRSI;
  st.site = &tg_emlrtRSI;
  c_DampedBFGSwGradientProjection(&st, obj, xSol, &exitFlag, &err, &iter);
  rrAttempts = 0.0;
  iterations = iter;
  errPrev = err;
  exitFlagPrev = exitFlag;
  emxInit_real_T(sp, &newseed, 1, &vf_emlrtRTEI);
  emxInit_real_T(sp, &ub, 1, &xf_emlrtRTEI);
  emxInit_real_T(sp, &lb, 1, &bg_emlrtRTEI);
  emxInit_real_T(sp, &rn, 1, &gg_emlrtRTEI);
  emxInit_real_T(sp, &r, 1, &hg_emlrtRTEI);
  emxInit_boolean_T(sp, &b_r, &cg_emlrtRTEI);
  emxInit_boolean_T(sp, &r1, &dg_emlrtRTEI);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (err > tol))) {
    boolean_T valid;
    obj->MaxNumIterationInternal -= iter;
    st.site = &ug_emlrtRSI;
    b_st.site = &am_emlrtRSI;
    valid = (obj->TimeObj.StartTime.tv_sec > 0.0);
    if (!valid) {
      emlrtErrorWithMessageIdR2018a(
          &st, &bb_emlrtRTEI,
          "shared_robotics:robotutils:timeprovider:TimeProviderNotInitialized",
          "shared_robotics:robotutils:timeprovider:TimeProviderNotInitialized",
          0);
    }
    b_st.site = &bm_emlrtRSI;
    err = toc(&b_st, obj->TimeObj.StartTime.tv_sec,
              obj->TimeObj.StartTime.tv_nsec);
    obj->MaxTimeInternal = obj->MaxTime - err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      exitFlag = IterationLimitExceeded;
    }
    if ((exitFlag == IterationLimitExceeded) ||
        (exitFlag == TimeLimitExceeded)) {
      exitFlagPrev = exitFlag;
      exitg1 = true;
    } else {
      st.site = &vg_emlrtRSI;
      args = obj->ExtraArgs;
      b_st.site = &lp_emlrtRSI;
      b_obj = args->Robot;
      if (!(b_obj->PositionNumber >= 0.0)) {
        emlrtNonNegativeCheckR2012b(b_obj->PositionNumber, &fc_emlrtDCI, &b_st);
      }
      err = b_obj->PositionNumber;
      if (err != (int32_T)muDoubleScalarFloor(err)) {
        emlrtIntegerCheckR2012b(err, &ec_emlrtDCI, &b_st);
      }
      i = newseed->size[0];
      newseed->size[0] = (int32_T)err;
      emxEnsureCapacity_real_T(&b_st, newseed, i, &vf_emlrtRTEI);
      newseed_data = newseed->data;
      if (!(b_obj->PositionNumber >= 0.0)) {
        emlrtNonNegativeCheckR2012b(b_obj->PositionNumber, &fc_emlrtDCI, &b_st);
      }
      err = b_obj->PositionNumber;
      if (err != (int32_T)muDoubleScalarFloor(err)) {
        emlrtIntegerCheckR2012b(err, &ec_emlrtDCI, &b_st);
      }
      loop_ub = (int32_T)err;
      for (i = 0; i < loop_ub; i++) {
        newseed_data[i] = 0.0;
      }
      err = b_obj->NumBodies;
      i = (int32_T)err;
      emlrtForLoopVectorCheckR2021a(1.0, 1.0, err, mxDOUBLE_CLASS, (int32_T)err,
                                    &cb_emlrtRTEI, &b_st);
      for (b_i = 0; b_i < i; b_i++) {
        if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
            ((int32_T)((uint32_T)b_i + 1U) > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, 6,
                                        &qc_emlrtBCI, &b_st);
        }
        err = b_obj->PositionDoFMap[b_i];
        iter = b_obj->PositionDoFMap[b_i + 6];
        if (err <= iter) {
          c_st.site = &mp_emlrtRSI;
          c_obj = b_obj->Bodies[b_i]->JointInternal;
          if (c_obj->PositionNumber !=
              (int32_T)muDoubleScalarFloor(c_obj->PositionNumber)) {
            emlrtIntegerCheckR2012b(c_obj->PositionNumber, &dc_emlrtDCI, &c_st);
          }
          if ((int32_T)c_obj->PositionNumber == 0) {
            i1 = lb->size[0];
            lb->size[0] = 1;
            emxEnsureCapacity_real_T(&c_st, lb, i1, &yf_emlrtRTEI);
            lb_data = lb->data;
            lb_data[0] = rtNaN;
          } else {
            boolean_T guard1 = false;
            boolean_T guard2 = false;
            boolean_T guard3 = false;
            i1 = ub->size[0];
            ub->size[0] = c_obj->PositionLimitsInternal->size[0];
            emxEnsureCapacity_real_T(&c_st, ub, i1, &xf_emlrtRTEI);
            ub_data = ub->data;
            loop_ub = c_obj->PositionLimitsInternal->size[0];
            for (i1 = 0; i1 < loop_ub; i1++) {
              ub_data[i1] =
                  c_obj->PositionLimitsInternal
                      ->data[i1 + c_obj->PositionLimitsInternal->size[0]];
            }
            i1 = lb->size[0];
            lb->size[0] = c_obj->PositionLimitsInternal->size[0];
            emxEnsureCapacity_real_T(&c_st, lb, i1, &bg_emlrtRTEI);
            lb_data = lb->data;
            loop_ub = c_obj->PositionLimitsInternal->size[0];
            for (i1 = 0; i1 < loop_ub; i1++) {
              lb_data[i1] = c_obj->PositionLimitsInternal->data[i1];
            }
            i1 = b_r->size[0];
            b_r->size[0] = lb->size[0];
            emxEnsureCapacity_boolean_T(&c_st, b_r, i1, &cg_emlrtRTEI);
            r2 = b_r->data;
            loop_ub = lb->size[0];
            for (i1 = 0; i1 < loop_ub; i1++) {
              r2[i1] = muDoubleScalarIsInf(lb_data[i1]);
            }
            i1 = r1->size[0];
            r1->size[0] = lb->size[0];
            emxEnsureCapacity_boolean_T(&c_st, r1, i1, &dg_emlrtRTEI);
            r3 = r1->data;
            loop_ub = lb->size[0];
            for (i1 = 0; i1 < loop_ub; i1++) {
              r3[i1] = muDoubleScalarIsNaN(lb_data[i1]);
            }
            loop_ub = b_r->size[0];
            for (i1 = 0; i1 < loop_ub; i1++) {
              r2[i1] = ((!r2[i1]) && (!r3[i1]));
            }
            guard1 = false;
            guard2 = false;
            guard3 = false;
            d_st.site = &np_emlrtRSI;
            if (all(&d_st, b_r)) {
              i1 = b_r->size[0];
              b_r->size[0] = ub->size[0];
              emxEnsureCapacity_boolean_T(&c_st, b_r, i1, &cg_emlrtRTEI);
              r2 = b_r->data;
              loop_ub = ub->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r2[i1] = muDoubleScalarIsInf(ub_data[i1]);
              }
              i1 = r1->size[0];
              r1->size[0] = ub->size[0];
              emxEnsureCapacity_boolean_T(&c_st, r1, i1, &dg_emlrtRTEI);
              r3 = r1->data;
              loop_ub = ub->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r3[i1] = muDoubleScalarIsNaN(ub_data[i1]);
              }
              loop_ub = b_r->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r2[i1] = ((!r2[i1]) && (!r3[i1]));
              }
              d_st.site = &np_emlrtRSI;
              if (all(&d_st, b_r)) {
                real_T varargin_1;
                d_st.site = &op_emlrtRSI;
                varargin_1 = c_obj->PositionNumber;
                if (!(varargin_1 >= 0.0)) {
                  emlrtNonNegativeCheckR2012b(varargin_1, &gc_emlrtDCI, &d_st);
                }
                i1 = rn->size[0];
                rn->size[0] = (int32_T)varargin_1;
                emxEnsureCapacity_real_T(&d_st, rn, i1, &eg_emlrtRTEI);
                rn_data = rn->data;
                if ((int32_T)varargin_1 != 0) {
                  emlrtRandu(&rn_data[0], (int32_T)varargin_1);
                }
                if ((ub->size[0] != lb->size[0]) &&
                    ((ub->size[0] != 1) && (lb->size[0] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(ub->size[0], lb->size[0],
                                              &r_emlrtECI, &c_st);
                }
                if (ub->size[0] == lb->size[0]) {
                  loop_ub = ub->size[0];
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    ub_data[i1] -= lb_data[i1];
                  }
                } else {
                  d_st.site = &nq_emlrtRSI;
                  minus(&d_st, ub, lb);
                  ub_data = ub->data;
                }
                if ((rn->size[0] != ub->size[0]) &&
                    ((rn->size[0] != 1) && (ub->size[0] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(rn->size[0], ub->size[0],
                                              &s_emlrtECI, &c_st);
                }
                if (rn->size[0] == ub->size[0]) {
                  loop_ub = rn->size[0];
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    rn_data[i1] *= ub_data[i1];
                  }
                } else {
                  d_st.site = &nq_emlrtRSI;
                  times(&d_st, rn, ub);
                  rn_data = rn->data;
                }
                if ((lb->size[0] != rn->size[0]) &&
                    ((lb->size[0] != 1) && (rn->size[0] != 1))) {
                  emlrtDimSizeImpxCheckR2021b(lb->size[0], rn->size[0],
                                              &t_emlrtECI, &c_st);
                }
                if (lb->size[0] == rn->size[0]) {
                  loop_ub = lb->size[0];
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    lb_data[i1] += rn_data[i1];
                  }
                } else {
                  d_st.site = &nq_emlrtRSI;
                  plus(&d_st, lb, rn);
                  lb_data = lb->data;
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }
            if (guard3) {
              i1 = b_r->size[0];
              b_r->size[0] = lb->size[0];
              emxEnsureCapacity_boolean_T(&c_st, b_r, i1, &cg_emlrtRTEI);
              r2 = b_r->data;
              loop_ub = lb->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r2[i1] = muDoubleScalarIsInf(lb_data[i1]);
              }
              i1 = r1->size[0];
              r1->size[0] = lb->size[0];
              emxEnsureCapacity_boolean_T(&c_st, r1, i1, &dg_emlrtRTEI);
              r3 = r1->data;
              loop_ub = lb->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r3[i1] = muDoubleScalarIsNaN(lb_data[i1]);
              }
              loop_ub = b_r->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r2[i1] = ((!r2[i1]) && (!r3[i1]));
              }
              d_st.site = &pp_emlrtRSI;
              if (all(&d_st, b_r)) {
                i1 = b_r->size[0];
                b_r->size[0] = ub->size[0];
                emxEnsureCapacity_boolean_T(&c_st, b_r, i1, &cg_emlrtRTEI);
                r2 = b_r->data;
                loop_ub = ub->size[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  r2[i1] = muDoubleScalarIsInf(ub_data[i1]);
                }
                i1 = r1->size[0];
                r1->size[0] = ub->size[0];
                emxEnsureCapacity_boolean_T(&c_st, r1, i1, &dg_emlrtRTEI);
                r3 = r1->data;
                loop_ub = ub->size[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  r3[i1] = muDoubleScalarIsNaN(ub_data[i1]);
                }
                loop_ub = b_r->size[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  r2[i1] = (r2[i1] || r3[i1]);
                }
                d_st.site = &pp_emlrtRSI;
                if (any(&d_st, b_r)) {
                  d_st.site = &qp_emlrtRSI;
                  i1 = r->size[0];
                  r->size[0] = lb->size[0];
                  emxEnsureCapacity_real_T(&d_st, r, i1, &fg_emlrtRTEI);
                  xSol_data = r->data;
                  if (lb->size[0] != 0) {
                    emlrtRandn(&xSol_data[0], lb->size[0]);
                  }
                  d_st.site = &qp_emlrtRSI;
                  b_abs(&d_st, r, rn);
                  rn_data = rn->data;
                  if ((lb->size[0] != rn->size[0]) &&
                      ((lb->size[0] != 1) && (rn->size[0] != 1))) {
                    emlrtDimSizeImpxCheckR2021b(lb->size[0], rn->size[0],
                                                &u_emlrtECI, &c_st);
                  }
                  if (lb->size[0] == rn->size[0]) {
                    loop_ub = lb->size[0];
                    for (i1 = 0; i1 < loop_ub; i1++) {
                      lb_data[i1] += rn_data[i1];
                    }
                  } else {
                    d_st.site = &qp_emlrtRSI;
                    plus(&d_st, lb, rn);
                    lb_data = lb->data;
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }
            if (guard2) {
              i1 = b_r->size[0];
              b_r->size[0] = lb->size[0];
              emxEnsureCapacity_boolean_T(&c_st, b_r, i1, &cg_emlrtRTEI);
              r2 = b_r->data;
              loop_ub = lb->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r2[i1] = muDoubleScalarIsInf(lb_data[i1]);
              }
              i1 = r1->size[0];
              r1->size[0] = lb->size[0];
              emxEnsureCapacity_boolean_T(&c_st, r1, i1, &dg_emlrtRTEI);
              r3 = r1->data;
              loop_ub = lb->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r3[i1] = muDoubleScalarIsNaN(lb_data[i1]);
              }
              loop_ub = b_r->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                r2[i1] = (r2[i1] || r3[i1]);
              }
              d_st.site = &rp_emlrtRSI;
              if (any(&d_st, b_r)) {
                i1 = b_r->size[0];
                b_r->size[0] = ub->size[0];
                emxEnsureCapacity_boolean_T(&c_st, b_r, i1, &cg_emlrtRTEI);
                r2 = b_r->data;
                loop_ub = ub->size[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  r2[i1] = muDoubleScalarIsInf(ub_data[i1]);
                }
                i1 = r1->size[0];
                r1->size[0] = ub->size[0];
                emxEnsureCapacity_boolean_T(&c_st, r1, i1, &dg_emlrtRTEI);
                r3 = r1->data;
                loop_ub = ub->size[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  r3[i1] = muDoubleScalarIsNaN(ub_data[i1]);
                }
                loop_ub = b_r->size[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  r2[i1] = ((!r2[i1]) && (!r3[i1]));
                }
                d_st.site = &rp_emlrtRSI;
                if (all(&d_st, b_r)) {
                  d_st.site = &sp_emlrtRSI;
                  i1 = r->size[0];
                  r->size[0] = ub->size[0];
                  emxEnsureCapacity_real_T(&d_st, r, i1, &fg_emlrtRTEI);
                  xSol_data = r->data;
                  if (ub->size[0] != 0) {
                    emlrtRandn(&xSol_data[0], ub->size[0]);
                  }
                  d_st.site = &sp_emlrtRSI;
                  b_abs(&d_st, r, lb);
                  if ((ub->size[0] != lb->size[0]) &&
                      ((ub->size[0] != 1) && (lb->size[0] != 1))) {
                    emlrtDimSizeImpxCheckR2021b(ub->size[0], lb->size[0],
                                                &v_emlrtECI, &c_st);
                  }
                  if (ub->size[0] == lb->size[0]) {
                    i1 = lb->size[0];
                    lb->size[0] = ub->size[0];
                    emxEnsureCapacity_real_T(&c_st, lb, i1, &yf_emlrtRTEI);
                    lb_data = lb->data;
                    loop_ub = ub->size[0];
                    for (i1 = 0; i1 < loop_ub; i1++) {
                      lb_data[i1] = ub_data[i1] - lb_data[i1];
                    }
                  } else {
                    d_st.site = &sp_emlrtRSI;
                    b_minus(&d_st, lb, ub);
                    lb_data = lb->data;
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }
            if (guard1) {
              d_st.site = &tp_emlrtRSI;
              i1 = lb->size[0];
              lb->size[0] = ub->size[0];
              emxEnsureCapacity_real_T(&d_st, lb, i1, &fg_emlrtRTEI);
              lb_data = lb->data;
              if (ub->size[0] != 0) {
                emlrtRandn(&lb_data[0], ub->size[0]);
              }
            }
          }
          if (err > iter) {
            i1 = 0;
            i2 = 0;
          } else {
            if (err != (int32_T)muDoubleScalarFloor(err)) {
              emlrtIntegerCheckR2012b(err, &bc_emlrtDCI, &b_st);
            }
            if (((int32_T)err < 1) || ((int32_T)err > newseed->size[0])) {
              emlrtDynamicBoundsCheckR2012b((int32_T)err, 1, newseed->size[0],
                                            &rc_emlrtBCI, &b_st);
            }
            i1 = (int32_T)err - 1;
            if (iter != (int32_T)muDoubleScalarFloor(iter)) {
              emlrtIntegerCheckR2012b(iter, &cc_emlrtDCI, &b_st);
            }
            if (((int32_T)iter < 1) || ((int32_T)iter > newseed->size[0])) {
              emlrtDynamicBoundsCheckR2012b((int32_T)iter, 1, newseed->size[0],
                                            &sc_emlrtBCI, &b_st);
            }
            i2 = (int32_T)iter;
          }
          loop_ub = i2 - i1;
          if (loop_ub != lb->size[0]) {
            emlrtSubAssignSizeCheck1dR2017a(loop_ub, lb->size[0], &q_emlrtECI,
                                            &b_st);
          }
          for (i2 = 0; i2 < loop_ub; i2++) {
            newseed_data[i1 + i2] = lb_data[i2];
          }
        }
      }
      i = obj->SeedInternal->size[0];
      if (i != newseed->size[0]) {
        emlrtSubAssignSizeCheck1dR2017a(i, newseed->size[0], &p_emlrtECI,
                                        (emlrtConstCTX)sp);
      }
      i = obj->SeedInternal->size[0];
      obj->SeedInternal->size[0] = newseed->size[0];
      emxEnsureCapacity_real_T(sp, obj->SeedInternal, i, &wf_emlrtRTEI);
      loop_ub = newseed->size[0];
      for (i = 0; i < loop_ub; i++) {
        obj->SeedInternal->data[i] = newseed_data[i];
      }
      st.site = &wg_emlrtRSI;
      c_DampedBFGSwGradientProjection(&st, obj, rn, &exitFlag, &err, &iter);
      rn_data = rn->data;
      if (err < errPrev) {
        i = xSol->size[0];
        xSol->size[0] = rn->size[0];
        emxEnsureCapacity_real_T(sp, xSol, i, &ag_emlrtRTEI);
        xSol_data = xSol->data;
        loop_ub = rn->size[0];
        for (i = 0; i < loop_ub; i++) {
          xSol_data[i] = rn_data[i];
        }
        errPrev = err;
        exitFlagPrev = exitFlag;
      }
      rrAttempts++;
      iterations += iter;
    }
  }
  emxFree_boolean_T(sp, &r1);
  emxFree_boolean_T(sp, &b_r);
  emxFree_real_T(sp, &r);
  emxFree_real_T(sp, &rn);
  emxFree_real_T(sp, &lb);
  emxFree_real_T(sp, &ub);
  emxFree_real_T(sp, &newseed);
  if (errPrev < tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (i = 0; i < 7; i++) {
      solutionInfo_Status_data[i] = b_cv1[i];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (i = 0; i < 14; i++) {
      solutionInfo_Status_data[i] = b_cv[i];
    }
  }
  *solutionInfo_Iterations = iterations;
  *solutionInfo_RRAttempts = rrAttempts;
  *solutionInfo_Error = errPrev;
  *solutionInfo_ExitFlag = (real_T)exitFlagPrev;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (NLPSolverInterface.c) */
