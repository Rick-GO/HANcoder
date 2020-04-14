/*=====================================*
 * Required setup for C MEX S-Function *
 *=====================================*/

#define S_FUNCTION_NAME sfcn_can_send_packed
#define S_FUNCTION_LEVEL 2



/*========================*
 * General Defines/macros *
 *========================*/



 /*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h" 
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>



// -------------------------------------------------------------------------------
// Macros to access the S-function parameter values
// -------------------------------------------------------------------------------

#define NUMBER_OF_ARG		2					// Number of s-function input arguments



// ----------------------------------------------------------------------------------------------------
// S-Function methods
// ----------------------------------------------------------------------------------------------------

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify they are okay.
   */
static void mdlCheckParameters(SimStruct *S)
{
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Initialize the sizes array
 */
/* Function: mdlInitializeSizes ===============================================
 *
 */
static void mdlInitializeSizes (SimStruct *S)
{
  /* set the expexted number of parameters for the S-function block */
  ssSetNumSFcnParams(S, NUMBER_OF_ARG);
   
  /* verify the number of parametrs that are specified for the S-function block */
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
  {
    /* incorrect number of parameters */
    mexPrintf("Incorrect number of parameters (%d). Should be %d\n",
             ssGetSFcnParamsCount(S), ssGetNumSFcnParams(S));
    return;
  }

  /* needed to force checking on the parameter data types */
  mdlCheckParameters(S);
  if (ssGetErrorStatus(S) != NULL)
  {
    return;
  }

  /* set number of input ports for the S-function block */
  if(!ssSetNumInputPorts(S, 1))
  {
    mexPrintf("Could not set number of input ports to %d\n", 1);
    return;
  }
   
  /* set number of output ports forthe S-function block */
  if(!ssSetNumOutputPorts(S, 0))
  {
    mexPrintf("Could not set number of output ports to %d\n", 2);
    return;
  }

 
DTypeId dataTypeIdReg;
ssRegisterTypeFromNamedObject(S, "CAN_MESSAGE_BUS", &dataTypeIdReg);
if(dataTypeIdReg == INVALID_DTYPE_ID) return;

ssSetInputPortDataType(S, 0, dataTypeIdReg);
ssSetInputPortWidth(S, 0, 1);
ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
ssSetInputPortDirectFeedThrough(S, 0, 1);
ssSetInputPortAcceptExprInRTW(S, 0, 0);
ssSetInputPortOverWritable(S, 0, 0);
ssSetInputPortOptimOpts(S, 0, SS_NOT_REUSABLE_AND_LOCAL);
ssSetInputPortRequiredContiguous(S, 0, 1);
 
ssSetArrayLayoutForCodeGen(S, SS_ALL);
 
 
  /* only 1 sampletime in this S-Function */
  ssSetNumSampleTimes (S, 1);

  ssSetNumContStates (S, 0);					// number of continuous states
  ssSetNumDiscStates (S, 0);					// number of discrete states
  
  ssSetNumIWork (S, 0);						// number of integer work vector elements
  ssSetNumRWork (S, 0);						// number of real work vector elements
  ssSetNumPWork (S, 0);						// number of pointer work vector elements
  ssSetNumDWork(S, 0); // number of data type work vector elements
   
  /* options */
  ssSetOptions(S,SS_OPTION_WORKS_WITH_CODE_REUSE | SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_CAN_BE_CALLED_CONDITIONALLY);
}


/* Function: mdlInitializeSampleTimes =========================================
 *
 */
static void mdlInitializeSampleTimes (SimStruct *S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


/* Function: mdlStart =========================================================
 *
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
   /* not used here */
}


/*
 * mdlOutputs - compute the outputs
 *
 * In this function, you compute the outputs of your S-function
 * block.  The outputs are placed in the y variable.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
   /* not used here */
}


/*
 * mdlTerminate - called when the simulation is terminated.
 *
 * In this function, you should perform any actions that are necessary
 * at the termination of a simulation.  For example, if memory was allocated
 * in mdlInitializeConditions, this is the place to free it.
 */
static void mdlTerminate (SimStruct *S)
{
   /* not used here */
}


// the define 'MATLAB_MEX_FILE' has to be specified when recompiling this module to a DLL.
#ifdef MATLAB_MEX_FILE
   #include "simulink.c"
#else
# error "Attempt to use <s_function>.c as non-inlined S-function"
#endif
/********************************* end of sfcn_pwmout_init.c ***************************/

