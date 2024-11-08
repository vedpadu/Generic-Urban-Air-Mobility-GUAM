/*
 * LpC_wrapper.h
 *
 * Code generation for function 'LpC_wrapper'
 *
 */

#ifndef LPC_WRAPPER_H
#define LPC_WRAPPER_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void LpC_wrapper(double rho, const double V_b[3], const double om_b[3],
    const double om_prop[5], const double surf[4], boolean_T ders, double FM[6],
    double FM_aero[6], double FM_prop[6], double Prop[10], double FM_x[36],
    double FM_u[156]);
    // Prop size 10 - 5 motors * 2 -> thrust and torque for each one
    // 42 * 6 = 252, 26 * 6 = 156 -> 6 forces and moments per u mode
    // 6 * 6 = 36
    // u and x definitely have to change. changed.. should Prop change
    // u was 13 long before

#ifdef __cplusplus

}
#endif
#endif

/* End of code generation (LpC_wrapper.h) */
