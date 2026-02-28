/*************************************************************************************************************
 * This file contains configuration parameters for STM32 Attitude Estimation using EKF
 *
 * State: [roll (φ), pitch (θ), yaw (ψ)]
 * Input: [p, q, r] - angular velocities from gyroscope
 * Measurement: [ax, ay, az] - accelerometer data
 *
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef KONFIG_H
#define KONFIG_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>


/* State Space dimension */
#define SS_X_LEN    (3)             /* State: roll, pitch, yaw */
#define SS_Z_LEN    (3)             /* Measurement: accelerometer (3-axis) */
#define SS_U_LEN    (3)             /* Input: gyroscope angular velocities (p, q, r) */
#define SS_DT_MS    (10)            /* 10 ms (100 Hz) */
#define SS_DT       (SS_DT_MS/1000.0f)   /* Sampling time in seconds */


/* Change this size based on the biggest matrix you will use */
#define MATRIX_MAXIMUM_SIZE     (3)

/* Define this to enable matrix bound checking */
#define MATRIX_USE_BOUNDS_CHECKING

/* Set this define to choose math precision of the system */
#define PRECISION_SINGLE    1
#define PRECISION_DOUBLE    2
#define FPU_PRECISION       (PRECISION_SINGLE)

#if (FPU_PRECISION == PRECISION_SINGLE)
    #define float_prec          float
    #define float_prec_ZERO     (1e-7f)
    #define float_prec_ZERO_ECO (1e-5f)      /* 'Economical' zero, for noisy calculation */
#elif (FPU_PRECISION == PRECISION_DOUBLE)
    #define float_prec          double
    #define float_prec_ZERO     (1e-13)
    #define float_prec_ZERO_ECO (1e-8)       /* 'Economical' zero, for noisy calculation */
#else
    #error "FPU_PRECISION has not been defined!"
#endif


/* Set this define to choose system implementation */
#define SYSTEM_IMPLEMENTATION_PC                    1
#define SYSTEM_IMPLEMENTATION_EMBEDDED_CUSTOM       2
#define SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO      3
#define SYSTEM_IMPLEMENTATION_EMBEDDED_STM32        4
#define SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT     5

#define SYSTEM_IMPLEMENTATION                       (SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT)


/* ASSERT is evaluated locally (without function call) to lower the computation cost */
void SPEW_THE_ERROR(const char * str);
#define ASSERT(truth, str) { if (!(truth)) SPEW_THE_ERROR(str); }


#endif // KONFIG_H
