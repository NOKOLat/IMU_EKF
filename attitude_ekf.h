/*************************************************************************************************************
 * STM32 Attitude Estimation using Extended Kalman Filter (EKF)
 *
 * This module implements attitude estimation (roll, pitch, yaw) from IMU sensor data
 * using an Extended Kalman Filter. It fuses accelerometer and gyroscope measurements.
 *
 * State Vector X: [roll (φ), pitch (θ), yaw (ψ)] - in radians
 * Input Vector U: [p, q, r] - angular velocities from gyroscope (rad/s)
 * Measurement Vector Z: [ax, ay, az] - accelerometer data (normalized)
 *
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/

#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"


/* ========================================== Attitude EKF System Structure ========================================== */

typedef struct {
    Matrix state;           /* State vector X [roll, pitch, yaw] */
    Matrix measurement;     /* Measurement vector Z [ax, ay, az] */
    Matrix input;           /* Input vector U [p, q, r] */
    EKF ekf_filter;         /* EKF instance */

    /* Covariance matrices */
    Matrix P_init;          /* Initial estimate covariance */
    Matrix Q;               /* Process noise covariance */
    Matrix R;               /* Measurement noise covariance */

    /* Euler angles output */
    float_prec roll;        /* Roll angle (φ) in radians */
    float_prec pitch;       /* Pitch angle (θ) in radians */
    float_prec yaw;         /* Yaw angle (ψ) in radians */
} AttitudeEKF_t;


/* ========================================== Function Declarations ========================================== */

/**
 * Initialize the attitude EKF system
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @param dt        Sampling time in seconds
 * @return          true if initialization successful, false otherwise
 */
bool AttitudeEKF_Init(AttitudeEKF_t *ekf_sys, float_prec dt);

/**
 * Reset the EKF state to initial values
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @return          true if reset successful, false otherwise
 */
bool AttitudeEKF_Reset(AttitudeEKF_t *ekf_sys);

/**
 * Update the EKF with sensor measurements
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @param accel     Accelerometer data [ax, ay, az] (should be normalized)
 * @param gyro      Gyroscope data [p, q, r] in rad/s
 * @return          true if update successful, false otherwise
 */
bool AttitudeEKF_Update(AttitudeEKF_t *ekf_sys, const float_prec accel[3], const float_prec gyro[3]);

/**
 * Get estimated Euler angles
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @param angles    Output array for angles [roll, pitch, yaw] in radians
 */
void AttitudeEKF_GetAngles(const AttitudeEKF_t *ekf_sys, float_prec angles[3]);

/**
 * Get the roll angle estimate
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @return          Roll angle in radians
 */
float_prec AttitudeEKF_GetRoll(const AttitudeEKF_t *ekf_sys);

/**
 * Get the pitch angle estimate
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @return          Pitch angle in radians
 */
float_prec AttitudeEKF_GetPitch(const AttitudeEKF_t *ekf_sys);

/**
 * Get the yaw angle estimate
 *
 * @param ekf_sys   Pointer to AttitudeEKF_t structure
 * @return          Yaw angle in radians
 */
float_prec AttitudeEKF_GetYaw(const AttitudeEKF_t *ekf_sys);


/* ========================================== Internal EKF Functions (called by EKF) ========================================== */

/**
 * Nonlinear state update function for gyroscope integration
 * Updates the attitude using gyroscope angular velocities
 */
bool AttitudeEKF_UpdateX(Matrix& X_Next, const Matrix& X, const Matrix& U);

/**
 * Nonlinear measurement function for accelerometer
 * Calculates expected accelerometer reading given estimated attitude
 */
bool AttitudeEKF_UpdateY(Matrix& Y, const Matrix& X, const Matrix& U);

/**
 * Jacobian of the state update function (F matrix)
 */
bool AttitudeEKF_CalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);

/**
 * Jacobian of the measurement function (H matrix)
 */
bool AttitudeEKF_CalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);


#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_EKF_H
