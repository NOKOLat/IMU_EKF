/*************************************************************************************************************
 * STM32 Attitude Estimation using Extended Kalman Filter (EKF)
 *
 * Implementation of attitude estimation functions
 *
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/

#include "attitude_ekf.h"
#include <string.h>

/* C++ implementation with C interface */
extern "C" {


/* ============================================= EKF Parameter Constants ============================================= */

/* Tuning parameters - adjust based on sensor characteristics and noise */
#define P_INIT_VALUE    (10.0f)         /* Initial state estimate uncertainty */
#define Q_INIT_VALUE    (1e-4f)         /* Process noise covariance */
#define R_INIT_VALUE    (0.01f)         /* Measurement noise covariance */


/* ================================================= Nonlinear Functions ================================================= */

/**
 * Nonlinear state update function (gyroscope integration)
 *
 * State equation: x(k+1) = f(x(k), u(k))
 *
 * Integrates angular velocities to update Euler angles using Euler method:
 *  φ_dot = p + q*sin(φ)*tan(θ) + r*cos(φ)*tan(θ)
 *  θ_dot = q*cos(φ) - r*sin(φ)
 *  ψ_dot = q*sin(φ)/cos(θ) + r*cos(φ)/cos(θ)
 *
 * Then: x(k+1) = x(k) + x_dot * dt
 */
bool AttitudeEKF_UpdateX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    float_prec phi, theta, psi;      /* Roll, pitch, yaw angles */
    float_prec p, q, r;              /* Angular velocities */
    float_prec cos_phi, sin_phi;
    float_prec cos_theta, sin_theta, tan_theta;
    float_prec cos_theta_inv;        /* 1/cos(theta) - avoid division if possible */

    /* Extract state variables */
    phi = X(0, 0);
    theta = X(1, 0);
    psi = X(2, 0);

    /* Extract input variables (angular velocities) */
    p = U(0, 0);
    q = U(1, 0);
    r = U(2, 0);

    /* Precompute trigonometric values */
    cos_phi = cosf(phi);
    sin_phi = sinf(phi);
    cos_theta = cosf(theta);
    sin_theta = sinf(theta);
    tan_theta = sin_theta / cos_theta;

    /* Avoid division by zero for yaw rate calculation */
    if (fabsf(cos_theta) < 1e-6f) {
        cos_theta_inv = 1e6f;  /* Large value to prevent division issues */
    } else {
        cos_theta_inv = 1.0f / cos_theta;
    }

    /* Calculate derivatives: [φ_dot, θ_dot, ψ_dot]' */
    float_prec phi_dot = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta;
    float_prec theta_dot = q * cos_phi - r * sin_phi;
    float_prec psi_dot = q * sin_phi * cos_theta_inv + r * cos_phi * cos_theta_inv;

    /* Euler integration: x(k+1) = x(k) + x_dot * dt */
    X_Next(0, 0) = phi + phi_dot * SS_DT;
    X_Next(1, 0) = theta + theta_dot * SS_DT;
    X_Next(2, 0) = psi + psi_dot * SS_DT;

    /* Normalize angles to [-π, π] range to avoid singularities */
    for (int16_t i = 0; i < 3; i++) {
        while (X_Next(i, 0) > M_PI) {
            X_Next(i, 0) -= 2.0f * M_PI;
        }
        while (X_Next(i, 0) < -M_PI) {
            X_Next(i, 0) += 2.0f * M_PI;
        }
    }

    return true;
}

/**
 * Nonlinear measurement function (accelerometer model)
 *
 * Measurement equation: z(k) = h(x(k), u(k))
 *
 * Expected accelerometer reading based on attitude (assuming gravity = 1g):
 *  ax_pred = -sin(θ)
 *  ay_pred = sin(φ)*cos(θ)
 *  az_pred = cos(φ)*cos(θ)
 *
 * This is the gravity vector rotated from Earth frame to body frame.
 */
bool AttitudeEKF_UpdateY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    float_prec phi, theta;
    float_prec cos_phi, sin_phi;
    float_prec cos_theta, sin_theta;

    /* Extract attitude from state (ignore yaw since it doesn't affect accel) */
    phi = X(0, 0);      /* Roll */
    theta = X(1, 0);    /* Pitch */

    /* Precompute trigonometric values */
    cos_phi = cosf(phi);
    sin_phi = sinf(phi);
    cos_theta = cosf(theta);
    sin_theta = sinf(theta);

    /* Predicted accelerometer measurements (normalized gravity vector in body frame) */
    Y(0, 0) = -sin_theta;                   /* ax = -sin(θ) */
    Y(1, 0) = sin_phi * cos_theta;          /* ay = sin(φ)*cos(θ) */
    Y(2, 0) = cos_phi * cos_theta;          /* az = cos(φ)*cos(θ) */

    return true;
}

/**
 * Jacobian of state update function (F = ∂f/∂x)
 *
 * This is the Jacobian matrix of the nonlinear state update function
 */
bool AttitudeEKF_CalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    float_prec phi, theta;
    float_prec p, q, r;
    float_prec cos_phi, sin_phi;
    float_prec cos_theta, sin_theta, tan_theta;
    float_prec sec_theta;              /* 1/cos(theta) - secant function */

    /* Extract state and input */
    phi = X(0, 0);
    theta = X(1, 0);
    p = U(0, 0);
    q = U(1, 0);
    r = U(2, 0);

    /* Precompute trigonometric values */
    cos_phi = cosf(phi);
    sin_phi = sinf(phi);
    cos_theta = cosf(theta);
    sin_theta = sinf(theta);
    tan_theta = sin_theta / cos_theta;
    sec_theta = 1.0f / cos_theta;

    /* Jacobian F = I + (∂f/∂x)*dt */
    /* Initialize to identity matrix */
    F(0, 0) = 1.0f;
    F(0, 1) = (q * cos_phi - r * sin_phi) * tan_theta * SS_DT;
    F(0, 2) = 0.0f;

    F(1, 0) = -(q * sin_phi + r * cos_phi) * SS_DT;
    F(1, 1) = 1.0f;
    F(1, 2) = 0.0f;

    F(2, 0) = (q * cos_phi - r * sin_phi) * sec_theta * SS_DT;
    F(2, 1) = (q * sin_phi + r * cos_phi) * sin_theta * sec_theta * sec_theta * SS_DT;
    F(2, 2) = 1.0f;

    return true;
}

/**
 * Jacobian of measurement function (H = ∂h/∂x)
 *
 * This is the Jacobian matrix of the nonlinear measurement function
 */
bool AttitudeEKF_CalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    float_prec phi, theta;
    float_prec cos_phi, sin_phi;
    float_prec cos_theta, sin_theta;

    /* Extract attitude from state */
    phi = X(0, 0);
    theta = X(1, 0);

    /* Precompute trigonometric values */
    cos_phi = cosf(phi);
    sin_phi = sinf(phi);
    cos_theta = cosf(theta);
    sin_theta = sinf(theta);

    /* Jacobian H = ∂h/∂x
     *  ∂ax/∂φ = 0,                  ∂ax/∂θ = -cos(θ),         ∂ax/∂ψ = 0
     *  ∂ay/∂φ = cos(φ)*cos(θ),      ∂ay/∂θ = -sin(φ)*sin(θ),  ∂ay/∂ψ = 0
     *  ∂az/∂φ = -sin(φ)*cos(θ),     ∂az/∂θ = -cos(φ)*sin(θ),  ∂az/∂ψ = 0
     */
    H(0, 0) = 0.0f;
    H(0, 1) = -cos_theta;
    H(0, 2) = 0.0f;

    H(1, 0) = cos_phi * cos_theta;
    H(1, 1) = -sin_phi * sin_theta;
    H(1, 2) = 0.0f;

    H(2, 0) = -sin_phi * cos_theta;
    H(2, 1) = -cos_phi * sin_theta;
    H(2, 2) = 0.0f;

    return true;
}


/* ================================================= Initialization Functions ================================================= */

bool AttitudeEKF_Init(AttitudeEKF_t *ekf_sys, float_prec dt)
{
    if (!ekf_sys) {
        return false;
    }

    /* Initialize state vector (3x1) */
    ekf_sys->state = Matrix(SS_X_LEN, 1);
    ekf_sys->state.vSetToZero();

    /* Initialize measurement vector (3x1) */
    ekf_sys->measurement = Matrix(SS_Z_LEN, 1);
    ekf_sys->measurement.vSetToZero();

    /* Initialize input vector (3x1) */
    ekf_sys->input = Matrix(SS_U_LEN, 1);
    ekf_sys->input.vSetToZero();

    /* Initialize covariance matrix P (3x3) */
    ekf_sys->P_init = Matrix(SS_X_LEN, SS_X_LEN);
    ekf_sys->P_init.vSetDiag(P_INIT_VALUE);

    /* Initialize process noise covariance Q (3x3) */
    ekf_sys->Q = Matrix(SS_X_LEN, SS_X_LEN);
    ekf_sys->Q.vSetDiag(Q_INIT_VALUE);

    /* Initialize measurement noise covariance R (3x3) */
    ekf_sys->R = Matrix(SS_Z_LEN, SS_Z_LEN);
    ekf_sys->R.vSetDiag(R_INIT_VALUE);

    /* Initialize the EKF instance */
    ekf_sys->ekf_filter = EKF(
        ekf_sys->state,
        ekf_sys->P_init,
        ekf_sys->Q,
        ekf_sys->R,
        AttitudeEKF_UpdateX,
        AttitudeEKF_UpdateY,
        AttitudeEKF_CalcJacobianF,
        AttitudeEKF_CalcJacobianH
    );

    /* Initialize angle outputs */
    ekf_sys->roll = 0.0f;
    ekf_sys->pitch = 0.0f;
    ekf_sys->yaw = 0.0f;

    return true;
}

bool AttitudeEKF_Reset(AttitudeEKF_t *ekf_sys)
{
    if (!ekf_sys) {
        return false;
    }

    /* Reset state to zero */
    ekf_sys->state.vSetToZero();

    /* Reset covariances */
    ekf_sys->P_init.vSetDiag(P_INIT_VALUE);
    ekf_sys->Q.vSetDiag(Q_INIT_VALUE);
    ekf_sys->R.vSetDiag(R_INIT_VALUE);

    /* Reset the EKF */
    ekf_sys->ekf_filter.vReset(ekf_sys->state, ekf_sys->P_init, ekf_sys->Q, ekf_sys->R);

    /* Reset angle outputs */
    ekf_sys->roll = 0.0f;
    ekf_sys->pitch = 0.0f;
    ekf_sys->yaw = 0.0f;

    return true;
}


/* ================================================= Update Function ================================================= */

bool AttitudeEKF_Update(AttitudeEKF_t *ekf_sys, const float_prec accel[3], const float_prec gyro[3])
{
    if (!ekf_sys || !accel || !gyro) {
        return false;
    }

    /* Set input: gyroscope angular velocities [p, q, r] */
    ekf_sys->input(0, 0) = gyro[0];
    ekf_sys->input(1, 0) = gyro[1];
    ekf_sys->input(2, 0) = gyro[2];

    /* Set measurement: normalized accelerometer data [ax, ay, az] */
    /* Calculate norm to normalize accelerometer */
    float_prec accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    if (accel_norm < 1e-6f) {
        /* Invalid accelerometer reading */
        return false;
    }

    ekf_sys->measurement(0, 0) = accel[0] / accel_norm;
    ekf_sys->measurement(1, 0) = accel[1] / accel_norm;
    ekf_sys->measurement(2, 0) = accel[2] / accel_norm;

    /* Update EKF */
    if (!ekf_sys->ekf_filter.bUpdate(ekf_sys->measurement, ekf_sys->input)) {
        /* EKF update failed, reset and return false */
        AttitudeEKF_Reset(ekf_sys);
        return false;
    }

    /* Extract estimated state */
    ekf_sys->state = ekf_sys->ekf_filter.GetX();

    /* Update output angles */
    ekf_sys->roll = ekf_sys->state(0, 0);
    ekf_sys->pitch = ekf_sys->state(1, 0);
    ekf_sys->yaw = ekf_sys->state(2, 0);

    return true;
}


/* ================================================= Getter Functions ================================================= */

void AttitudeEKF_GetAngles(const AttitudeEKF_t *ekf_sys, float_prec angles[3])
{
    if (ekf_sys && angles) {
        angles[0] = ekf_sys->roll;
        angles[1] = ekf_sys->pitch;
        angles[2] = ekf_sys->yaw;
    }
}

float_prec AttitudeEKF_GetRoll(const AttitudeEKF_t *ekf_sys)
{
    return (ekf_sys) ? ekf_sys->roll : 0.0f;
}

float_prec AttitudeEKF_GetPitch(const AttitudeEKF_t *ekf_sys)
{
    return (ekf_sys) ? ekf_sys->pitch : 0.0f;
}

float_prec AttitudeEKF_GetYaw(const AttitudeEKF_t *ekf_sys)
{
    return (ekf_sys) ? ekf_sys->yaw : 0.0f;
}


/* ================================================= Error Handler ================================================= */

/**
 * Error handler function (called when assertions fail)
 * In embedded systems, this should not cause blocking behavior
 */
void SPEW_THE_ERROR(const char * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT)
        /* Silent error handling - no print */
        (void)str;  /* Avoid unused parameter warning */
    #else
        /* For debugging during development */
        /* You can implement custom debug output here */
        (void)str;
    #endif
    /* Note: Removed while(1) to allow system to continue operation */
}

} /* extern "C" */
