#include "kalman_imu.h"

void KalmanAxis_Init(KalmanAxis *k, float initial_angle)
{
    k->Q_angle   = 0.001f;
    k->Q_bias    = 0.003f;
    k->R_measure = 0.03f;

    k->angle = initial_angle;
    k->bias  = 0.0f;

    k->P[0][0] = 1.0f;  /* some initial uncertainty */
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f;
}

float KalmanAxis_Update(KalmanAxis *k, float accel_angle,
                        float gyro_rate, float dt)
{
    /* --- Predict: integrate bias-corrected rate --- */
    float rate = gyro_rate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    /* --- Update: correct with accelerometer angle --- */
    float S  = k->P[0][0] + k->R_measure;   /* innovation covariance */
    float K0 = k->P[0][0] / S;              /* Kalman gain           */
    float K1 = k->P[1][0] / S;

    float y = accel_angle - k->angle;       /* innovation */
    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0];
    float P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}
