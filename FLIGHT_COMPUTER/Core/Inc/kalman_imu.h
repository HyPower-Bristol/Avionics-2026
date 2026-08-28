#ifndef INC_KALMAN_IMU_H_
#define INC_KALMAN_IMU_H_

/*
 * 2-state Kalman filter for one attitude axis.
 *
 * State:      x = [angle, gyro_bias]
 * Prediction: gyro rate integration (bias-corrected)
 * Update:     absolute angle from accelerometer (gravity reference)
 *
 * Compared to a complementary filter this also estimates the gyro's
 * slowly-varying bias, so the angle doesn't lean as the gyro warms up.
 */
typedef struct {
    /* tuning */
    float Q_angle;   /* process noise: angle    (default 0.001)  */
    float Q_bias;    /* process noise: bias     (default 0.003)  */
    float R_measure; /* accel angle meas. noise (default 0.03)   */
    /* state */
    float angle;     /* filtered angle, degrees */
    float bias;      /* estimated gyro bias, deg/s */
    float P[2][2];   /* error covariance */
} KalmanAxis;

void  KalmanAxis_Init(KalmanAxis *k, float initial_angle);

/*
 * One predict+update step.
 *   accel_angle : absolute angle from accelerometer, degrees
 *   gyro_rate   : angular rate, deg/s
 *   dt          : timestep, seconds
 * Returns the filtered angle in degrees.
 */
float KalmanAxis_Update(KalmanAxis *k, float accel_angle,
                        float gyro_rate, float dt);

#endif /* INC_KALMAN_IMU_H_ */
