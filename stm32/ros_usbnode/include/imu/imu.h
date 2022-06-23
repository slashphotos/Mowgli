
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/*
 * conversions to ROS units
 */
#define RAD_PER_G               0.01745             // convert °/sec to rad/sec
#define MS2_PER_G               9.80665             // convert G to m/s^2 
#define T_PER_GAUSS             1/10000             // conver Gauss to T


/*
 * IMU functions that a compatible IMU needs to be able to provide
 */

uint8_t IMU_TestDevice(void);           // test of existence of IMU and if possible functioning 
void IMU_Init(void);                    // initialize IMU (make settings, ready to fetch readings

void IMU_ReadAccelerometer(float *x, float *y, float *z);
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z);
float IMU_Onboard_ReadTemp(void);
void IMU_ReadGyro(float *x, float *y, float *z);
void IMU_ReadMagnetometer(float *x, float *y, float *z);
float IMU_ReadBarometerTemperatureC(void);
float IMU_ReadBarometerAltitudeMeters(void);
void IMU_Onboard_AccelerometerSetCovariance(float *cm);
void IMU_AccelerometerSetCovariance(float *cm);
void IMU_GyroSetCovariance(float *cm);



/* Any external IMU needs to implement the following functions and adhere to the ROS REP 103 standard (https://www.ros.org/reps/rep-0103.html) */
void IMU_ReadGyroRaw(float *x, float *y, float *z);
void IMU_ReadAccelerometerRaw(float *x, float *y, float *z);
void IMU_ReadMagnetometerRaw(float *x, float *y, float *z);
/* end of functions to implement for IMU */


/* IMU calibration (accel/gyro only) */
#define IMU_CAL_SAMPLES     100
void IMU_Calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */