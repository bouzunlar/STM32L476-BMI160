#ifndef IMU_TIMING_H_
#define IMU_TIMING_H_

/*
 * Single source of truth for IMU timing.
 * IMU_ODR_HZ must match the BMI160 ACC/GYR ODR programmed in bmi160_driver.c.
 */
#define IMU_ODR_HZ      (100u)
#define IMU_DT_SEC      (1.0f / (float) IMU_ODR_HZ)
#define IMU_DT_MS       (1000u / IMU_ODR_HZ)

#endif /* IMU_TIMING_H_ */
