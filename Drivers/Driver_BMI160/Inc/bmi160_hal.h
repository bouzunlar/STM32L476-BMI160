#ifndef BMI160_HAL_H_
#define BMI160_HAL_H_


#ifdef __cplusplus
extern "C" {
#endif
/*=========================================================================
    REGISTERS
=========================================================================*/
#define BMI160_REG_CHIP_ID          (0x00)   /* WHO_AM_I, expected: 0xD1  */
#define BMI160_REG_DATA_GYR_X_L    (0x0C)   /* Gyro X LSB (burst: 0C-11) */
#define BMI160_REG_DATA_ACC_X_L    (0x12)   /* Acc  X LSB (burst: 12-17) */
#define BMI160_REG_TEMPERATURE_L   (0x20)   /* Temperature LSB            */
#define BMI160_REG_ACC_CONF        (0x40)   /* Acc  ODR / BWP             */
#define BMI160_REG_ACC_RANGE       (0x41)   /* Acc  range                 */
#define BMI160_REG_GYR_CONF        (0x42)   /* Gyro ODR / BWP             */
#define BMI160_REG_GYR_RANGE       (0x43)   /* Gyro range                 */
#define BMI160_REG_CMD             (0x7E)   /* Command register           */

/* Interrupt configuration registers */
#define BMI160_REG_INT_EN_1        (0x51)   /* Interrupt enable 1         */
#define BMI160_REG_INT_OUT_CTRL    (0x53)   /* INT1/INT2 output behavior  */
#define BMI160_REG_INT_LATCH       (0x54)   /* Latch mode for interrupts  */
#define BMI160_REG_INT_MAP_1       (0x56)   /* Map interrupts to INT1/INT2 */

/* INT_EN_1 bits */
#define BMI160_INT_EN_1_DRDY       (0x10)   /* Data ready interrupt enable */

/* INT_OUT_CTRL bits — INT1 */
#define BMI160_INT_OUT_CTRL_INT1_OUT_EN  (0x08) /* INT1 output enable      */
#define BMI160_INT_OUT_CTRL_INT1_LVL_HI  (0x02) /* INT1 active high        */
#define BMI160_INT_OUT_CTRL_INT1_PUSHPULL (0x00) /* INT1 push-pull (default) */

/* INT_LATCH values (bits [3:0]) */
#define BMI160_INT_LATCH_NONE      (0x00)   /* Non-latched (pulsed)        */

/* INT_MAP_1 bits — map data-ready to INT1 */
#define BMI160_INT_MAP_1_DRDY_INT1 (0x80)   /* Route DRDY → INT1           */

/*=========================================================================
    CMD VALUES
=========================================================================*/
#define BMI160_CMD_SOFTRESET        (0xB6)
#define BMI160_CMD_ACC_NORMAL       (0x11)
#define BMI160_CMD_GYR_NORMAL       (0x15)

/*=========================================================================
    CHIP ID
=========================================================================*/
#define BMI160_CHIP_ID              (0xD1)

/*=========================================================================
    ACC_CONF  ODR values  (bits [3:0])
=========================================================================*/
#define BMI160_ACC_ODR_100HZ        (0x08)

/*=========================================================================
    ACC_CONF  BWP values  (bits [6:4])
=========================================================================*/
#define BMI160_ACC_BWP_NORMAL       (0x02)

/*=========================================================================
    ACC_RANGE values
=========================================================================*/
#define BMI160_ACC_RANGE_4G         (0x05)

/*=========================================================================
    GYR_CONF  ODR values  (bits [3:0])
=========================================================================*/
#define BMI160_GYR_ODR_100HZ        (0x08)

/*=========================================================================
    GYR_CONF  BWP values  (bits [5:4])
=========================================================================*/
#define BMI160_GYR_BWP_NORMAL       (0x02)

/*=========================================================================
    GYR_RANGE values
=========================================================================*/
#define BMI160_GYR_RANGE_500DPS     (0x02)

/*=========================================================================
    SCALE FACTORS
    Acc  : ±4G  -> 4 / 32768 = 0.000122070 g/LSB
    Gyro : ±500 -> 500 / 32768 = 0.015259 dps/LSB
    Temp : 1/512 deg/LSB, 0 LSB = 23 °C
=========================================================================*/
#define BMI160_ACC_SCALE_4G         (0.000122070f)
#define BMI160_GYR_SCALE_500DPS     (0.015259f)
#define BMI160_TEMP_SCALE           (0.001953125f)   /* 1/512 */
#define BMI160_TEMP_OFFSET          (23.0f)

/*=========================================================================
    STARTUP DELAYS (ms)
=========================================================================*/
#define BMI160_SOFTRESET_DELAY_MS   (100u)
#define BMI160_ACC_NORMAL_DELAY_MS  (5u)
#define BMI160_GYR_NORMAL_DELAY_MS  (80u)



#define BMI160_ADDR             (0x68 << 1)

#ifdef __cplusplus
}
#endif

#endif /* BMI160_HAL_H_ */
