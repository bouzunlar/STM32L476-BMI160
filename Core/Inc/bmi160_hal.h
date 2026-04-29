#ifndef BMI160_HAL_H_
#define BMI160_HAL_H_


#define BMI160_ADDR             (0x68 << 1)


#define BMI160_REG_CHIPID       0x00
#define BMI160_REG_DATA_GYRO    0x0C
#define BMI160_REG_CMD          0x7E

/* Güç Modu Komutları */
#define BMI160_CMD_ACC_NORMAL   0x11
#define BMI160_CMD_GYRO_NORMAL  0x15

#endif /* BMI160_HAL_H_ */
