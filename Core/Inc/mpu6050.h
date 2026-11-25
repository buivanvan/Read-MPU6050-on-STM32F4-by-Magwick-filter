#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
} ImuRaw;

void timer_init(void);
uint32_t micros(void);
void delay_us(uint32_t us);
bool mpu6050_init(void);
bool mpu6050_read(ImuRaw *sample);

#endif
