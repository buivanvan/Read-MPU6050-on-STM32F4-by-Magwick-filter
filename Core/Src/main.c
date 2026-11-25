/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "mpu6050.h"
#include "madgwick.h"
#include "stm32f407xx.h"
#include <math.h>
#include <stdio.h>

#ifndef use_filter
#define use_filter 1  /* 1: Madgwick orientation, 0: calibrated IMU data only */
#endif
#if use_filter != 0 && use_filter != 1
#error "use_filter must be 0 or 1"
#endif
#define CALIBRATION_SAMPLES 200u
#define BETA 0.08f
#define RAD_TO_DEG 57.2957795f

static const float accel_bias[3] = {0, 0, 0};
static const float accel_lsb_per_g[3] = {16384, 16384, 16384};
static float gyro_bias[3];

typedef struct {
#if use_filter
    Quaternion q;
    float roll_deg, pitch_deg, yaw_deg;
#endif
    float accel_g[3], gyro_rad_s[3];
    uint32_t dt_us;
} Attitude;

/* Watch these symbols in the debugger. */
volatile Attitude attitude;
volatile uint32_t sample_count, error_count;
typedef enum { CONNECTING, CALIBRATING, RUNNING } ImuState;
volatile ImuState imu_state;

static void uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15u << 8)) | (7u << 8);
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 4)) | (2u << 4);
    USART2->BRR = (16000000u + 57600u) / 115200u;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

/* Integer CSV keeps transmission time bounded at 115200 baud. */
static void send_sample(uint32_t time_us, const Attitude *out)
{
    char line[112];
    int i, length;
    length = snprintf(line, sizeof(line), "%lu,%d,%ld,%ld,%ld,%ld,%ld,%ld",
        (unsigned long)time_us, use_filter,
        lroundf(1000*out->accel_g[0]), lroundf(1000*out->accel_g[1]),
        lroundf(1000*out->accel_g[2]), lroundf(1000*out->gyro_rad_s[0]),
        lroundf(1000*out->gyro_rad_s[1]), lroundf(1000*out->gyro_rad_s[2]));
    if (length < 0 || length >= (int)sizeof(line) - 2) return;
#if use_filter
    {
        int extra = snprintf(line + length, sizeof(line) - length, ",%ld,%ld,%ld",
            lroundf(100*out->roll_deg), lroundf(100*out->pitch_deg),
            lroundf(100*out->yaw_deg));
        if (extra < 0 || extra >= (int)sizeof(line) - length - 2) return;
        length += extra;
    }
#endif
    line[length++] = '\r';
    line[length++] = '\n';
    for (i = 0; i < length; ++i) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)line[i];
    }
}

static bool calibrate(Attitude *out)
{
    ImuRaw raw;
    int32_t accel_sum[3] = {0}, gyro_sum[3] = {0};
    unsigned i, axis;
    for (i = 0; i < CALIBRATION_SAMPLES; ++i) {
        if (!mpu6050_read(&raw)) return false;
        for (axis = 0; axis < 3; ++axis) {
            accel_sum[axis] += raw.accel[axis];
            gyro_sum[axis] += raw.gyro[axis];
        }
    }
    for (axis = 0; axis < 3; ++axis) {
        gyro_bias[axis] = (float)gyro_sum[axis] / CALIBRATION_SAMPLES;
        out->accel_g[axis] = ((float)accel_sum[axis] / CALIBRATION_SAMPLES
                            - accel_bias[axis]) / accel_lsb_per_g[axis];
    }
#if use_filter
    out->q = initial_tilt(out->accel_g);
#endif
    return true;
}

static bool run_sensor(void)
{
    Attitude out = {0};
    ImuRaw raw;
    unsigned axis;
    uint32_t previous;
    imu_state = CONNECTING;
    if (!mpu6050_init()) return false;
    imu_state = CALIBRATING;
    if (!calibrate(&out)) return false;
    previous = micros();
    imu_state = RUNNING;
    while (mpu6050_read(&raw)) {
        uint32_t now = micros();
#if use_filter
        float sin_pitch;
        Quaternion q;
#endif
        out.dt_us = now - previous;
        previous = now;
        for (axis = 0; axis < 3; ++axis) {
            out.accel_g[axis] = (raw.accel[axis] - accel_bias[axis])
                              / accel_lsb_per_g[axis];
            out.gyro_rad_s[axis] = (raw.gyro[axis] - gyro_bias[axis])
                                 / (131.0f * RAD_TO_DEG);
        }
        /* Discard long gaps (e.g. debugger halt) instead of extrapolating. */
        if (out.dt_us > 50000u) continue;
#if use_filter
        madgwick_update(&out.q, out.accel_g, out.gyro_rad_s, BETA, out.dt_us * 1e-6f);
        q = out.q;
        out.roll_deg = RAD_TO_DEG * atan2f(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y));
        sin_pitch = 2*(q.w*q.y - q.z*q.x);
        if (sin_pitch > 1) sin_pitch = 1;
        if (sin_pitch < -1) sin_pitch = -1;
        out.pitch_deg = RAD_TO_DEG * asinf(sin_pitch);
        out.yaw_deg = RAD_TO_DEG * atan2f(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));
#endif
        attitude = out;
        ++sample_count;
        if ((sample_count & 1u) == 0) send_sample(now, &out);
    }
    return false;
}

int main(void)
{
    timer_init();
    uart_init();
    for (;;) {
        run_sensor();
        imu_state = CONNECTING;
        ++error_count;
        delay_us(250000);
    }
}
