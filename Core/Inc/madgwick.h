#ifndef MADGWICK_H
#define MADGWICK_H

typedef struct { float w, x, y, z; } Quaternion;
Quaternion initial_tilt(const float accel[3]);
void madgwick_update(Quaternion *q, const float accel[3], const float gyro[3],
                     float beta, float dt);
#endif
