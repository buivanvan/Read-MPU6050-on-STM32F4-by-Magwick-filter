#include "madgwick.h"
#include <math.h>

Quaternion initial_tilt(const float a[3])
{
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    Quaternion q = {cr*cp, sr*cp, cr*sp, -sr*sp};
    return q;
}

void madgwick_update(Quaternion *q, const float a[3], const float g[3],
                     float beta, float dt)
{
    float w = q->w, x = q->x, y = q->y, z = q->z;
    float dw = 0.5f * (-x*g[0] - y*g[1] - z*g[2]);
    float dx = 0.5f * ( w*g[0] + y*g[2] - z*g[1]);
    float dy = 0.5f * ( w*g[1] - x*g[2] + z*g[0]);
    float dz = 0.5f * ( w*g[2] + x*g[1] - y*g[0]);
    float norm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (norm > 1e-6f) {
        float fx = 2*(x*z - w*y) - a[0]/norm;
        float fy = 2*(w*x + y*z) - a[1]/norm;
        float fz = 1 - 2*(x*x + y*y) - a[2]/norm;
        /* J transpose times the gravity residual. */
        float sw = -2*y*fx + 2*x*fy;
        float sx =  2*z*fx + 2*w*fy - 4*x*fz;
        float sy = -2*w*fx + 2*z*fy - 4*y*fz;
        float sz =  2*x*fx + 2*y*fy;
        float length = sqrtf(sw*sw + sx*sx + sy*sy + sz*sz);
        if (length > 1e-6f) {
            float gain = beta / length;
            dw -= gain*sw;
            dx -= gain*sx;
            dy -= gain*sy;
            dz -= gain*sz;
        }
    }
    w += dw*dt;
    x += dx*dt;
    y += dy*dt;
    z += dz*dt;
    norm = sqrtf(w*w + x*x + y*y + z*z);
    q->w = w/norm;
    q->x = x/norm;
    q->y = y/norm;
    q->z = z/norm;
}
