#include "esp32gfx/math.hpp"

namespace esp32gfx {

mat4 mat4::rotate(float rad, float ax, float ay, float az) {
    float len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len < 1e-10f) return identity();
    float x = ax / len, y = ay / len, z = az / len;
    float c = std::cos(rad), s = std::sin(rad), t = 1 - c;

    mat4 r;
    r.m[0]  = t * x * x + c;
    r.m[1]  = t * x * y + s * z;
    r.m[2]  = t * x * z - s * y;
    r.m[3]  = 0;
    r.m[4]  = t * x * y - s * z;
    r.m[5]  = t * y * y + c;
    r.m[6]  = t * y * z + s * x;
    r.m[7]  = 0;
    r.m[8]  = t * x * z + s * y;
    r.m[9]  = t * y * z - s * x;
    r.m[10] = t * z * z + c;
    r.m[11] = 0;
    r.m[12] = 0;
    r.m[13] = 0;
    r.m[14] = 0;
    r.m[15] = 1;
    return r;
}

mat4 mat4::look_at(vec3 eye, vec3 target, vec3 up) {
    vec3 f = (target - eye).normalized();
    vec3 s = f.cross(up).normalized();
    vec3 u = s.cross(f);

    mat4 r;
    r.m[0]  = s.x;  r.m[4]  = s.y;  r.m[8]  = s.z;  r.m[12] = -s.dot(eye);
    r.m[1]  = u.x;  r.m[5]  = u.y;  r.m[9]  = u.z;  r.m[13] = -u.dot(eye);
    r.m[2]  = -f.x; r.m[6]  = -f.y; r.m[10] = -f.z; r.m[14] =  f.dot(eye);
    r.m[3]  = 0;    r.m[7]  = 0;    r.m[11] = 0;    r.m[15] = 1;
    return r;
}

mat4 mat4::perspective(float fov_y, float aspect, float near, float far) {
    float f = 1.0f / std::tan(fov_y * 0.5f);
    float range_inv = 1.0f / (near - far);

    mat4 r;
    for (int i = 0; i < 16; i++) r.m[i] = 0;
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    r.m[10] = (far + near) * range_inv;
    r.m[11] = 2.0f * far * near * range_inv;
    r.m[14] = -1;
    return r;
}

mat4 mat4::viewport(float x, float y, float w, float h) {
    mat4 r;
    for (int i = 0; i < 16; i++) r.m[i] = 0;
    r.m[0]  = w * 0.5f;
    r.m[5]  = -h * 0.5f;
    r.m[10] = 0.5f;
    r.m[12] = x + w * 0.5f;
    r.m[13] = y + h * 0.5f;
    r.m[14] = 0.5f;
    r.m[15] = 1;
    return r;
}

mat4 mat4::operator*(mat4 b) const {
    mat4 r;
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0;
            for (int k = 0; k < 4; k++)
                sum += m[k * 4 + row] * b.m[col * 4 + k];
            r.m[col * 4 + row] = sum;
        }
    }
    return r;
}

quat quat::from_euler(float roll, float pitch, float yaw) {
    float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    float cy = std::cos(yaw * 0.5f),   sy = std::sin(yaw * 0.5f);

    return {
        sr * cp * cy + cr * sp * sy,
        cr * sp * cy - sr * cp * sy,
        cr * cp * sy + sr * sp * cy,
        cr * cp * cy - sr * sp * sy
    };
}

mat4 quat::to_mat4() const {
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, xw = x * w;
    float yz = y * z, yw = y * w, zw = z * w;

    mat4 r;
    r.m[0]  = 1 - 2 * (yy + zz);
    r.m[1]  = 2 * (xy + zw);
    r.m[2]  = 2 * (xz - yw);
    r.m[3]  = 0;
    r.m[4]  = 2 * (xy - zw);
    r.m[5]  = 1 - 2 * (xx + zz);
    r.m[6]  = 2 * (yz + xw);
    r.m[7]  = 0;
    r.m[8]  = 2 * (xz + yw);
    r.m[9]  = 2 * (yz - xw);
    r.m[10] = 1 - 2 * (xx + yy);
    r.m[11] = 0;
    r.m[12] = 0;
    r.m[13] = 0;
    r.m[14] = 0;
    r.m[15] = 1;
    return r;
}

} // namespace esp32gfx
