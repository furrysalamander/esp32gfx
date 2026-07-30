#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>

namespace esp32gfx {

template<std::floating_point T>
struct vec3 {
    T x{}, y{}, z{};

    constexpr vec3() = default;
    constexpr vec3(T x, T y, T z) : x(x), y(y), z(z) {}

    vec3 operator+(vec3 v) const { return {x + v.x, y + v.y, z + v.z}; }
    vec3 operator-(vec3 v) const { return {x - v.x, y - v.y, z - v.z}; }
    vec3 operator-() const { return {-x, -y, -z}; }
    vec3 operator*(T s) const { return {x * s, y * s, z * s}; }
    vec3 operator/(T s) const { return {x / s, y / s, z / s}; }

    vec3& operator+=(vec3 v) { x += v.x; y += v.y; z += v.z; return *this; }
    vec3& operator-=(vec3 v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    vec3& operator*=(T s) { x *= s; y *= s; z *= s; return *this; }

    T dot(vec3 v) const { return x * v.x + y * v.y + z * v.z; }

    vec3 cross(vec3 v) const {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }

    T length_sq() const { return x * x + y * y + z * z; }
    T length() const { return std::sqrt(length_sq()); }

    vec3 normalized() const {
        T len = length();
        if (len < T(1e-10)) return {};
        return *this / len;
    }
};

template<std::floating_point T>
struct vec4 {
    T x{}, y{}, z{}, w{1};

    constexpr vec4() = default;
    constexpr vec4(T x, T y, T z, T w = T(1)) : x(x), y(y), z(z), w(w) {}
};

template<std::floating_point T>
struct mat4 {
    T m[16]{};

    static mat4 identity() {
        mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1;
        return r;
    }

    static mat4 translate(T x, T y, T z) {
        mat4 r = identity();
        r.m[12] = x; r.m[13] = y; r.m[14] = z;
        return r;
    }

    static mat4 scale(T x, T y, T z) {
        mat4 r;
        r.m[0] = x; r.m[5] = y; r.m[10] = z; r.m[15] = 1;
        return r;
    }

    static mat4 rotate(T rad, T ax, T ay, T az) {
        T len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len < T(1e-10)) return identity();
        T x = ax / len, y = ay / len, z = az / len;
        T c = std::cos(rad), s = std::sin(rad), t = T(1) - c;

        mat4 r;
        r.m[0]  = t * x * x + c;
        r.m[1]  = t * x * y + s * z;
        r.m[2]  = t * x * z - s * y;
        r.m[5]  = t * y * y + c;
        r.m[6]  = t * y * z + s * x;
        r.m[10] = t * z * z + c;
        r.m[4]  = t * x * y - s * z;
        r.m[8]  = t * x * z + s * y;
        r.m[9]  = t * y * z - s * x;
        r.m[15] = 1;
        return r;
    }

    static mat4 look_at(vec3<T> eye, vec3<T> target, vec3<T> up) {
        vec3 f = (target - eye).normalized();
        vec3 s = f.cross(up).normalized();
        vec3 u = s.cross(f);

        mat4 r;
        r.m[0]  = s.x; r.m[4]  = s.y; r.m[8]  = s.z; r.m[12] = -s.dot(eye);
        r.m[1]  = u.x; r.m[5]  = u.y; r.m[9]  = u.z; r.m[13] = -u.dot(eye);
        r.m[2]  = -f.x; r.m[6] = -f.y; r.m[10] = -f.z; r.m[14] = f.dot(eye);
        r.m[15] = 1;
        return r;
    }

    static mat4 perspective(T fov_y, T aspect, T near, T far) {
        T f = T(1) / std::tan(fov_y * T(0.5));
        T range_inv = T(1) / (near - far);

        mat4 r;
        r.m[0]  = f / aspect;
        r.m[5]  = f;
        r.m[10] = (far + near) * range_inv;
        r.m[11] = T(-1);
        r.m[14] = T(2) * far * near * range_inv;
        return r;
    }

    static mat4 viewport(T x, T y, T w, T h) {
        mat4 r;
        r.m[0]  = w * T(0.5);
        r.m[5]  = -h * T(0.5);
        r.m[10] = T(0.5);
        r.m[12] = x + w * T(0.5);
        r.m[13] = y + h * T(0.5);
        r.m[14] = T(0.5);
        r.m[15] = 1;
        return r;
    }

    mat4 operator*(mat4 b) const {
        mat4 r;
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++)
                for (int k = 0; k < 4; k++)
                    r.m[col * 4 + row] += m[k * 4 + row] * b.m[col * 4 + k];
        return r;
    }

    vec4<T> operator*(vec4<T> v) const {
        return {
            m[0] * v.x + m[4] * v.y + m[8]  * v.z + m[12] * v.w,
            m[1] * v.x + m[5] * v.y + m[9]  * v.z + m[13] * v.w,
            m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w,
            m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w
        };
    }

    vec3<T> operator*(vec3<T> v) const {
        vec4 r = *this * vec4(v.x, v.y, v.z, T(1));
        return {r.x, r.y, r.z};
    }
};

template<std::floating_point T>
struct quat {
    T x{}, y{}, z{}, w{1};

    constexpr quat() = default;
    constexpr quat(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}

    static quat from_euler(T roll, T pitch, T yaw) {
        T cr = std::cos(roll * T(0.5)), sr = std::sin(roll * T(0.5));
        T cp = std::cos(pitch * T(0.5)), sp = std::sin(pitch * T(0.5));
        T cy = std::cos(yaw * T(0.5)),   sy = std::sin(yaw * T(0.5));

        return {
            sr * cp * cy + cr * sp * sy,
            cr * sp * cy - sr * cp * sy,
            cr * cp * sy + sr * sp * cy,
            cr * cp * cy - sr * sp * sy
        };
    }

    quat operator*(quat q) const {
        return {
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z
        };
    }

    quat normalized() const {
        T len = std::sqrt(x * x + y * y + z * z + w * w);
        if (len < T(1e-10)) return {T(0), T(0), T(0), T(1)};
        return {x / len, y / len, z / len, w / len};
    }

    mat4<T> to_mat4() const {
        T xx = x * x, yy = y * y, zz = z * z;
        T xy = x * y, xz = x * z, xw = x * w;
        T yz = y * z, yw = y * w, zw = z * w;

        mat4<T> r;
        r.m[0]  = T(1) - T(2) * (yy + zz);
        r.m[1]  = T(2) * (xy + zw);
        r.m[2]  = T(2) * (xz - yw);
        r.m[4]  = T(2) * (xy - zw);
        r.m[5]  = T(1) - T(2) * (xx + zz);
        r.m[6]  = T(2) * (yz + xw);
        r.m[8]  = T(2) * (xz + yw);
        r.m[9]  = T(2) * (yz - xw);
        r.m[10] = T(1) - T(2) * (xx + yy);
        r.m[15] = 1;
        return r;
    }
};

using vec3f = vec3<float>;
using vec4f = vec4<float>;
using mat4f = mat4<float>;
using quatf = quat<float>;

} // namespace esp32gfx
