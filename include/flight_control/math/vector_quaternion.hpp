#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "flight_control/config.hpp"

namespace flight_control {

struct Vector3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vector3() = default;
    constexpr Vector3(float x_value, float y_value, float z_value)
        : x(x_value), y(y_value), z(z_value) {}
};

inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline Vector3 operator-(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

inline Vector3 operator*(const Vector3& value, float scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

inline Vector3 operator*(float scale, const Vector3& value) {
    return value * scale;
}

inline Vector3 operator/(const Vector3& value, float scale) {
    return {value.x / scale, value.y / scale, value.z / scale};
}

inline Vector3& operator+=(Vector3& lhs, const Vector3& rhs) {
    lhs = lhs + rhs;
    return lhs;
}

inline Vector3& operator-=(Vector3& lhs, const Vector3& rhs) {
    lhs = lhs - rhs;
    return lhs;
}

inline Vector3& operator*=(Vector3& lhs, float scale) {
    lhs = lhs * scale;
    return lhs;
}

inline float dot(const Vector3& lhs, const Vector3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline float norm_squared(const Vector3& value) {
    return dot(value, value);
}

inline float norm(const Vector3& value) {
    return std::sqrt(norm_squared(value));
}

inline Vector3 normalize_or_zero(const Vector3& value) {
    const float magnitude = norm(value);
    if (magnitude <= 1e-6f) {
        return {};
    }
    return value / magnitude;
}

inline Vector3 clamp_magnitude(const Vector3& value, float limit) {
    const float magnitude = norm(value);
    if (magnitude <= limit || magnitude <= 1e-6f) {
        return value;
    }
    return value * (limit / magnitude);
}

struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quaternion() = default;
    constexpr Quaternion(float w_value, float x_value, float y_value, float z_value)
        : w(w_value), x(x_value), y(y_value), z(z_value) {}
};

inline Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
    return {lhs.w + rhs.w, lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline Quaternion operator*(const Quaternion& value, float scale) {
    return {value.w * scale, value.x * scale, value.y * scale, value.z * scale};
}

inline Quaternion operator*(float scale, const Quaternion& value) {
    return value * scale;
}

inline Quaternion& operator+=(Quaternion& lhs, const Quaternion& rhs) {
    lhs = lhs + rhs;
    return lhs;
}

inline Quaternion normalize(const Quaternion& value) {
    const float magnitude = std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    if (magnitude <= 1e-6f) {
        return {};
    }
    return {value.w / magnitude, value.x / magnitude, value.y / magnitude, value.z / magnitude};
}

inline Quaternion conjugate(const Quaternion& value) {
    return {value.w, -value.x, -value.y, -value.z};
}

inline Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs) {
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

inline Quaternion from_euler_zyx(float roll, float pitch, float yaw) {
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);
    return normalize({
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    });
}

inline Vector3 to_euler_zyx(const Quaternion& quaternion) {
    const Quaternion q = normalize(quaternion);
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float roll = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    float pitch = 0.0f;
    if (std::fabs(sinp) >= 1.0f) {
        pitch = std::copysign(kPi * 0.5f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

inline Quaternion attitude_error(const Quaternion& target, const Quaternion& current) {
    Quaternion error = normalize(multiply(conjugate(target), current));
    if (error.w < 0.0f) {
        error = {-error.w, -error.x, -error.y, -error.z};
    }
    return error;
}

inline Vector3 rotate(const Quaternion& quaternion, const Vector3& vector) {
    const Quaternion pure{0.0f, vector.x, vector.y, vector.z};
    const Quaternion rotated = multiply(multiply(quaternion, pure), conjugate(quaternion));
    return {rotated.x, rotated.y, rotated.z};
}

inline Quaternion integrate_body_rates(const Quaternion& quaternion, const Vector3& body_rates, float dt) {
    const Quaternion omega{0.0f, body_rates.x, body_rates.y, body_rates.z};
    Quaternion next = quaternion + 0.5f * multiply(quaternion, omega) * dt;
    return normalize(next);
}

inline float wrap_angle(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}

}  // namespace flight_control
