#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "flight_control/config.hpp"

namespace flight_control {

/**
 * 三维向量。
 *
 * 用于表示位置、速度、角速度、加速度和力矩等三轴物理量。
 */
struct Vector3 {
    /** x 分量。 */
    float x{0.0f};
    /** y 分量。 */
    float y{0.0f};
    /** z 分量。 */
    float z{0.0f};

    /** 构造零向量。 */
    constexpr Vector3() = default;
    /**
     * 构造三维向量。
     *
     * @param x_value x 分量。
     * @param y_value y 分量。
     * @param z_value z 分量。
     */
    constexpr Vector3(float x_value, float y_value, float z_value)
        : x(x_value), y(y_value), z(z_value) {}
};

/**
 * 向量加法。
 *
 * @param lhs 左操作数。
 * @param rhs 右操作数。
 * @return 两个向量逐分量相加的结果。
 */
inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

/**
 * 向量减法。
 *
 * @param lhs 左操作数。
 * @param rhs 右操作数。
 * @return 两个向量逐分量相减的结果。
 */
inline Vector3 operator-(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

/**
 * 向量按标量缩放。
 *
 * @param value 原始向量。
 * @param scale 缩放系数。
 * @return 缩放后的向量。
 */
inline Vector3 operator*(const Vector3& value, float scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

/**
 * 标量乘向量。
 *
 * @param scale 缩放系数。
 * @param value 原始向量。
 * @return 缩放后的向量。
 */
inline Vector3 operator*(float scale, const Vector3& value) {
    return value * scale;
}

/**
 * 向量除以标量。
 *
 * @param value 原始向量。
 * @param scale 除数。
 * @return 缩放后的向量。
 */
inline Vector3 operator/(const Vector3& value, float scale) {
    return {value.x / scale, value.y / scale, value.z / scale};
}

/**
 * 向量累加。
 *
 * @param lhs 被原地修改的左操作数。
 * @param rhs 被累加到 lhs 上的右操作数。
 * @return 修改后的 lhs 引用。
 */
inline Vector3& operator+=(Vector3& lhs, const Vector3& rhs) {
    lhs = lhs + rhs;
    return lhs;
}

/**
 * 向量累减。
 *
 * @param lhs 被原地修改的左操作数。
 * @param rhs 从 lhs 中减去的右操作数。
 * @return 修改后的 lhs 引用。
 */
inline Vector3& operator-=(Vector3& lhs, const Vector3& rhs) {
    lhs = lhs - rhs;
    return lhs;
}

/**
 * 向量按标量累乘。
 *
 * @param lhs 被原地缩放的向量。
 * @param scale 缩放系数。
 * @return 修改后的 lhs 引用。
 */
inline Vector3& operator*=(Vector3& lhs, float scale) {
    lhs = lhs * scale;
    return lhs;
}

/**
 * 计算两个向量点积。
 *
 * @param lhs 左向量。
 * @param rhs 右向量。
 * @return 点积结果。
 */
inline float dot(const Vector3& lhs, const Vector3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

/**
 * 计算向量长度平方。
 *
 * @param value 输入向量。
 * @return 长度平方。
 */
inline float norm_squared(const Vector3& value) {
    return dot(value, value);
}

/**
 * 计算向量长度。
 *
 * @param value 输入向量。
 * @return 向量模长。
 */
inline float norm(const Vector3& value) {
    return std::sqrt(norm_squared(value));
}

/**
 * 归一化向量，长度过小时返回零向量。
 *
 * @param value 输入向量。
 * @return 归一化结果或零向量。
 */
inline Vector3 normalize_or_zero(const Vector3& value) {
    /** 输入向量的当前模长。 */
    const float magnitude = norm(value);
    if (magnitude <= 1e-6f) {
        return {};
    }
    return value / magnitude;
}

/**
 * 把向量幅值限制在指定阈值内。
 *
 * @param value 输入向量。
 * @param limit 幅值上限。
 * @return 限幅后的向量。
 */
inline Vector3 clamp_magnitude(const Vector3& value, float limit) {
    /** 输入向量的当前模长。 */
    const float magnitude = norm(value);
    if (magnitude <= limit || magnitude <= 1e-6f) {
        return value;
    }
    return value * (limit / magnitude);
}

/**
 * 四元数。
 *
 * 用于描述姿态旋转和机体系到世界系的变换。
 */
struct Quaternion {
    /** 实部。 */
    float w{1.0f};
    /** i 分量。 */
    float x{0.0f};
    /** j 分量。 */
    float y{0.0f};
    /** k 分量。 */
    float z{0.0f};

    /** 构造单位四元数。 */
    constexpr Quaternion() = default;
    /**
     * 构造四元数。
     *
     * @param w_value 实部。
     * @param x_value i 分量。
     * @param y_value j 分量。
     * @param z_value k 分量。
     */
    constexpr Quaternion(float w_value, float x_value, float y_value, float z_value)
        : w(w_value), x(x_value), y(y_value), z(z_value) {}
};

/**
 * 四元数加法。
 *
 * @param lhs 左操作数。
 * @param rhs 右操作数。
 * @return 逐分量相加结果。
 */
inline Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
    return {lhs.w + rhs.w, lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

/**
 * 四元数按标量缩放。
 *
 * @param value 原始四元数。
 * @param scale 缩放系数。
 * @return 缩放后的四元数。
 */
inline Quaternion operator*(const Quaternion& value, float scale) {
    return {value.w * scale, value.x * scale, value.y * scale, value.z * scale};
}

/**
 * 标量乘四元数。
 *
 * @param scale 缩放系数。
 * @param value 原始四元数。
 * @return 缩放后的四元数。
 */
inline Quaternion operator*(float scale, const Quaternion& value) {
    return value * scale;
}

/**
 * 四元数累加。
 *
 * @param lhs 被原地修改的左操作数。
 * @param rhs 被累加到 lhs 上的右操作数。
 * @return 修改后的 lhs 引用。
 */
inline Quaternion& operator+=(Quaternion& lhs, const Quaternion& rhs) {
    lhs = lhs + rhs;
    return lhs;
}

/**
 * 归一化四元数。
 *
 * @param value 输入四元数。
 * @return 单位四元数；输入过小时返回默认四元数。
 */
inline Quaternion normalize(const Quaternion& value) {
    /** 输入四元数的当前模长。 */
    const float magnitude = std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    if (magnitude <= 1e-6f) {
        return {};
    }
    return {value.w / magnitude, value.x / magnitude, value.y / magnitude, value.z / magnitude};
}

/**
 * 计算四元数共轭。
 *
 * @param value 输入四元数。
 * @return 共轭四元数。
 */
inline Quaternion conjugate(const Quaternion& value) {
    return {value.w, -value.x, -value.y, -value.z};
}

/**
 * 四元数乘法。
 *
 * @param lhs 左四元数。
 * @param rhs 右四元数。
 * @return 乘积四元数。
 */
inline Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs) {
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

/**
 * 从 ZYX 欧拉角生成四元数。
 *
 * @param roll 横滚角，单位 rad。
 * @param pitch 俯仰角，单位 rad。
 * @param yaw 偏航角，单位 rad。
 * @return 对应姿态四元数。
 */
inline Quaternion from_euler_zyx(float roll, float pitch, float yaw) {
    /** 横滚半角的余弦。 */
    const float cr = std::cos(roll * 0.5f);
    /** 横滚半角的正弦。 */
    const float sr = std::sin(roll * 0.5f);
    /** 俯仰半角的余弦。 */
    const float cp = std::cos(pitch * 0.5f);
    /** 俯仰半角的正弦。 */
    const float sp = std::sin(pitch * 0.5f);
    /** 偏航半角的余弦。 */
    const float cy = std::cos(yaw * 0.5f);
    /** 偏航半角的正弦。 */
    const float sy = std::sin(yaw * 0.5f);
    return normalize({
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    });
}

/**
 * 把四元数转换为 ZYX 欧拉角。
 *
 * @param quaternion 输入四元数。
 * @return roll、pitch、yaw 三元组。
 */
inline Vector3 to_euler_zyx(const Quaternion& quaternion) {
    /** 归一化后的输入四元数。 */
    const Quaternion q = normalize(quaternion);
    /** 横滚角 atan2 的正弦项。 */
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    /** 横滚角 atan2 的余弦项。 */
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    /** 解算出的横滚角，单位 rad。 */
    const float roll = std::atan2(sinr_cosp, cosr_cosp);

    /** 俯仰角 asin 的输入项。 */
    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    /** 解算出的俯仰角，单位 rad。 */
    float pitch = 0.0f;
    if (std::fabs(sinp) >= 1.0f) {
        pitch = std::copysign(kPi * 0.5f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    /** 偏航角 atan2 的正弦项。 */
    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    /** 偏航角 atan2 的余弦项。 */
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    /** 解算出的偏航角，单位 rad。 */
    const float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

/**
 * 计算目标姿态与当前姿态之间的误差四元数。
 *
 * @param target 目标姿态。
 * @param current 当前姿态。
 * @return 姿态误差四元数。
 */
inline Quaternion attitude_error(const Quaternion& target, const Quaternion& current) {
    /** target 到 current 的短弧误差四元数。 */
    Quaternion error = normalize(multiply(conjugate(target), current));
    if (error.w < 0.0f) {
        error = {-error.w, -error.x, -error.y, -error.z};
    }
    return error;
}

/**
 * 使用四元数旋转向量。
 *
 * @param quaternion 旋转四元数。
 * @param vector 被旋转的向量。
 * @return 旋转后的向量。
 */
inline Vector3 rotate(const Quaternion& quaternion, const Vector3& vector) {
    /** 将三维向量嵌入四元数乘法的纯四元数。 */
    const Quaternion pure{0.0f, vector.x, vector.y, vector.z};
    /** 完成 q * v * q^-1 后的旋转结果。 */
    const Quaternion rotated = multiply(multiply(quaternion, pure), conjugate(quaternion));
    return {rotated.x, rotated.y, rotated.z};
}

/**
 * 根据机体系角速度对姿态进行积分。
 *
 * @param quaternion 当前姿态四元数。
 * @param body_rates 机体系角速度，单位 rad/s。
 * @param dt 积分步长，单位秒。
 * @return 积分后的姿态四元数。
 */
inline Quaternion integrate_body_rates(const Quaternion& quaternion, const Vector3& body_rates, float dt) {
    /** 机体系角速度对应的纯四元数。 */
    const Quaternion omega{0.0f, body_rates.x, body_rates.y, body_rates.z};
    /** 使用一阶积分得到的下一帧未归一化姿态。 */
    Quaternion next = quaternion + 0.5f * multiply(quaternion, omega) * dt;
    return normalize(next);
}

/**
 * 把角度归一化到 [-pi, pi]。
 *
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度。
 */
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
