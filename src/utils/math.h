// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "serialization.h"

#include <array>
#include <cmath>

typedef std::array<int, 2> Size2i;
typedef std::array<float, 3> Vector3f;
typedef std::array<float, 4> Vector4f;
typedef std::array<float, 4 * 4> Matrix4f;
typedef Vector4f Quaternionf;
typedef Vector3f Color3f;
typedef Vector4f Color4f;

/**
 * @brief Affine transform in 3D
 */
struct Affine3f {
    Vector3f origin;
    Quaternionf quat;
    Vector3f scale;

    static constexpr Affine3f Identity() { return Affine3f{{0, 0, 0}, {1, 0, 0, 0}, {1, 1, 1}}; }

    inline Vector3f euler() const
    {
        const auto qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (qw * qy - qz * qx);
        double pitch = (std::abs(sinp) >= 1) //
                           ? std::copysign(M_PI / 2, sinp)
                           : std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {float(yaw), float(roll), float(pitch)};
    }

    inline Matrix4f matrix() const
    {
        const auto qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];

        const auto m00 = 1 - 2 * (qy * qy + qz * qz);
        const auto m01 = 2 * (qx * qy - qz * qw);
        const auto m02 = 2 * (qx * qz + qy * qw);

        const auto m10 = 2 * (qx * qy + qz * qw);
        const auto m11 = 1 - 2 * (qx * qx + qz * qz);
        const auto m12 = 2 * (qy * qz - qx * qw);

        const auto m20 = 2 * (qx * qz - qy * qw);
        const auto m21 = 2 * (qy * qz + qx * qw);
        const auto m22 = 1 - 2 * (qx * qx + qy * qy);

        return {m00 * scale[0], m10 * scale[1], m20 * scale[2], 0.f,
                m01 * scale[0], m11 * scale[1], m21 * scale[2], 0.f,
                m02 * scale[0], m12 * scale[1], m22 * scale[2], 0.f,
                origin[0],      origin[1],      origin[2],      1.f};
    }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Affine3f& other) const
    {
        return origin == other.origin && quat == other.quat && scale == other.scale;
    }
    bool operator!=(const Affine3f& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(origin, quat, scale);
    }
};
