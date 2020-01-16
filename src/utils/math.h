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
    Quaternionf quaternion;
    Vector3f scale;

    static constexpr Affine3f Identity() { return Affine3f{{0, 0, 0}, {1, 0, 0, 0}, {1, 1, 1}}; }

    Vector3f euler() const
    {
        const float qw = quaternion[0], qx = quaternion[1], //
            qy = quaternion[2], qz = quaternion[3];

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

    /**
     * @brief Comparison operators
     */
    bool operator==(const Affine3f& other) const
    {
        return origin == other.origin && quaternion == other.quaternion && scale == other.scale;
    }
    bool operator!=(const Affine3f& other) const { return !(*this == other); }

  private:
    NOP_STRUCTURE(Affine3f, origin, quaternion, scale);
};
