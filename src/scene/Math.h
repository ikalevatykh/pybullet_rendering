// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/Serialization.h>

#include <array>
#include <tuple>

namespace scene {

// typedef std::tuple<int, int> Size2i;
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

} // namespace scene
