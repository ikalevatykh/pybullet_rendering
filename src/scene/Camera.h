// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

namespace scene {

/**
 * @brief Camera configuration
 *
 */
class Camera
{
  public:
    /**
     * @brief Construct a new Camera object
     *
     */
    Camera() noexcept : _viewMatrix{0}, _projMatrix{0} {}

    /**
     * @brief Construct a new Camera object
     *
     */
    Camera(const Matrix4f& viewMatrix, const Matrix4f& projMatrix)
        : _viewMatrix{viewMatrix}, _projMatrix{projMatrix}
    {
    }

    /**
     * @brief Camera projection 4x4 matrix
     */
    const Matrix4f& projMatrix() const { return _projMatrix; }
    /** @overload */
    void setProjMatrix(const Matrix4f& m) { _projMatrix = m; }

    /**
     * @brief Camera view 4x4 matrix
     */
    const Matrix4f& viewMatrix() const { return _viewMatrix; }
    /** @overload */
    void setViewMatrix(const Matrix4f& m) { _viewMatrix = m; }

    /**
     * @brief Camera transformation 4x4 matrix
     *
     * This matrix is equal to inv(viewMatrix)
     */
    inline Matrix4f poseMatrix() const
    {
        const auto m00 = _viewMatrix[0], m01 = _viewMatrix[1], m02 = _viewMatrix[2];
        const auto m10 = _viewMatrix[4], m11 = _viewMatrix[5], m12 = _viewMatrix[6];
        const auto m20 = _viewMatrix[8], m21 = _viewMatrix[9], m22 = _viewMatrix[10];
        const auto p0 = _viewMatrix[12], p1 = _viewMatrix[13], p2 = _viewMatrix[14];

        const auto h0 = -m00 * p0 - m01 * p1 - m02 * p2;
        const auto h1 = -m10 * p0 - m11 * p1 - m12 * p2;
        const auto h2 = -m20 * p0 - m21 * p1 - m22 * p2;

        return {m00, m10, m20, 0.f, m01, m11, m21, 0.f, m02, m12, m22, 0.f, h0, h1, h2, 1.f};
    }

    /**
     * @brief Comparison operator
     */
    bool operator==(const Camera& other) const
    {
        return _projMatrix == other._projMatrix && _viewMatrix == other._viewMatrix;
    }
    bool operator!=(const Camera& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_projMatrix, _viewMatrix);
    }

  private:
    Matrix4f _projMatrix;
    Matrix4f _viewMatrix;
};

} // namespace scene
