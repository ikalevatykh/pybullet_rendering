// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Math.h"
#include <utils/Serialization.h>

namespace scene {

/**
 * @brief Camera configuration
 *
 */
class Camera
{
  public:
    /**
     * @brief Camera projection 4x4 matrix.
     */
    const Matrix4f& projMatrix() const { return _projMatrix; }
    /** @overload */
    void projMatrix(const Matrix4f& m) { _projMatrix = m; }

    /**
     * @brief Camera view 4x4 matrix.
     */
    const Matrix4f& viewMatrix() const { return _viewMatrix; }
    /** @overload */
    void viewMatrix(const Matrix4f& m) { _viewMatrix = m; }

    /**
     * @brief Comparison operator
     */
    bool operator==(const Camera& other) const
    {
        return _projMatrix == other._projMatrix && _viewMatrix == other._viewMatrix;
    }
    bool operator!=(const Camera& other) const { return !(*this == other); }

  private:
    Matrix4f _projMatrix;
    Matrix4f _viewMatrix;

    NOP_STRUCTURE(Camera, _projMatrix, _viewMatrix);
};

} // namespace scene
