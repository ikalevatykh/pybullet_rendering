// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Camera.h"
#include "Light.h"
#include "Math.h"
#include <utils/Serialization.h>

namespace scene {

/**
 * @brief View configuration
 *
 * One scene you can draw many times using view configurations
 * with different camera, light and other settings.
 *
 */
class SceneView
{
    /**
     * @brief Scene view configure flags
     *
     */
    enum Flags
    {
        ShadowFlag = 1 << 0, /**< Draw shadows */
        ColorFlag = 1 << 1, /**< Render to color channel  */
        DepthFlag = 1 << 2, /**< Render to depth channel  */
        MaskFlag = 1 << 3, /**< Render to mask channel  */
    };

  public:
    /**
     * @brief Construct a new empty SceneView object
     *
     */
    SceneView() noexcept : _flags(0){};

    /**
     * @brief Backgroud color
     */
    const Color3f& backgroundColor() const { return _background; }
    /** @overload */
    void backgroundColor(const Color3f& color) { _background = color; }

    /**
     * @brief Viewport dimentions
     */
    const Size2i& viewport() const { return _viewport; }
    /** @overload */
    void viewport(const Size2i& s) { _viewport = s; }

    /**
     * @brief Camera parameters
     */
    const Camera& camera() const { return _camera; }
    /** @overload */
    void camera(const Camera& camera) { _camera = camera; }

    /**
     * @brief Light parameters
     */
    const Light& light() const { return _light; }
    /** @overload */
    void light(const Light& light) { _light = light; }

    /**
     * @brief Enable data planes to render
     */
    void enablePlanes(bool enableColor, bool enableDepth, bool enableMask)
    {
        _flags = enableColor ? _flags | ColorFlag : _flags & ~ColorFlag;
        _flags = enableDepth ? _flags | DepthFlag : _flags & ~DepthFlag;
        _flags = enableMask ? _flags | MaskFlag : _flags & ~MaskFlag;
    }

    /**
     * @brief Is color plane rendering enabled
     */
    bool isColorEnabled() const { return bool(_flags & ColorFlag); }

    /**
     * @brief Is depth plane rendering enabled
     */
    bool isDepthEnabled() const { return bool(_flags & DepthFlag); }

    /**
     * @brief Is mask plane rendering enabled
     */
    bool isMaskEnabled() const { return bool(_flags & MaskFlag); }

    /**
     * @brief Enable shadows or not
     */
    void enableShadow(bool enable) { _flags = enable ? _flags | ShadowFlag : _flags & ~ShadowFlag; }
    /** @overload */
    bool isShadowEnabled() const { return bool(_flags & ShadowFlag); }

    /**
     * @brief Resets view configuration
     */
    void reset() { _flags = 0; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const SceneView& other) const
    {
        return _background == other._background && _camera == other._camera &&
               _flags == other._flags && _light == other._light && _viewport == other._viewport;
    }
    bool operator!=(const SceneView& other) const { return !(*this == other); }

  private:
    Color3f _background;
    Camera _camera;
    Light _light;
    Size2i _viewport;
    int _flags;
    /** @todo: projective texture matrices */

    NOP_STRUCTURE(SceneView, _background, _camera, _light, _viewport, _flags);
};

} // namespace scene
