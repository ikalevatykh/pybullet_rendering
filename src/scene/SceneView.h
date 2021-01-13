// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

#include "Camera.h"
#include "Light.h"

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
  public:
    /**
     * @brief Construct a new empty SceneView object
     *
     */
    SceneView() noexcept : _flags(0), _bg_texture(-1){};

    /**
     * @brief Flags
     */
    const int flags() const { return _flags; }
    /** @overload */
    void setFlags(int flags) { _flags = flags; }

    /**
     * @brief Backgroud color
     */
    const Color3f& backgroundColor() const { return _bg_color; }
    /** @overload */
    void setBackgroundColor(const Color3f& color) { _bg_color = color; }

    /**
     * @brief Backgroud texture
     */
    int backgroundTexture() const { return _bg_texture; }
    /** @overload */
    void setBackgroundTexture(int texture_id) { _bg_texture = texture_id; }

    /**
     * @brief Viewport dimentions
     */
    const Size2i& viewport() const { return _viewport; }
    /** @overload */
    void setViewport(const Size2i& s) { _viewport = s; }

    /**
     * @brief Camera parameters
     */
    const std::shared_ptr<Camera>& camera() const { return _camera; }
    /** @overload */
    void setCamera(const std::shared_ptr<Camera>& camera) { _camera = camera; }

    /**
     * @brief Light parameters
     */
    const std::shared_ptr<Light>& light() const { return _light; }
    /** @overload */
    void setLight(const std::shared_ptr<Light>& light) { _light = light; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const SceneView& other) const
    {
        return _viewport == other._viewport && _bg_color == other._bg_color &&
               _bg_texture == other._bg_texture && _flags == other._flags &&
               (_camera == other._camera ||
                _camera && other._camera && *_camera == *other._camera) &&
               (_light == other._light || _light && other._light && *_light == *other._light);
    }
    bool operator!=(const SceneView& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_viewport, _bg_color, _bg_texture, _flags, _camera, _light);
    }

  private:
    Size2i _viewport;
    Color3f _bg_color;
    int _bg_texture;
    int _flags;
    std::shared_ptr<Camera> _camera;
    std::shared_ptr<Light> _light;
    /** @todo: projective texture matrices */
};

} // namespace scene
