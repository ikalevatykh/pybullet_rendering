// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Texture.h"
#include <utils/math.h>

namespace scene {

/**
 * @brief Material description
 *
 */
class Material
{
  public:
    /**
     * @brief Construct a new Material object
     *
     */
    Material() noexcept : _diffuseColor{0, 0, 0}, _specularColor{0, 0, 0}, _diffuseTexture(-1){};

    /**
     * @brief Construct a new Material object
     *
     * @param diffuseColor - diffuse color
     * @param specularColor - specular color
     * @param texture - texture
     */
    Material(const Color4f& diffuse, const Color3f& specular, int textureId = -1)
        : _diffuseColor(diffuse), _specularColor(specular), _diffuseTexture(textureId)
    {
    }

    /**
     * @brief Diffuse color
     */
    const Color4f& diffuseColor() const { return _diffuseColor; }
    /** @overload */
    void setDiffuseColor(const Color4f& color) { _diffuseColor = color; }

    /**
     * @brief Specular color
     */
    const Color3f& specularColor() const { return _specularColor; }
    /** @overload */
    void setSpecularColor(const Color3f& color) { _specularColor = color; }

    /**
     * @brief Diffuse texture
     */
    const int diffuseTexture() const { return _diffuseTexture; }
    /** @overload */
    void setDiffuseTexture(int textureId) { _diffuseTexture = textureId; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Material& other) const
    {
        return _diffuseColor == other._diffuseColor && _specularColor == other._specularColor &&
               _diffuseTexture == other._diffuseTexture;
    }
    bool operator!=(const Material& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_diffuseColor, _specularColor, _diffuseTexture);
    }

  private:
    Color4f _diffuseColor;
    Color3f _specularColor;
    int _diffuseTexture;
};

} // namespace scene
