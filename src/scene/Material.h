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
    Material() noexcept : _diffuseColor{0, 0, 0}, _specularColor{0, 0, 0} {};

    /**
     * @brief Construct a new Material object
     *
     * @param diffuseColor - diffuse color
     * @param specularColor - specular color
     * @param texture - texture
     */
    Material(const Color4f& diffuse, const Color3f& specular,
             const std::shared_ptr<Texture>& texture = nullptr)
        : _diffuseColor(diffuse), _specularColor(specular), _texture(texture)
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
    const std::shared_ptr<Texture> diffuseTexture() const { return _texture; }
    /** @overload */
    void setDiffuseTexture(const std::shared_ptr<Texture>& texture) { _texture = texture; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Material& other) const
    {
        return _diffuseColor == other._diffuseColor && _specularColor == other._specularColor &&
               (_texture == other._texture ||
                _texture && other._texture && *_texture == *other._texture);
    }
    bool operator!=(const Material& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_diffuseColor, _specularColor, _texture);
    }

  private:
    Color4f _diffuseColor;
    Color3f _specularColor;
    std::shared_ptr<Texture> _texture;
};

} // namespace scene
