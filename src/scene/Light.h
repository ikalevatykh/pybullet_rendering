// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>
#include <utils/serialization.h>

namespace scene {

/**
 * @brief Light types
 *
 */
enum class LightType
{
    Unknown,
    AmbientLight,
    DirectionalLight,
    PointLight,
};

/**
 * @brief Light configuration
 *
 */
class Light
{
  public:
    /**
     * @brief Construct a new Light object
     *
     */
    Light() noexcept {}

    /**
     * @brief Construct a new Light object
     *
     */
    Light(LightType type) : _type(type) {}

    /**
     * @brief Construct a new Directional Light
     *
     */
    Light(const Color3f& color, const Vector3f& direction, float distance)
        : _type(LightType::DirectionalLight), _color(color), _direction(direction),
          _distance(distance), _position{0, 0, 0}
    {
    }

    /**
     * @brief Light type
     */
    LightType type() const { return _type; }
    /** @overload */
    void setType(LightType type) { _type = type; }

    /**
     * @brief Light color
     */
    const Color3f& color() const { return _color; }
    /** @overload */
    void setColor(const Color3f& color) { _color = color; }

    /**
     * @brief Light direction vector in world coordinates (DirectionalLight)
     */
    const Vector3f& direction() const { return _direction; }
    /** @overload */
    void setDirection(const Vector3f& direction) { _direction = direction; }

    /**
     * @brief Light distance (DirectionalLight)
     */
    float distance() const { return _distance; }
    /** @overload */
    void setDistance(float distance) { _distance = distance; }

    /**
     * @brief Light position in world coordinates (PointLight)
     */
    const Vector3f& position() const { return _position; }
    /** @overload */
    void setPosition(const Vector3f& position) { _position = position; }

    /**
     * @brief Light ambient coeffitient
     */
    float ambientCoeff() const { return _ambientCoeff; }
    /** @overload */
    void setAmbientCoeff(float coeff) { _ambientCoeff = coeff; }

    /**
     * @brief Light diffuse coeffitient
     */
    float diffuseCoeff() const { return _diffuseCoeff; }
    /** @overload */
    void setDiffuseCoeff(float coeff) { _diffuseCoeff = coeff; }

    /**
     * @brief Light specular coeffitient
     */
    float specularCoeff() const { return _specularCoeff; }
    /** @overload */
    void setSpecularCoeff(float coeff) { _specularCoeff = coeff; }

    /**
     * @brief Flag indicating whether this light should cast shadows or not
     */
    bool isShadowCaster() const { return _isShadowCaster; }
    /** @overload */
    void shadowCaster(bool caster) { _isShadowCaster = caster; }

    /**
     * @brief Light ambient color
     */
    Color3f ambientColor() const
    {
        return {_color[0] * _ambientCoeff, _color[1] * _ambientCoeff, _color[2] * _ambientCoeff};
    }

    /**
     * @brief Light diffuse color
     */
    Color3f diffuseColor() const
    {
        return {_color[0] * _diffuseCoeff, _color[1] * _diffuseCoeff, _color[2] * _diffuseCoeff};
    }

    /**
     * @brief Light specular color
     */
    Color3f specularColor() const
    {
        return {_color[0] * _specularCoeff, _color[1] * _specularCoeff, _color[2] * _specularCoeff};
    }

    /**
     * @brief Comparison operator
     */
    bool operator==(const Light& other) const
    {
        return _type == other._type && _direction == other._direction && _color == other._color &&
               _distance == other._distance && _position == other._position &&
               _ambientCoeff == other._ambientCoeff && _diffuseCoeff == other._diffuseCoeff &&
               _specularCoeff == other._specularCoeff && _isShadowCaster == other._isShadowCaster;
    }
    bool operator!=(const Light& other) const { return !(*this == other); }

  private:
    LightType _type = LightType::Unknown;
    Color3f _color = Color3f{0.f, 0.f, 0.f};
    Vector3f _direction = Vector3f{0.f, 0.f, 0.f};
    Vector3f _position = Vector3f{0.f, 0.f, 0.f};
    float _distance = 0.f;
    float _ambientCoeff = 1.f;
    float _diffuseCoeff = 1.f;
    float _specularCoeff = 1.f;
    bool _isShadowCaster = false;

    NOP_STRUCTURE(Light, _type, _color, _direction, _position, _distance, _ambientCoeff,
                  _diffuseCoeff, _specularCoeff, _isShadowCaster);
};

} // namespace scene
