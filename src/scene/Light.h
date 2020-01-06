// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Math.h"
#include <utils/Serialization.h>

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
    Light() noexcept : _type(LightType::Unknown) {}

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
          _distance(distance)
    {
    }

    /**
     * @brief Light type
     */
    LightType type() const { return _type; }

    /**
     * @brief Light color
     */
    const Color3f& color() const { return _color; }
    /** @overload */
    void color(const Color3f& color) { _color = color; }

    /**
     * @brief Light direction vector in world coordinates (DirectionalLight)
     */
    const Vector3f& direction() const { return _direction; }
    /** @overload */
    void direction(const Vector3f& direction) { _direction = direction; }

    /**
     * @brief Light distance (DirectionalLight)
     */
    float distance() const { return _distance; }
    /** @overload */
    void distance(float distance) { _distance = distance; }

    /**
     * @brief Light position in world coordinates (PointLight)
     */
    const Vector3f& position() const { return _position; }
    /** @overload */
    void position(const Vector3f& position) { _position = position; }

    /**
     * @brief Light ambient coeffitient
     */
    float ambientCoeff() const { return _ambientCoeff; }
    /** @overload */
    void ambientCoeff(float coeff) { _ambientCoeff = coeff; }

    /**
     * @brief Light diffuse coeffitient
     */
    float diffuseCoeff() const { return _diffuseCoeff; }
    /** @overload */
    void diffuseCoeff(float coeff) { _diffuseCoeff = coeff; }

    /**
     * @brief Light ambient coeffitient
     */
    float specularCoeff() const { return _specularCoeff; }
    /** @overload */
    void specularCoeff(float coeff) { _specularCoeff = coeff; }

    /**
     * @brief Comparison operator
     */
    bool operator==(const Light& other) const
    {
        return _type == other._type && _direction == other._direction && _color == other._color &&
               _distance == other._distance && _position == other._position &&
               _ambientCoeff == other._ambientCoeff && _diffuseCoeff == other._diffuseCoeff &&
               _specularCoeff == other._specularCoeff;
    }
    bool operator!=(const Light& other) const { return !(*this == other); }

  private:
    LightType _type;
    Color3f _color;
    Vector3f _direction;
    Vector3f _position;
    float _distance;
    float _ambientCoeff;
    float _diffuseCoeff;
    float _specularCoeff;

    NOP_STRUCTURE(Light, _type, _color, _direction, _distance, _position, _ambientCoeff,
                  _diffuseCoeff, _specularCoeff);
};

} // namespace scene
