// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Material.h"
#include "Math.h"
#include "Mesh.h"

#include <utils/Serialization.h>

namespace scene {

/**
 * @brief Shape types
 *
 */
enum class ShapeType
{
    Unknown,
    Mesh,
    Plane,
    Cube,
    Sphere,
    Cylinder,
    Capsule
};

/**
 * @brief Shape description
 *
 */
class Shape
{
  public:
    /**
     * @brief Construct a new Scene Object object
     *
     */
    Shape() : _type(ShapeType::Unknown) {}

    /**
     * @brief Construct a new Shape object
     *
     * @param mesh - mesh
     * @param matrix - shape poseation depending parent node
     */
    Shape(const Mesh& mesh, const Affine3f& pose, const Material& material)
        : _type(ShapeType::Mesh), _mesh(mesh), _pose(pose), _material(material)
    {
    }

    /**
     * @brief Construct a new Shape object
     *
     * @param type - shape type
     * @param matrix - shape poseation depending parent node
     */
    Shape(ShapeType type, const Affine3f& pose, const Material& material)
        : _type(type), _pose(pose), _material(material)
    {
    }

    /**
     * @brief Shape valid or not
     */
    bool valid() const { return _type != ShapeType::Unknown; }

    /**
     * @brief Shape type
     */
    ShapeType type() const { return _type; }

    /**
     * @brief Shape has specific type or not
     */
    bool hasType(ShapeType type) const { return _type == type; }

    /**
     * @brief Shape pose depending on a parent object
     */
    const Affine3f& pose() const { return _pose; }

    /**
     * @brief Associated material
     */
    Material& material() { return _material; }
    /** @overload */
    const Material& material() const { return _material; }

    /**
     * @brief Shape mesh description (only for ShapeType::Mesh shape)
     */
    const Mesh& mesh() const { return _mesh; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Shape& other) const
    {
        return _type == other._type && _pose == other._pose && _material == other._material &&
               _mesh == other._mesh;
    }
    bool operator!=(const Shape& other) const { return !(*this == other); }

  private:
    ShapeType _type;
    Affine3f _pose;
    Material _material;
    Mesh _mesh;

    NOP_STRUCTURE(Shape, _type, _pose, _material, _mesh);
};

} // namespace scene
