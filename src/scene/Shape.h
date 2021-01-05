// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

#include "Material.h"
#include "Mesh.h"

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
    Shape() : _type(ShapeType::Unknown), _hasMaterial(false) {}

    /**
     * @brief Construct a new Shape object
     *
     * @param mesh - mesh
     * @param pose - shape position depending parent node
     * @param hasMaterial - shape has material
     * @param material - shape material
     */
    Shape(const Mesh& mesh, const Affine3f& pose, bool hasMaterial, const Material& material)
        : _type(ShapeType::Mesh), _mesh(mesh), _pose(pose), _hasMaterial(hasMaterial),
          _material(material)
    {
    }

    /**
     * @brief Construct a new Shape object
     *
     * @param type - shape type
     * @param matrix - shape position depending parent node
     */
    Shape(ShapeType type, const Affine3f& pose, const Material& material)
        : _type(type), _pose(pose), _hasMaterial(true), _material(material)
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
     * @brief Shape pose depending on a parent object
     */
    const Affine3f& pose() const { return _pose; }

    /**
     * @brief Shape mesh description (only for ShapeType::Mesh shape)
     */
    const Mesh& mesh() const { return _mesh; }

    /**
     * @brief If shape has material
     */
    bool hasMaterial() const { return _hasMaterial; }

    /**
     * @brief Associated material
     */
    const Material& material() const { return _material; }
    /** @overload */
    void setMaterial(const Material& material)
    {
        _hasMaterial = true;
        _material = material;
    }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Shape& other) const
    {
        return _type == other._type && _pose == other._pose && _mesh == other._mesh &&
               _hasMaterial == other._hasMaterial && _material == other._material;
    }
    bool operator!=(const Shape& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_type, _pose, _hasMaterial, _material, _mesh);
    }

  private:
    ShapeType _type;
    Affine3f _pose;
    bool _hasMaterial;
    Material _material;
    Mesh _mesh;
};

} // namespace scene
