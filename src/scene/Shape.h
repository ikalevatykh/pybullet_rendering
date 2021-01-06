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
    Capsule,
    Heightfield
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
    Shape() noexcept : _type(ShapeType::Unknown) {}

    /**
     * @brief Construct a new Shape object
     *
     * @param type - shape type
     * @param pose - shape position depending parent node
     * @param material - shape material
     * @param mesh - shape mesh (optional)
     */
    Shape(ShapeType type, const Affine3f& pose, const std::shared_ptr<Material>& material,
          const std::shared_ptr<Mesh>& mesh = nullptr)
        : _type(type), _pose(pose), _material(material), _mesh(mesh)
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
    const std::shared_ptr<Mesh>& mesh() const { return _mesh; }

    /**
     * @brief If shape has material
     */
    bool hasMaterial() const { return !!_material; }

    /**
     * @brief Associated material
     */
    const std::shared_ptr<Material>& material() const { return _material; }
    /** @overload */
    void setMaterial(const std::shared_ptr<Material>& material) { _material = material; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Shape& other) const
    {
        return _type == other._type && _pose == other._pose &&
               (_material == other._material ||
                _material && other._material && *_material == *other._material) &&
               (_mesh == other._mesh || _mesh && other._mesh && *_mesh == *other._mesh);
    }
    bool operator!=(const Shape& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_type, _pose, _material, _mesh);
    }

  private:
    ShapeType _type;
    Affine3f _pose;
    std::shared_ptr<Material> _material;
    std::shared_ptr<Mesh> _mesh;
};

} // namespace scene
