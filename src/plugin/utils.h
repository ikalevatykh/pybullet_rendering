// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// project imports
#include <scene/SceneGraph.h>
#include <utils/math.h>

// bullet imports
#include <Importers/ImportURDFDemo/UrdfParser.h>
#include <SharedMemory/SharedMemoryPublic.h>

// std imports
#include <cstring>

/**
 * @brief Convert PyBullet's transform and scale to Affine3f
 *
 * @param pose - pose to update
 * @param frame - transformation
 * @param scale - scale vector
 */
inline Affine3f makePose(const btTransform& frame, const btVector3& scale = {1.0, 1.0, 1.0})
{
    const auto& origin = frame.getOrigin();
    const auto& basis = frame.getBasis();

    btQuaternion quat;
    basis.getRotation(quat);

    Affine3f pose;
    pose.origin = {float(origin.x()), float(origin.y()), float(origin.z())};
    pose.quat = {float(quat.getW()), float(quat.x()), float(quat.y()), float(quat.z())};
    pose.scale = {float(scale.x()), float(scale.y()), float(scale.z())};
    return pose;
}

/**
 * @brief Convert geometry to MeshData
 *
 * @param geometry - geometry containing the mesh data
 * @return std::shared_ptr<scene::MeshData>
 */
inline std::shared_ptr<scene::MeshData> getMeshData(const UrdfGeometry& geometry)
{
    std::vector<float> vertices;
    std::vector<float> uvs;
    std::vector<float> normals;
    std::vector<int> indices;

    const int verticesCount = geometry.m_vertices.size();
    const int uvsCount = geometry.m_uvs.size();
    const int normalsCount = geometry.m_normals.size();
    const int indicesCount = geometry.m_indices.size();

    vertices.reserve(verticesCount * 3);
    uvs.reserve(uvsCount * 2);
    normals.reserve(normalsCount * 3);
    indices.reserve(indicesCount);

    for (int i = 0; i < verticesCount; ++i) {
        const auto& vertex = geometry.m_vertices[i];
        vertices.push_back(float(vertex.x()));
        vertices.push_back(float(vertex.y()));
        vertices.push_back(float(vertex.z()));
    }
    for (int i = 0; i < uvsCount; ++i) {
        const auto& uv = geometry.m_uvs[i];
        uvs.push_back(float(uv.x()));
        uvs.push_back(float(uv.y()));
    }
    for (int i = 0; i < normalsCount; ++i) {
        const auto& normal = geometry.m_normals[i];
        normals.push_back(float(normal.x()));
        normals.push_back(float(normal.y()));
        normals.push_back(float(normal.z()));
    }
    for (int i = 0; i < indicesCount; ++i) {
        indices.push_back(geometry.m_indices[i]);
    }
    return std::make_shared<scene::MeshData>(std::move(vertices), std::move(uvs),
                                             std::move(normals), std::move(indices));
}

/**
 * @brief Convert URDF shape to internal scene::Shape description
 *
 * @param urdfShape - pybullet URDF shape description
 * @param urdfMaterial - pybullet URDF material description
 * @param localInertiaFrame - link inertia frame
 * @param flags - URDF loading options
 * @return scene::Shape
 */
inline scene::Shape makeShape(const UrdfShape& urdfShape, const UrdfMaterial& urdfMaterial,
                              const btTransform& localInertiaFrame, int flags)
{
    using namespace scene;
    auto frame = localInertiaFrame.inverse() * urdfShape.m_linkLocalFrame;

    const auto& filename = urdfMaterial.m_textureFilename;
    const auto& diff = urdfMaterial.m_matColor.m_rgbaColor;
    const auto& spec = urdfMaterial.m_matColor.m_specularColor;

    auto material = std::make_shared<Material>( //
        Color4f{float(diff[0]), float(diff[1]), float(diff[2]), float(diff[3])},
        Color3f{float(spec[0]), float(spec[1]), float(spec[2])},
        filename.empty() ? nullptr : std::make_shared<Texture>(filename));

    const auto& geometry = urdfShape.m_geometry;
    if (URDF_GEOM_BOX == geometry.m_type) {
        const auto pose = makePose(frame);
        const auto dx = float(geometry.m_boxSize.x());
        const auto dy = float(geometry.m_boxSize.y());
        const auto dz = float(geometry.m_boxSize.z());
        return Shape{ShapeType::Cube, pose, {dx, dy, dz}, material};
    }
    else if (URDF_GEOM_SPHERE == geometry.m_type) {
        const auto pose = makePose(frame);
        const auto radius = float(geometry.m_sphereRadius);
        return Shape{ShapeType::Sphere, pose, {radius}, material};
    }
    else if (URDF_GEOM_CYLINDER == geometry.m_type) {
        const auto pose = makePose(frame);
        const auto radius = float(geometry.m_capsuleRadius);
        const auto height = float(geometry.m_capsuleHeight);
        return Shape{ShapeType::Cylinder, pose, {radius, height}, material};
    }
    else if (URDF_GEOM_CAPSULE == geometry.m_type) {
        const auto pose = makePose(frame);
        const auto radius = float(geometry.m_capsuleRadius);
        const auto height = float(geometry.m_capsuleHeight);
        return Shape{ShapeType::Capsule, pose, {radius, height}, material};
    }
    else if (URDF_GEOM_PLANE == geometry.m_type) {
        const auto n = geometry.m_planeNormal;
        const auto z = btVector3{0.0, 0.0, 1.0};
        if (n.dot(z) < 0.99) {
            const auto axis = n.cross(z);
            const auto quat = btQuaternion(axis, btAsin(axis.length()));
            frame = frame * btTransform(quat);
        }
        const auto pose = makePose(frame);
        return Shape{ShapeType::Plane, pose, {0.f}, material};
    }
    else if (URDF_GEOM_MESH == geometry.m_type) {
        const auto pose = makePose(frame, geometry.m_meshScale);
        const auto mesh = geometry.m_meshFileType == UrdfGeometry::MEMORY_VERTICES
                              ? std::make_shared<Mesh>(getMeshData(geometry))
                              : std::make_shared<Mesh>(geometry.m_meshFileName);
        if (flags & URDF_USE_MATERIAL_COLORS_FROM_MTL)
            material.reset();
        return Shape{ShapeType::Mesh, pose, mesh, material};
    }
    else if (URDF_GEOM_HEIGHTFIELD == geometry.m_type) {
        const auto pose = makePose(frame);
        const auto mesh = std::make_shared<Mesh>(getMeshData(geometry));
        return Shape{ShapeType::Heightfield, pose, mesh, material};
    }

    return Shape{};
}

/**
 * @brief Convert URDF shape to a pybullet's visual shape representation
 *
 * @param urdfShape - pybullet URDF shape description
 * @param urdfMaterial - pybullet URDF material description
 * @param localInertiaFrame - link inertia frame
 * @param bodyUniqueId - body unique id
 * @param linkIndex - link index
 * @return VisualShapeData
 */
inline b3VisualShapeData makeVisualShapeData(const UrdfShape& urdfShape,
                                             const UrdfMaterial& urdfMaterial,
                                             const btTransform& localInertiaFrame, //
                                             int bodyUniqueId, int linkIndex)
{
    b3VisualShapeData shape;

    shape.m_objectUniqueId = bodyUniqueId;
    shape.m_linkIndex = linkIndex;

    const auto& origin = localInertiaFrame.getOrigin();
    shape.m_localVisualFrame[0] = origin.getX();
    shape.m_localVisualFrame[1] = origin.getY();
    shape.m_localVisualFrame[2] = origin.getZ();

    const auto& rotate = localInertiaFrame.getRotation();
    shape.m_localVisualFrame[3] = rotate.getX();
    shape.m_localVisualFrame[4] = rotate.getY();
    shape.m_localVisualFrame[5] = rotate.getZ();
    shape.m_localVisualFrame[6] = rotate.getW();

    shape.m_visualGeometryType = urdfShape.m_geometry.m_type;
    shape.m_dimensions[0] = urdfShape.m_geometry.m_meshScale[0];
    shape.m_dimensions[1] = urdfShape.m_geometry.m_meshScale[0];
    shape.m_dimensions[2] = urdfShape.m_geometry.m_meshScale[0];

    shape.m_rgbaColor[0] = urdfMaterial.m_matColor.m_rgbaColor[0];
    shape.m_rgbaColor[1] = urdfMaterial.m_matColor.m_rgbaColor[1];
    shape.m_rgbaColor[2] = urdfMaterial.m_matColor.m_rgbaColor[2];
    shape.m_rgbaColor[3] = urdfMaterial.m_matColor.m_rgbaColor[3];

    std::strncpy(shape.m_meshAssetFileName, urdfMaterial.m_textureFilename.c_str(),
                 VISUAL_SHAPE_MAX_PATH_LEN);

    shape.m_textureUniqueId = -1;
    shape.m_openglTextureId = -1;
    shape.m_tinyRendererTextureId = -1;

    return shape;
}

inline std::shared_ptr<scene::Light> makeDefaultLight()
{
    auto light = std::make_shared<scene::Light>();
    light->setType(scene::LightType::DirectionalLight);
    light->setTarget({0.0, 0.0, 0.0});
    light->setDirection({0.4, -0.25, -0.86});
    light->setDistance(2.0);
    light->setAmbientCoeff(0.6);
    light->setAmbientCoeff(0.35);
    light->setAmbientCoeff(0.05);
    light->shadowCaster(false);
    return light;
}