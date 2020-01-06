#pragma once

// project imports
#include <scene/Math.h>
#include <scene/SceneGraph.h>

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
inline void setPose(scene::Affine3f& pose, const btTransform& frame, const btVector3& scale)
{
    const auto& origin = frame.getOrigin();
    const auto& basis = frame.getBasis();

    btQuaternion quat;
    basis.getRotation(quat);

    pose.origin = {float(origin.x()), float(origin.y()), float(origin.z())};
    pose.quaternion = {float(quat.getW()), float(quat.x()), float(quat.y()), float(quat.z())};
    pose.scale = {float(scale.x()), float(scale.y()), float(scale.z())};
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
                              const btTransform& localInertiaFrame, int flags,
                              scene::SceneGraph& graph)
{
    auto frame = localInertiaFrame.inverse() * urdfShape.m_linkLocalFrame;
    scene::Affine3f pose;

    const auto& fname = urdfMaterial.m_textureFilename;
    const int textureId = fname.empty() ? -1 : graph.registerTexture(scene::Texture{fname});

    const auto& d = urdfMaterial.m_matColor.m_rgbaColor;
    const auto& s = urdfMaterial.m_matColor.m_specularColor;

    auto material = scene::Material{{float(d[0]), float(d[1]), float(d[2]), float(d[3])},
                                    {float(s[0]), float(s[1]), float(s[2])},
                                    textureId};

    const auto& urdfGeometry = urdfShape.m_geometry;
    if (URDF_GEOM_MESH == urdfGeometry.m_type) {
        if (urdfGeometry.m_meshFileType == UrdfGeometry::MEMORY_VERTICES) {
            /** @todo populate mesh from data */
        }
        else {
            const bool useMaterial = flags & URDF_USE_MATERIAL_COLORS_FROM_MTL;
            const auto mesh = scene::Mesh{urdfGeometry.m_meshFileName, useMaterial};
            setPose(pose, frame, urdfGeometry.m_meshScale);
            return scene::Shape{mesh, pose, material};
        }
    }
    else if (URDF_GEOM_BOX == urdfGeometry.m_type) {
        setPose(pose, frame, urdfGeometry.m_boxSize);
        return scene::Shape{scene::ShapeType::Cube, pose, material};
    }
    else if (URDF_GEOM_SPHERE == urdfGeometry.m_type) {
        const auto r = urdfGeometry.m_sphereRadius;
        setPose(pose, frame, {r, r, r});
        return scene::Shape{scene::ShapeType::Sphere, pose, material};
    }
    else if (URDF_GEOM_CYLINDER == urdfGeometry.m_type) {
        const auto r = urdfGeometry.m_capsuleRadius;
        const auto h = urdfGeometry.m_capsuleHeight;
        setPose(pose, frame, {r, r, h});
        return scene::Shape{scene::ShapeType::Cylinder, pose, material};
    }
    else if (URDF_GEOM_CAPSULE == urdfGeometry.m_type) {
        const auto r = urdfGeometry.m_capsuleRadius;
        const auto h = urdfGeometry.m_capsuleHeight;
        setPose(pose, frame, {r, r, h});
        return scene::Shape{scene::ShapeType::Capsule, pose, material};
    }
    else if (URDF_GEOM_PLANE == urdfGeometry.m_type) {
        const auto n = urdfGeometry.m_planeNormal;
        const auto z = btVector3{0, 0, 1};
        if (n.dot(z) < 0.99) {
            const auto axis = n.cross(z);
            const auto quat = btQuaternion(axis, btAsin(axis.length()));
            frame = frame * btTransform(quat);
        }
        setPose(pose, frame, {1, 1, 1});
        return scene::Shape{scene::ShapeType::Plane, pose, material};
    }

    return scene::Shape{scene::ShapeType::Unknown, pose, material};
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
