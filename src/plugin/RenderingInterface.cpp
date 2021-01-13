// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderingInterface.h"
#include "utils.h"
#include <scene/Shape.h>

#include <CommonInterfaces/CommonFileIOInterface.h>
#include <Importers/ImportURDFDemo/UrdfParser.h>
#include <SharedMemory/SharedMemoryPublic.h>
#include <TinyRenderer/tgaimage.h>

RenderingInterface::RenderingInterface()
    : _sceneGraph{std::make_shared<scene::SceneGraph>()},
      _sceneState{std::make_shared<scene::SceneState>()}, //
      _sceneView{std::make_shared<scene::SceneView>()}
{
    resetAll();
}

RenderingInterface::~RenderingInterface() {}

void RenderingInterface::setRenderer(const std::shared_ptr<render::BaseRenderer>& renderer)
{
    _renderer = renderer;
    _syncSceneGraph = true;
    _syncMaterials = true;
}

void RenderingInterface::resetAll()
{
    _flags = 0;
    _syncSceneGraph = true;
    _syncMaterials = true;
    _sceneGraph->clear();
    _sceneState->clear();
    _visualShapes.clear();
    _objectIndices.clear();
    _textures.clear();
}

int RenderingInterface::convertVisualShapes(int linkIndex, const char* pathPrefix,
                                            const btTransform& localInertiaFrame,
                                            const UrdfLink* linkPtr, const UrdfModel* urdfModel,
                                            int collisionObjectUid, int bodyUniqueId,
                                            struct CommonFileIOInterface* fileIO)
{
    const int numVisual = linkPtr->m_visualArray.size();
    const int numCollision = numVisual ? 0 : linkPtr->m_collisionArray.size();

    std::vector<scene::Shape> sceneShapes;
    sceneShapes.reserve(numVisual + numCollision);

    // Process visual shapes
    for (int i = 0; i < numVisual; ++i) {
        const auto& urdfShape = linkPtr->m_visualArray[i];

        const auto key = btHashString(urdfShape.m_materialName.c_str());
        const auto& urdfMaterial = urdfModel->m_materials[key]
                                       ? **urdfModel->m_materials[key]
                                       : urdfShape.m_geometry.m_localMaterial;

        // append a bullet-specific shape description
        _visualShapes[bodyUniqueId].emplace_back(makeVisualShapeData(
            urdfShape, urdfMaterial, localInertiaFrame, bodyUniqueId, linkIndex));

        // append a new shape to render
        const auto& shape = makeShape(urdfShape, urdfMaterial, localInertiaFrame, _flags);
        if (shape.valid())
            sceneShapes.push_back(shape);
    }

    // Process collision shapes only if an object has no one visual shape
    for (int i = 0; i < numCollision; ++i) {
        const auto& urdfShape = linkPtr->m_collisionArray[i];

        static btVector4 diffuseColor[] = {
            {0.2, 0.7, 0.3, 1.0}, {0.9, 0.7, 0.1, 1.0}, {0.8, 0.2, 0.2, 1.0}, {0.3, 0.5, 0.9, 1.0}};

        UrdfMaterial urdfMaterial;
        urdfMaterial.m_matColor.m_rgbaColor = diffuseColor[i % 4];
        urdfMaterial.m_matColor.m_specularColor = {1.0, 1.0, 1.0};

        // append a new shape to render
        const auto& shape = makeShape(urdfShape, urdfMaterial, localInertiaFrame, _flags);
        if (shape.valid())
            sceneShapes.push_back(shape);
    }

    // if there is something to render adding an object to a scene
    if (!sceneShapes.empty()) {
        const auto nodeId = collisionObjectUid;
        const bool noCache = !(_flags & URDF_ENABLE_CACHED_GRAPHICS_SHAPES);
        _sceneGraph->appendNode(nodeId, {bodyUniqueId, linkIndex, std::move(sceneShapes), noCache});
        _sceneState->appendNode(nodeId);
        _syncSceneGraph = true;

        _objectIndices.emplace(std::make_pair(bodyUniqueId, linkIndex), collisionObjectUid);
        return collisionObjectUid;
    }

    return -1;
}

int RenderingInterface::registerShapeAndInstance(const struct b3VisualShapeData& visualShape,
                                                 const float* vertices, int numvertices,
                                                 const int* indices, int numIndices,
                                                 int primitiveType, int textureId,
                                                 int orgGraphicsUniqueId, int bodyUniqueId,
                                                 int linkIndex)
{
    /// @todo: registerShapeAndInstance not implemented
    return -1;
}

void RenderingInterface::updateShape(int shapeUniqueId, const btVector3* vertices, int numVertices,
                                     const btVector3* normals, int numNormals)
{
    /// @todo: updateShape not implemented
}

int RenderingInterface::getNumVisualShapes(int bodyUniqueId)
{
    auto ptr = _visualShapes.find(bodyUniqueId);
    if (ptr != _visualShapes.end()) {
        return static_cast<int>(ptr->second.size());
    }
    return 0;
}

int RenderingInterface::getVisualShapesData(int bodyUniqueId, int shapeIndex,
                                            struct b3VisualShapeData* shapeData)
{
    auto ptr = _visualShapes.find(bodyUniqueId);
    if (ptr != _visualShapes.end() && ptr->second.size() > shapeIndex) {
        *shapeData = ptr->second.at(shapeIndex);
        return 1;
    }
    return 0;
}

void RenderingInterface::changeRGBAColor(int bodyUniqueId, int linkIndex, int shapeIndex,
                                         const double rgba[4])
{
    const int collisionObjectUid = _objectIndices.at({bodyUniqueId, linkIndex});

    auto ptr = _visualShapes.find(bodyUniqueId);
    if (ptr != _visualShapes.end()) {
        int i = -1;
        for (auto& visualShape : ptr->second) {
            if (visualShape.m_linkIndex != linkIndex)
                continue;

            if (shapeIndex != ++i && shapeIndex != -1)
                continue;

            // update pybullet-specific descriptor
            for (int j = 0; j < 4; ++j)
                visualShape.m_rgbaColor[j] = rgba[j];

            // update scene graph
            _sceneGraph->changeShapeColor(
                collisionObjectUid, i,
                {float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3])});
            _syncMaterials = true;
        }
    }
}

void RenderingInterface::changeInstanceFlags(int bodyUniqueId, int linkIndex, int shapeIndex,
                                             int flags)
{
    /// @todo: changeInstanceFlags not implemented
}

void RenderingInterface::changeShapeTexture(int bodyUniqueId, int linkIndex, int shapeIndex,
                                            int textureUniqueId)
{
    const int collisionObjectUid = _objectIndices.at({bodyUniqueId, linkIndex});

    auto ptr = _visualShapes.find(bodyUniqueId);
    if (ptr != _visualShapes.end()) {
        int i = -1;
        for (auto& visualShape : ptr->second) {
            if (visualShape.m_linkIndex != linkIndex)
                continue;

            if (shapeIndex != ++i && shapeIndex != -1)
                continue;

            // update pybullet-specific descriptor
            visualShape.m_textureUniqueId = textureUniqueId;

            // update scene graph
            _sceneGraph->changeShapeTexture(collisionObjectUid, i,
                                            textureUniqueId < 0 ? std::shared_ptr<scene::Texture>()
                                                                : _textures.at(textureUniqueId));
            _syncMaterials = true;
        }
    }
}

void RenderingInterface::removeVisualShape(int collisionObjectUid)
{
    _sceneGraph->removeNode(collisionObjectUid);
    _sceneState->removeNode(collisionObjectUid);
    _syncSceneGraph = true;
}

void RenderingInterface::setUpAxis(int axis)
{
    /// @todo: setUpAxis not implemented
}

void RenderingInterface::resetCamera(float camDist, float yaw, float pitch, float camPosX,
                                     float camPosY, float camPosZ)
{
    /// @todo: resetCamera not implemented
}

void RenderingInterface::clearBuffers(struct TGAColor& clearColor)
{
    auto b = clearColor.bgra[0] / 255.f;
    auto g = clearColor.bgra[1] / 255.f;
    auto r = clearColor.bgra[2] / 255.f;

    _sceneView->setBackgroundColor({r, g, b});
}

void RenderingInterface::getWidthAndHeight(int& width, int& height)
{
    const auto& vp = _sceneView->viewport();

    width = vp[0];
    height = vp[1];
}

void RenderingInterface::setWidthAndHeight(int width, int height)
{
    _sceneView->setBackgroundColor({0.7, 0.7, 0.8});
    _sceneView->setViewport({width, height});
}

void RenderingInterface::setLightDirection(float x, float y, float z)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->setDirection({x, y, z});
}

void RenderingInterface::setLightColor(float r, float g, float b)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->setColor({r, g, b});
}

void RenderingInterface::setLightDistance(float dist)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->setDistance(dist);
}

void RenderingInterface::setLightAmbientCoeff(float ambientCoeff)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->setAmbientCoeff(ambientCoeff);
}

void RenderingInterface::setLightDiffuseCoeff(float diffuseCoeff)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->setDiffuseCoeff(diffuseCoeff);
}

void RenderingInterface::setLightSpecularCoeff(float specularCoeff)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->setSpecularCoeff(specularCoeff);
}

void RenderingInterface::setShadow(bool hasShadow)
{
    _light = !_light ? makeDefaultLight() : _light;
    _light->shadowCaster(hasShadow);
}

void RenderingInterface::setFlags(int flags)
{
    _flags = flags;
}

int RenderingInterface::loadTextureFile(const char* filename, struct CommonFileIOInterface* fileIO)
{
    _textures.push_back(std::make_shared<scene::Texture>(filename));
    return int(_textures.size()) - 1;
}

int RenderingInterface::registerTexture(unsigned char* texels, int width, int height)
{
    auto data = std::vector<unsigned char>{texels, texels + width * height * 4};
    _textures.push_back(std::make_shared<scene::Texture>(std::move(data), Size2i{width, height}));
    return int(_textures.size()) - 1;
}

void RenderingInterface::setProjectiveTextureMatrices(const float viewMatrix[16],
                                                      const float projectionMatrix[16])
{
    /// @todo: setProjectiveTextureMatrices not implemented
}

void RenderingInterface::setProjectiveTexture(bool setProjectiveTexture)
{
    /// @todo: setProjectiveTexture not implemented
}

void RenderingInterface::syncTransform(int collisionObjectUId,
                                       const class btTransform& worldTransform,
                                       const class btVector3& localScaling)
{
    if (collisionObjectUId >= 0)
        _sceneState->setPose(collisionObjectUId, makePose(worldTransform, localScaling));
}

void RenderingInterface::render(const float viewMat[16], const float projMat[16])
{
    _camera = std::make_shared<scene::Camera>(*reinterpret_cast<const Matrix4f*>(viewMat),
                                              *reinterpret_cast<const Matrix4f*>(projMat));

    //               _camera.setViewMatrix(*reinterpret_cast<const Matrix4f*>(viewMat));
    // _camera.setProjMatrix(*reinterpret_cast<const Matrix4f*>(projMat));
}

void RenderingInterface::render()
{
    /// @todo: render not implemented
}

void RenderingInterface::copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
                                             float* depthBuffer, int depthBufferSizeInPixels,
                                             int* maskBuffer, int maskSizeInPixels,
                                             int startPixelIndex, int* widthPtr, int* heightPtr,
                                             int* numPixelsCopied)
{
    if (!!_renderer) {
        // update scene if something changed
        if (_syncSceneGraph || _syncMaterials) {
            _renderer->updateScene(_sceneGraph, !_syncSceneGraph);
            _syncSceneGraph = false;
            _syncMaterials = false;
        }

        // set light and camera
        _sceneView->setLight(_light);
        _sceneView->setCamera(_camera);
        _sceneView->setFlags(_flags);

        _light.reset();
        _camera.reset();

        // destination buffer
        ///@todo: partial buffers, check buffers size
        render::FrameData frame{*widthPtr, *heightPtr, pixelsRGBA, depthBuffer, maskBuffer};

        // render
        if (_renderer->renderFrame(_sceneState, _sceneView, frame)) {
            *numPixelsCopied = frame.rows * frame.cols;
            return;
        }
    }

    *widthPtr = 0;
    *heightPtr = 0;
    *numPixelsCopied = 0;
}
