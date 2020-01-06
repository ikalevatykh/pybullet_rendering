#include "RenderingInterface.h"
#include "utils.h"
#include <scene/Shape.h>

#include <CommonInterfaces/CommonFileIOInterface.h>
#include <Importers/ImportURDFDemo/UrdfParser.h>
#include <SharedMemory/SharedMemoryPublic.h>
#include <TinyRenderer/tgaimage.h>

RenderingInterface::RenderingInterface()
{
    resetAll();
}

RenderingInterface::~RenderingInterface()
{
}

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
    _sceneGraph.clear();
    _sceneState.clear();
    _sceneView.reset();
    _visualShapes.clear();
    _objectIndices.clear();
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
        const auto& shape =
            makeShape(urdfShape, urdfMaterial, localInertiaFrame, _flags, _sceneGraph);
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
        const auto& shape =
            makeShape(urdfShape, urdfMaterial, localInertiaFrame, _flags, _sceneGraph);
        if (shape.valid())
            sceneShapes.push_back(shape);
    }

    // if there is something to render adding an object to a scene
    if (!sceneShapes.empty()) {
        const auto nodeId = collisionObjectUid;
        const bool enableCache = _flags & URDF_ENABLE_CACHED_GRAPHICS_SHAPES;
        _sceneGraph.appendNode(nodeId, {bodyUniqueId, linkIndex, sceneShapes, enableCache});
        _sceneState.appendNode(nodeId);
        _syncSceneGraph = true;

        _objectIndices.emplace(std::make_pair(bodyUniqueId, linkIndex), collisionObjectUid);
        return collisionObjectUid;
    }

    return -1;
}

int RenderingInterface::addVisualShape(struct b3VisualShapeData* visualShape,
                                       struct CommonFileIOInterface* fileIO)
{
    /// @todo: addVisualShape not implemented
    return -1;
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
        int i = 0;
        for (auto& visualShape : ptr->second) {
            if (visualShape.m_linkIndex != linkIndex)
                continue;

            if (shapeIndex != -1 && shapeIndex != i++)
                continue;

            // update pybullet-specific descriptor
            for (int i = 0; i < 4; ++i)
                visualShape.m_rgbaColor[i] = rgba[i];

            // update scene graph
            _sceneGraph.changeShapeColor(
                collisionObjectUid, shapeIndex,
                {float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3])});
            _syncMaterials = true;
        }
    }
}

void RenderingInterface::changeShapeTexture(int bodyUniqueId, int linkIndex, int shapeIndex,
                                            int textureUniqueId)
{
    const int collisionObjectUid = _objectIndices.at({bodyUniqueId, linkIndex});

    auto ptr = _visualShapes.find(bodyUniqueId);
    if (ptr != _visualShapes.end()) {
        int i = 0;
        for (auto& visualShape : ptr->second) {
            if (visualShape.m_linkIndex != linkIndex)
                continue;

            if (shapeIndex != -1 && shapeIndex != i++)
                continue;

            // update pybullet-specific descriptor
            visualShape.m_textureUniqueId = textureUniqueId;

            // update scene graph
            _sceneGraph.changeShapeTexture(collisionObjectUid, shapeIndex, textureUniqueId);
            _syncMaterials = true;
        }
    }
}

void RenderingInterface::removeVisualShape(int collisionObjectUid)
{
    _sceneGraph.removeNode(collisionObjectUid);
    _sceneState.removeNode(collisionObjectUid);
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

    _sceneView.backgroundColor({r, g, b});
}

void RenderingInterface::getWidthAndHeight(int& width, int& height)
{
    const auto& vp = _sceneView.viewport();

    width = vp[0];
    height = vp[1];
}

void RenderingInterface::setWidthAndHeight(int width, int height)
{
    // just a good place to reset it
    _light.direction({-0.4, 0.25, 0.86});
    _light.color({1.0, 1.0, 1.0});
    _light.distance(2.0);
    _light.ambientCoeff(0.6);
    _light.diffuseCoeff(0.35);
    _light.specularCoeff(0.05);

    _sceneView.backgroundColor({0.7, 0.7, 0.8});

    //_sceneView.reset();
    _sceneView.viewport({width, height});
}

void RenderingInterface::setLightDirection(float x, float y, float z)
{
    _light.direction({x, y, z});
}

void RenderingInterface::setLightColor(float r, float g, float b)
{
    _light.color({r, g, b});
}

void RenderingInterface::setLightDistance(float dist)
{
    _light.distance(dist);
}

void RenderingInterface::setLightAmbientCoeff(float ambientCoeff)
{
    _light.ambientCoeff(ambientCoeff);
}

void RenderingInterface::setLightDiffuseCoeff(float diffuseCoeff)
{
    _light.diffuseCoeff(diffuseCoeff);
}

void RenderingInterface::setLightSpecularCoeff(float specularCoeff)
{
    _light.specularCoeff(specularCoeff);
}

void RenderingInterface::setShadow(bool hasShadow)
{
    _sceneView.enableShadow(hasShadow);
}

void RenderingInterface::setFlags(int flags)
{
    _flags = flags;
}

int RenderingInterface::loadTextureFile(const char* filename, struct CommonFileIOInterface* fileIO)
{
    return _sceneGraph.registerTexture(scene::Texture{filename});
}

int RenderingInterface::registerTexture(unsigned char* texels, int width, int height)
{
    const auto data = std::vector<unsigned char>{texels, texels + width * height * 4};
    return _sceneGraph.registerTexture(scene::Texture{data, {width, height}});
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
    if (collisionObjectUId >= 0) {
        scene::Affine3f pose;
        setPose(pose, worldTransform, localScaling);
        _sceneState.pose(collisionObjectUId, pose);
    }
}

void RenderingInterface::render(const float viewMat[16], const float projMat[16])
{
    _camera.viewMatrix(reinterpret_cast<const scene::Matrix4f&>(viewMat));
    _camera.projMatrix(reinterpret_cast<const scene::Matrix4f&>(projMat));

    // auto& cam = _sceneView.camera();

    // cam.viewMatrix(scene::Matrix4f{viewMat});
    // cam.projMatrix(scene::Matrix4f{projMat});
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

        // can disable some plane for performance reasons
        ///@todo: add flags
        bool enableColor = true;
        bool enableDepth = true;
        bool enableMask = maskBuffer != nullptr && maskSizeInPixels > 0;
        _sceneView.enablePlanes(enableColor, enableDepth, enableMask);

        // set standart light and camera
        _sceneView.light(_light);
        _sceneView.camera(_camera);

        // draw
        if (_renderer->draw(_sceneState, _sceneView)) {
            // _renderer->readColor(pixelsRGBA + startPixelIndex, rgbaBufferSizeInPixels);
            // _renderer->readDepth(depthBuffer + startPixelIndex, depthBufferSizeInPixels);
            // _renderer->readMask((uint32_t*)maskBuffer + startPixelIndex, maskSizeInPixels);

            *numPixelsCopied = *widthPtr * *heightPtr;
            return;
        }
    }

    // do nothing
    *widthPtr = 0;
    *heightPtr = 0;
    *numPixelsCopied = 0;
}
