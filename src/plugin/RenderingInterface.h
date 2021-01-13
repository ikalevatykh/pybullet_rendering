// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <render/BaseRenderer.h>
#include <scene/SceneGraph.h>
#include <scene/SceneState.h>
#include <scene/SceneView.h>

#include <map>
#include <memory>
#include <vector>

#include <Importers/ImportURDFDemo/UrdfRenderingInterface.h>

class RenderingInterface : public UrdfRenderingInterface
{
  public:
    /// constructor
    RenderingInterface();

    /// destructor
    virtual ~RenderingInterface();

    /// set renderer
    void setRenderer(const std::shared_ptr<render::BaseRenderer>& renderer);

    /// given a URDF link, convert all visual shapes into internal renderer (loading graphics
    /// meshes, textures etc)
    /// use the collisionObjectUid as a unique identifier to synchronize the world transform and to
    /// remove the visual shape.
    int convertVisualShapes(int linkIndex, const char* pathPrefix,
                            const btTransform& localInertiaFrame, const UrdfLink* linkPtr,
                            const UrdfModel* model, int collisionObjectUniqueId, int bodyUniqueId,
                            struct CommonFileIOInterface* fileIO) override;

    int registerShapeAndInstance(const struct b3VisualShapeData& visualShape, const float* vertices,
                                 int numvertices, const int* indices, int numIndices,
                                 int primitiveType, int textureId, int orgGraphicsUniqueId,
                                 int bodyUniqueId, int linkIndex) override;

    void updateShape(int shapeUniqueId, const btVector3* vertices, int numVertices,
                     const btVector3* normals, int numNormals) override;

    /// remove a visual shapes, based on the shape unique id (shapeUid)
    void removeVisualShape(int collisionObjectUid) override;

    /// update the world transform + scaling of the visual shape, using the shapeUid
    void syncTransform(int collisionObjectUid, const class btTransform& worldTransform,
                       const class btVector3& localScaling) override;

    /// return the number of visual shapes, for a particular body unique id
    int getNumVisualShapes(int bodyUniqueId) override;

    /// return the visual shape information, for a particular body unique id and 'shape index'
    int getVisualShapesData(int bodyUniqueId, int shapeIndex,
                            struct b3VisualShapeData* shapeData) override;

    /// change the RGBA color for some visual shape.
    void changeRGBAColor(int bodyUniqueId, int linkIndex, int shapeIndex,
                         const double rgbaColor[4]) override;

    /// change the instance flags, double-sided rendering
    void changeInstanceFlags(int bodyUniqueId, int linkIndex, int shapeIndex, int flags) override;

    /// select a given texture for some visual shape.
    void changeShapeTexture(int objectUniqueId, int linkIndex, int shapeIndex,
                            int textureUniqueId) override;

    /// pick the up-axis, either Y (1) or Z (2)
    void setUpAxis(int axis) override;

    /// compute the view matrix based on those parameters
    void resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY,
                     float camPosZ) override;

    /// clear the render buffer with a particular color.
    void clearBuffers(struct TGAColor& clearColor) override;

    /// remove all visual shapes.
    void resetAll() override;

    /// return the frame buffer width and height for the renderer
    void getWidthAndHeight(int& width, int& height) override;

    /// set the frame buffer width and height for the renderer
    void setWidthAndHeight(int width, int height) override;

    /// set the light direction, in world coordinates
    void setLightDirection(float x, float y, float z) override;

    /// set the ambient light color, in world coordinates
    void setLightColor(float x, float y, float z) override;

    /// set the light distance
    void setLightDistance(float dist) override;

    /// set the light ambient coefficient
    void setLightAmbientCoeff(float ambientCoeff) override;

    /// set the light diffuse coefficient
    void setLightDiffuseCoeff(float diffuseCoeff) override;

    /// set the light specular coefficient
    void setLightSpecularCoeff(float specularCoeff) override;

    /// enable or disable rendering of shadows
    void setShadow(bool hasShadow) override;

    /// some undocumented flags for experimentation (todo: document)
    void setFlags(int flags) override;

    /// provide the image pixels as a part of a stream.
    void copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
                             float* depthBuffer, int depthBufferSizeInPixels,
                             int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,
                             int startPixelIndex, int* widthPtr, int* heightPtr,
                             int* numPixelsCopied) override;

    /// render an image, using some arbitraty view and projection matrix
    void render() override;

    /// render an image using the provided view and projection matrix
    void render(const float viewMat[16], const float projMat[16]) override;

    /// load a texture from file, in png or other popular/supported format
    // int loadTextureFile(const char* filename);
    int loadTextureFile(const char* filename, struct CommonFileIOInterface* fileIO) override;

    /// register a texture using an in-memory pixel buffer of a given width and height
    int registerTexture(unsigned char* texels, int width, int height) override;

    /// control projective texture mode
    void setProjectiveTextureMatrices(const float viewMatrix[16],
                                      const float projectionMatrix[16]) override;
    void setProjectiveTexture(bool useProjectiveTexture) override;

    /// retrieve camera info
    // virtual bool getCameraInfo(int* width, int* height, float viewMatrix[16], float
    // projectionMatrix[16], float camUp[3], float camForward[3],
    // 	float hor[3], float vert[3], float* yaw, float* pitch, float* camDist, float
    // cameraTarget[3]) const;

  private:
    std::shared_ptr<render::BaseRenderer> _renderer;

    int _flags;
    bool _syncSceneGraph;
    bool _syncMaterials;
    std::shared_ptr<scene::SceneGraph> _sceneGraph;
    std::shared_ptr<scene::SceneState> _sceneState;
    std::shared_ptr<scene::SceneView> _sceneView;
    std::shared_ptr<scene::Light> _light;
    std::shared_ptr<scene::Camera> _camera;
    std::vector<std::shared_ptr<scene::Texture>> _textures;

    // bullet-specific data
    std::map<int, std::vector<struct b3VisualShapeData>> _visualShapes;
    std::map<std::pair<int, int>, int> _objectIndices;
};
