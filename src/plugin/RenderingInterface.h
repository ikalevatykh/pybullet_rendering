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
    virtual int convertVisualShapes(int linkIndex, const char* pathPrefix,
                                    const btTransform& localInertiaFrame, const UrdfLink* linkPtr,
                                    const UrdfModel* model, int collisionObjectUniqueId,
                                    int bodyUniqueId, struct CommonFileIOInterface* fileIO);

    virtual int registerShapeAndInstance(const struct b3VisualShapeData& visualShape,
                                         const float* vertices, int numvertices, const int* indices,
                                         int numIndices, int primitiveType, int textureId,
                                         int orgGraphicsUniqueId, int bodyUniqueId, int linkIndex);

    virtual void updateShape(int shapeUniqueId, const btVector3* vertices, int numVertices,
                             const btVector3* normals, int numNormals);

    /// Given b3VisualShapeData, add render information (texture, rgbaColor etc) to the visualShape
    /// and visualShape to internal renderer.
    /// Returns a visualShapeUniqueId as a unique identifier to synchronize the world transform and
    /// to remove the visual shape.
    virtual int addVisualShape(struct b3VisualShapeData* visualShape,
                               struct CommonFileIOInterface* fileIO);

    /// remove a visual shapes, based on the shape unique id (shapeUid)
    virtual void removeVisualShape(int collisionObjectUid);

    /// update the world transform + scaling of the visual shape, using the shapeUid
    virtual void syncTransform(int collisionObjectUid, const class btTransform& worldTransform,
                               const class btVector3& localScaling);

    /// return the number of visual shapes, for a particular body unique id
    virtual int getNumVisualShapes(int bodyUniqueId);

    /// return the visual shape information, for a particular body unique id and 'shape index'
    virtual int getVisualShapesData(int bodyUniqueId, int shapeIndex,
                                    struct b3VisualShapeData* shapeData);

    /// change the RGBA color for some visual shape.
    virtual void changeRGBAColor(int bodyUniqueId, int linkIndex, int shapeIndex,
                                 const double rgbaColor[4]);

    /// change the instance flags, double-sided rendering
    virtual void changeInstanceFlags(int bodyUniqueId, int linkIndex, int shapeIndex, int flags);

    /// select a given texture for some visual shape.
    virtual void changeShapeTexture(int objectUniqueId, int linkIndex, int shapeIndex,
                                    int textureUniqueId);

    /// pick the up-axis, either Y (1) or Z (2)
    virtual void setUpAxis(int axis);

    /// compute the view matrix based on those parameters
    virtual void resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY,
                             float camPosZ);

    /// clear the render buffer with a particular color.
    virtual void clearBuffers(struct TGAColor& clearColor);

    /// remove all visual shapes.
    virtual void resetAll();

    /// return the frame buffer width and height for the renderer
    virtual void getWidthAndHeight(int& width, int& height);

    /// set the frame buffer width and height for the renderer
    virtual void setWidthAndHeight(int width, int height);

    /// set the light direction, in world coordinates
    virtual void setLightDirection(float x, float y, float z);

    /// set the ambient light color, in world coordinates
    virtual void setLightColor(float x, float y, float z);

    /// set the light distance
    virtual void setLightDistance(float dist);

    /// set the light ambient coefficient
    virtual void setLightAmbientCoeff(float ambientCoeff);

    /// set the light diffuse coefficient
    virtual void setLightDiffuseCoeff(float diffuseCoeff);

    /// set the light specular coefficient
    virtual void setLightSpecularCoeff(float specularCoeff);

    /// enable or disable rendering of shadows
    virtual void setShadow(bool hasShadow);

    /// some undocumented flags for experimentation (todo: document)
    virtual void setFlags(int flags);

    /// provide the image pixels as a part of a stream.
    virtual void copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
                                     float* depthBuffer, int depthBufferSizeInPixels,
                                     int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,
                                     int startPixelIndex, int* widthPtr, int* heightPtr,
                                     int* numPixelsCopied);

    /// render an image, using some arbitraty view and projection matrix
    virtual void render();

    /// render an image using the provided view and projection matrix
    virtual void render(const float viewMat[16], const float projMat[16]);

    /// load a texture from file, in png or other popular/supported format
    // virtual int loadTextureFile(const char* filename);
    virtual int loadTextureFile(const char* filename, struct CommonFileIOInterface* fileIO);

    /// register a texture using an in-memory pixel buffer of a given width and height
    virtual int registerTexture(unsigned char* texels, int width, int height);

    /// control projective texture mode
    virtual void setProjectiveTextureMatrices(const float viewMatrix[16],
                                              const float projectionMatrix[16]);
    virtual void setProjectiveTexture(bool useProjectiveTexture);

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
    scene::SceneGraph _sceneGraph;
    scene::SceneState _sceneState;
    scene::SceneView _sceneView;
    scene::Light _light;
    scene::Camera _camera;

    // bullet-specific data
    std::map<int, std::vector<struct b3VisualShapeData>> _visualShapes;
    std::map<std::pair<int, int>, int> _objectIndices;
};
