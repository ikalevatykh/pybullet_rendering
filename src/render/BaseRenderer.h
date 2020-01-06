#pragma once

#include <scene/SceneGraph.h>
#include <scene/SceneState.h>
#include <scene/SceneView.h>

namespace render {

// /**
//  * @brief Rendered frame
//  *
//  * Frame may contain color, depth and mask planes
//  */
// class BaseFrame
// {
//   public:
//     /**
//      * @brief Destroy the Base Frame object
//      *
//      */
//     virtual ~BaseFrame(){};

//     /**
//      * @brief Read color frame to a CPU memory buffer
//      *
//      * @param dest - pointer to a destination buffer
//      * @param size - destination buffer length
//      */
//     virtual bool readColor(uint8_t* dest, int size) = 0;

//     /**
//      * @brief  Read depth frame to a CPU memory buffer
//      *
//      * @param dest - pointer to a destination buffer
//      * @param size - destination buffer length
//      */
//     virtual bool readDepth(float* dest, int size) = 0;

//     /**
//      * @brief  Read mask frame to a CPU memory buffer
//      *
//      * @param dest - pointer to a destination buffer
//      * @param size - destination buffer length
//      */
//     virtual bool readMask(uint32_t* dest, int size) = 0;
// };

/**
 * @brief Interface for all renderers
 *
 */
class BaseRenderer
{
  public:
    /**
     * @brief Destroy the Base Renderer object
     *
     */
    virtual ~BaseRenderer(){};

    /**
     * @brief Update a scene using \p sceneGraph description
     *
     * @param sceneGraph - scene description
     * @param materialsOnly - update only shape materials
     */
    virtual void updateScene(const scene::SceneGraph& sceneGraph, bool materialsOnly) = 0;

    /**
     * @brief Draw a scene at state \p sceneState with a view \p sceneView
     *
     * @param sceneState - scene state, e.g. transformations of all objects
     * @param sceneView - view, e.g. camera, light, viewport parameters
     * @return bool - success
     */
    virtual bool draw(const scene::SceneState& sceneState, const scene::SceneView& sceneView) = 0;
};

} // namespace render
