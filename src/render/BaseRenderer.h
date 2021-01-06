// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <scene/SceneGraph.h>
#include <scene/SceneState.h>
#include <scene/SceneView.h>

namespace render {

/**
 * @brief Memory buffer to write rendered frame
 *
 */
struct FrameData {
    const int cols; //<- image width
    const int rows; //<- image height
    uint8_t* const color; //<- pointer to the color plane memory
    float* const depth; //<- pointer to the depth plane memory
    int* const mask; //<- pointer to the mask plane memory
};

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
    virtual void updateScene(const std::shared_ptr<scene::SceneGraph>& sceneGraph,
                             bool materialsOnly) = 0;

    /**
     * @brief Render a scene at state \p sceneState with a view settings \p sceneView
     *
     * @param sceneState - scene state, e.g. transformations of all objects
     * @param sceneView - view settings, e.g. camera, light, viewport size
     * @param outputFrame - rendered images
     *
     * @return True if rendered
     */
    virtual bool renderFrame(const std::shared_ptr<scene::SceneState>& sceneState,
                             const std::shared_ptr<scene::SceneView>& sceneView,
                             FrameData& outputFrame) = 0;
};

} // namespace render
