#pragma once

#include <render/BaseRenderer.h>

#include <scene/SceneGraph.h>
#include <scene/SceneState.h>
#include <scene/SceneView.h>

class PyRenderer : public render::BaseRenderer
{
  public:
    /* Inherit the constructors */
    // using render::BaseRenderer::BaseRenderer;
    PyRenderer() {}

    /**
     * @brief Update a scene using \p sceneGraph description
     *
     * @param sceneGraph - scene description
     * @param materialsOnly - update only shape materials
     */
    virtual void updateScene(const scene::SceneGraph& sceneGraph, bool materialsOnly)
    {
        PYBIND11_OVERLOAD_PURE(void, render::BaseRenderer, updateScene, sceneGraph, materialsOnly);
    }

    /**
     * @brief Draw a scene at state \p sceneState with a view \p sceneView
     *
     * @param sceneState - scene state, e.g. transformations of all objects
     * @param sceneView - view, e.g. camera, light, viewport parameters
     * @return bool - success
     */
    virtual bool draw(const scene::SceneState& sceneState, const scene::SceneView& sceneView)
    {
        PYBIND11_OVERLOAD_PURE(bool, render::BaseRenderer, draw, sceneState, sceneView);
    }
};
