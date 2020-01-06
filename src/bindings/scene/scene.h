#pragma once

#include "SceneGraph.h"
#include "SceneState.h"
#include "SceneView.h"

void bindScene(py::module& m)
{
    bindSceneGraph(m);
    bindSceneState(m);
    bindSceneView(m);
}
