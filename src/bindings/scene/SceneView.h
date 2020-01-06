#pragma once

#include <scene/SceneView.h>

void bindSceneView(py::module& m)
{
    using namespace scene;

    // LightType enum
    py::enum_<LightType>(m, "LightType", py::arithmetic())
        .value("Unknown", LightType::Unknown)
        .value("AmbientLight", LightType::AmbientLight)
        .value("DirectionalLight", LightType::DirectionalLight)
        .value("PointLight", LightType::PointLight);

    py::class_<Light>(m, "Light")
        .def_property_readonly("type", &Light::type,
                               R"(Light type)")
        .def_property_readonly("color", py::overload_cast<>(&Light::color, py::const_),
                               R"(Light color)")
        .def_property_readonly("direction", py::overload_cast<>(&Light::direction, py::const_),
                               R"(Light direction vector in world coordinates (DirectionalLight))")
        .def_property_readonly("distance", py::overload_cast<>(&Light::distance, py::const_),
                               R"(Light distance (DirectionalLight))")
        .def_property_readonly("position", py::overload_cast<>(&Light::position, py::const_),
                               R"(Light position in world coordinates (PointLight))")
        .def_property_readonly("ambient_coeff",
                               py::overload_cast<>(&Light::ambientCoeff, py::const_),
                               R"(Light ambient coeffitient)")
        .def_property_readonly("diffuse_coeff",
                               py::overload_cast<>(&Light::diffuseCoeff, py::const_),
                               R"(Light diffuse coeffitient)")
        .def_property_readonly("specular_coeff",
                               py::overload_cast<>(&Light::specularCoeff, py::const_),
                               R"(Light specular coeffitient)");

    py::class_<Camera>(m, "Camera")
        .def_property_readonly("view_matrix", py::overload_cast<>(&Camera::viewMatrix, py::const_),
                               R"(Camera view matrix)")
        .def_property_readonly("projection_matrix",
                               py::overload_cast<>(&Camera::projMatrix, py::const_),
                               R"(Camera projection matrix)");

    py::class_<SceneView>(m, "SceneView")
        .def_property_readonly("viewport", py::overload_cast<>(&SceneView::viewport, py::const_),
                               R"(Image size)")
        .def_property_readonly("background_color",
                               py::overload_cast<>(&SceneView::backgroundColor, py::const_),
                               R"(Background color)")
        .def_property_readonly("light", py::overload_cast<>(&SceneView::light, py::const_),
                               R"(Light)")
        .def_property_readonly("camera", py::overload_cast<>(&SceneView::camera, py::const_),
                               R"(Camera)")
        .def_property_readonly("has_shadow", &SceneView::isShadowEnabled,
                               R"(Enable shadows drawing)")
        .def_property_readonly("is_color_enabled", &SceneView::isColorEnabled,
                               R"(Is color plane rendering enabled)")
        .def_property_readonly("is_depth_enabled", &SceneView::isDepthEnabled,
                               R"(Is depth plane rendering enabled)")
        .def_property_readonly("is_mask_enabled", &SceneView::isMaskEnabled,
                               R"(Is mask plane rendering enabled)");
}
