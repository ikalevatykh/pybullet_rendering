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
        .def_property_readonly("type", &Light::type, "Light type")
        .def_property_readonly("ambient_color", &Light::ambientColor, "Light ambient color")
        .def_property_readonly("diffuse_color", &Light::diffuseColor, "Light diffuse color")
        .def_property_readonly("specular_color", &Light::specularColor, "Light specular color")
        .def_property("color", &Light::color, &Light::setColor, "Light color")
        .def_property("direction", &Light::direction, &Light::setDirection,
                      "Light direction vector in world coordinates (DirectionalLight)")
        .def_property("distance", &Light::distance, &Light::setDistance,
                      "Light distance (DirectionalLight)")
        .def_property("position", &Light::position, &Light::setPosition,
                      "Light position in world coordinates (PointLight)")
        .def_property("ambient_coeff", &Light::ambientCoeff, &Light::setAmbientCoeff,
                      "Light ambient coeffitient")
        .def_property("diffuse_coeff", &Light::diffuseCoeff, &Light::setDiffuseCoeff,
                      "Light diffuse coeffitient")
        .def_property("specular_coeff", &Light::specularCoeff, &Light::setSpecularCoeff,
                      "Light specular coeffitient")
        .def_property("shadow_caster", &Light::isShadowCaster, &Light::shadowCaster,
                      "Flag indicating whether this light should cast shadows or not")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    py::class_<Camera>(m, "Camera")
        .def_property("view_mat", &Camera::viewMatrix, &Camera::setViewMatrix,
                      py::return_value_policy::reference_internal, "Camera view matrix")
        .def_property("proj_mat", &Camera::projMatrix, &Camera::setProjMatrix,
                      py::return_value_policy::reference_internal, "Camera projection matrix")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    py::class_<SceneView>(m, "SceneView")
        .def_property("viewport", &SceneView::viewport, &SceneView::setViewport, "Image size")
        .def_property("background_color", &SceneView::backgroundColor,
                      &SceneView::setBackgroundColor, "Background color")
        .def_property("background_texture", &SceneView::backgroundTexture,
                      &SceneView::setBackgroundTexture, "Background texture")
        .def_property("light", &SceneView::light, &SceneView::setLight,
                      py::return_value_policy::reference_internal, "Light")
        .def_property("camera", &SceneView::camera, &SceneView::setCamera,
                      py::return_value_policy::reference_internal, "Camera")
        .def_property("flags", &SceneView::flags, &SceneView::setFlags, "Flags")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self)
        // pickle
        .def(pickle<SceneView>());
}
