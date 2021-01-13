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

    py::class_<Light, std::shared_ptr<Light>>(m, "Light")
        .def_property_readonly("type", &Light::type, "Light type")
        .def_property_readonly("ambient_color", &Light::ambientColor, "Light ambient color")
        .def_property_readonly("diffuse_color", &Light::diffuseColor, "Light diffuse color")
        .def_property_readonly("specular_color", &Light::specularColor, "Light specular color")
        .def_property_readonly("position", &Light::position, "Light position in world coordinates")
        .def_property("target", &Light::target, &Light::setTarget,
                      "Light target position in world coordinates")
        .def_property("direction", &Light::direction, &Light::setDirection,
                      "Light direction vector in world coordinates")
        .def_property("distance", &Light::distance, &Light::setDistance, "Light distance")
        .def_property("color", &Light::color, &Light::setColor, "Light color")
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

    py::class_<Camera, std::shared_ptr<Camera>>(m, "Camera")
        .def_property_readonly(
            "projection_matrix",
            [](const Camera& self) {
                return py::array_t<float>({4, 4}, self.projMatrix().data(), py::cast(self));
            },
            "Camera projection matrix")
        .def_property_readonly(
            "view_matrix",
            [](const Camera& self) {
                return py::array_t<float>({4, 4}, self.viewMatrix().data(), py::cast(self));
            },
            "Camera view matrix")
        .def_property_readonly(
            "pose_matrix",
            [](const Camera& self) {
                return py::array_t<float>({4, 4}, self.poseMatrix().data());
            },
            "Pose matrix")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    py::class_<SceneView, std::shared_ptr<SceneView>>(m, "SceneView")
        .def_property("viewport", &SceneView::viewport, &SceneView::setViewport, "Image size")
        .def_property("bg_color", &SceneView::backgroundColor, &SceneView::setBackgroundColor,
                      "Background color")
        .def_property("bg_texture", &SceneView::backgroundTexture, &SceneView::setBackgroundTexture,
                      "Background texture index")
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
