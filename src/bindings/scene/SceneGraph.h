#pragma once

#include <scene/SceneGraph.h>

void bindSceneGraph(py::module& m)
{
    using namespace scene;

    // Affine3f
    py::class_<Affine3f>(m, "Affine")
        .def_readonly("origin", &Affine3f::origin, R"(Origin)")
        .def_readonly("quaternion", &Affine3f::quaternion, R"(Rotation)")
        .def_readonly("scale", &Affine3f::scale, R"(Scale)");

    // Material
    py::class_<Material>(m, "Material")
        .def_property_readonly("diffuse_color",
                               py::overload_cast<>(&Material::diffuseColor, py::const_),
                               R"(Diffuse color)")
        .def_property_readonly("specular_color",
                               py::overload_cast<>(&Material::specularColor, py::const_),
                               R"(Specular color)")
        .def_property_readonly("diffuse_texture",
                               py::overload_cast<>(&Material::diffuseTexture, py::const_),
                               R"(Texture index)");

    // ShapeType enum
    py::enum_<ShapeType>(m, "ShapeType", py::arithmetic())
        .value("Unknown", ShapeType::Unknown)
        .value("Mesh", ShapeType::Mesh)
        .value("Plane", ShapeType::Plane)
        .value("Cube", ShapeType::Cube)
        .value("Sphere", ShapeType::Sphere)
        .value("Cylinder", ShapeType::Cylinder)
        .value("Capsule", ShapeType::Capsule);

    // Texture
    py::class_<Texture>(m, "Texture")
        .def_property_readonly("filename", &Texture::filename,
                               R"(Texture filename)");

    // Mesh
    py::class_<Mesh>(m, "Mesh") //
        .def_property_readonly("filename", &Mesh::filename,
                               R"(Mesh filename)")
        .def_property_readonly("use_materials", &Mesh::useMaterials,
                               R"(Use materials from model file)");

    // Shape
    py::class_<Shape>(m, "Shape")
        .def_property_readonly("type", &Shape::type, R"(Shape type)")
        .def_property_readonly("pose", &Shape::pose, R"(Shape pose relative to a parent node)")
        .def_property_readonly("mesh", &Shape::mesh, R"(Mesh description)")
        .def_property_readonly("material", py::overload_cast<>(&Shape::material, py::const_),
                               R"(Shape material)");

    // Node
    py::class_<Node>(m, "Node")
        .def_property_readonly("body", &Node::body, R"(Body index)")
        .def_property_readonly("link", &Node::link, R"(Link index)")
        .def_property_readonly("cache_graphics", &Node::cacheGraphics,
                               R"(Enable or not caching for child shapes)")
        .def_property_readonly("shapes", &Node::shapes, R"(List of node's child shapes)",
                               py::return_value_policy::reference_internal);

    // SceneGraph
    py::class_<SceneGraph>(m, "SceneGraph")
        .def_property_readonly("nodes", &SceneGraph::nodes, R"(Scene nodes)",
                               py::return_value_policy::reference_internal)
        .def("texture", &SceneGraph::texture, R"(Registered texture)",
             py::return_value_policy::reference_internal);
}
