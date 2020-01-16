#pragma once

#include <scene/SceneGraph.h>

PYBIND11_MAKE_OPAQUE(std::map<int, scene::Node>);
PYBIND11_MAKE_OPAQUE(std::vector<scene::Shape>);

void bindSceneGraph(py::module& m)
{
    using namespace scene;

    py::bind_map<std::map<int, Node>>(m, "MapIntNode");
    py::bind_vector<std::vector<Shape>>(m, "VectorShape");

    // ShapeType enum
    py::enum_<ShapeType>(m, "ShapeType", py::arithmetic())
        .value("Unknown", ShapeType::Unknown)
        .value("Mesh", ShapeType::Mesh)
        .value("Plane", ShapeType::Plane)
        .value("Cube", ShapeType::Cube)
        .value("Sphere", ShapeType::Sphere)
        .value("Cylinder", ShapeType::Cylinder)
        .value("Capsule", ShapeType::Capsule);

    // Material
    py::class_<Material>(m, "Material")
        .def_property("diffuse_color", &Material::diffuseColor, &Material::setDiffuseColor,
                      "Diffuse color")
        .def_property("specular_color", &Material::specularColor, &Material::setSpecularColor,
                      "Specular color")
        .def_property("diffuse_texture", &Material::diffuseTexture, &Material::setDiffuseTexture,
                      "Texture index")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Texture
    py::class_<Texture>(m, "Texture")
        .def_property_readonly("filename", &Texture::filename, "Texture filename")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Mesh
    py::class_<Mesh>(m, "Mesh")
        .def_property_readonly("filename", &Mesh::filename, "Mesh filename")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Shape
    py::class_<Shape>(m, "Shape")
        .def_property_readonly("type", &Shape::type, "Shape type")
        .def_property_readonly("pose", &Shape::pose, "Shape pose relative to a parent node")
        .def_property_readonly("mesh", &Shape::mesh, "Mesh description")
        .def_property_readonly("has_material", &Shape::hasMaterial, "Shape has material")
        .def_property("material", &Shape::material, &Shape::setMaterial, "Shape material")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Node
    py::class_<Node>(m, "Node")
        .def_property_readonly("body", &Node::body, "Body index")
        .def_property_readonly("link", &Node::link, "Link index")
        .def_property_readonly("no_cache", &Node::noCache, "Disable caching for child shapes")
        .def_property_readonly("shapes", &Node::shapes, "List of node's child shapes",
                               py::return_value_policy::reference_internal)
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // SceneGraph
    py::class_<SceneGraph>(m, "SceneGraph")
        .def_property_readonly("nodes", &SceneGraph::nodes, "Scene nodes",
                               py::return_value_policy::reference_internal)
        .def("texture", &SceneGraph::texture, "Registered texture",
             py::return_value_policy::reference_internal)
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self)
        // pickle
        .def(pickle<SceneGraph>());
}
