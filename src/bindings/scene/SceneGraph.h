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
        .value("Capsule", ShapeType::Capsule)
        .value("Heightfield", ShapeType::Heightfield);

    // Material
    py::class_<Material, std::shared_ptr<Material>>(m, "Material")
        .def_property("diffuse_color", &Material::diffuseColor, &Material::setDiffuseColor,
                      "Diffuse color")
        .def_property("specular_color", &Material::specularColor, &Material::setSpecularColor,
                      "Specular color")
        .def_property("diffuse_texture", &Material::diffuseTexture, &Material::setDiffuseTexture,
                      "Texture index")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Bitmap
    py::class_<Bitmap, std::shared_ptr<Bitmap>>(m, "Bitmap", pybind11::buffer_protocol())
        .def_buffer([](Bitmap& im) -> pybind11::buffer_info {
            return pybind11::buffer_info(const_cast<unsigned char*>(im.data().data()),
                                         sizeof(unsigned char),
                                         pybind11::format_descriptor<unsigned char>::format(),
                                         ssize_t(3), {im.rows(), im.cols(), im.channels()},
                                         {im.channels() * im.cols(), im.channels(), ssize_t(1)});
        });

    // Texture
    py::class_<Texture, std::shared_ptr<Texture>>(m, "Texture")
        .def_property_readonly("filename", &Texture::filename, "Texture filename")
        .def_property_readonly("bitmap", &Texture::bitmap, "Texture bitmap")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // MeshData
    py::class_<MeshData, std::shared_ptr<MeshData>>(m, "MeshData")
        .def_property_readonly(
            "vertices",
            [](const MeshData& self) {
                return py::array_t<float>({ssize_t(self.vertices().size() / 3), ssize_t(3)},
                                          self.vertices().data(), py::cast(self));
            },
            "Vertex coordinates")
        .def_property_readonly(
            "uvs",
            [](const MeshData& self) {
                return py::array_t<float>({ssize_t(self.uvs().size() / 2), ssize_t(2)},
                                          self.uvs().data(), py::cast(self));
            },
            "Vertex texture UV coordinates")
        .def_property_readonly(
            "normals",
            [](const MeshData& self) {
                return py::array_t<float>({ssize_t(self.normals().size() / 3), ssize_t(3)},
                                          self.normals().data(), py::cast(self));
            },
            "Vertex normals")
        .def_property_readonly(
            "faces",
            [](const MeshData& self) {
                return py::array_t<int>({ssize_t(self.indices().size() / 3), ssize_t(3)},
                                        self.indices().data(), py::cast(self));
            },
            "Triangle faces")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Mesh
    py::class_<Mesh, std::shared_ptr<Mesh>>(m, "Mesh")
        .def_property_readonly("filename", &Mesh::filename, "Mesh filename")
        .def_property_readonly("data", &Mesh::data, "Mesh in-memory data")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Shape
    py::class_<Shape>(m, "Shape")
        .def_property_readonly("type", &Shape::type, "Shape type")
        .def_property_readonly("pose", &Shape::pose, "Shape pose relative to a parent node",
                               py::return_value_policy::reference_internal)
        .def_property_readonly("radius", &Shape::radius, "Sphere, cylinder, capsule radius")
        .def_property_readonly("height", &Shape::height, "Cylinder, capsule height")
        .def_property_readonly("extents", &Shape::extents, "Box extents")
        .def_property_readonly("mesh", &Shape::mesh, "Mesh description",
                               py::return_value_policy::reference_internal)
        .def_property("material", &Shape::material, &Shape::setMaterial, "Shape material",
                      py::return_value_policy::reference_internal)
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
    py::class_<SceneGraph, std::shared_ptr<SceneGraph>>(m, "SceneGraph")
        .def_property_readonly("nodes", &SceneGraph::nodes, "Scene nodes",
                               py::return_value_policy::reference_internal)
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self)
        // pickle
        .def(pickle<SceneGraph>());
}
