#pragma once

#include <scene/SceneGraph.h>

PYBIND11_MAKE_OPAQUE(std::map<int, scene::Node>);
PYBIND11_MAKE_OPAQUE(std::vector<scene::Shape>);

template <typename T>
pybind11::array asarray(const std::vector<T>& vector, ssize_t cols, py::handle base)
{
    if (cols > 1)
        return pybind11::array_t<T>({ssize_t(vector.size() / cols), cols},
                                    vector.data(), base);
    else
        return pybind11::array_t<T>(ssize_t(vector.size()), vector.data(), base);
}

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
    py::class_<Texture>(m, "Texture")
        .def_property_readonly("filename", &Texture::filename, "Texture filename")
        .def_property_readonly("bitmap", &Texture::bitmap, "Texture bitmap")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // MeshData
    py::class_<MeshData, std::shared_ptr<MeshData>>(m, "MeshData")
        .def_property_readonly(
            "vertices",
            [](const std::shared_ptr<MeshData>& data) {
                return asarray(data->vertices(), 3, py::cast(data));
            },
            "Vertices coordinates")
        .def_property_readonly(
            "uvs",
            [](const std::shared_ptr<MeshData>& data) {
                return asarray(data->uvs(), 2, py::cast(data));
            },
            "Vertex texture UV coordinates")
        .def_property_readonly(
            "normals",
            [](const std::shared_ptr<MeshData>& data) {
                return asarray(data->normals(), 3, py::cast(data));
            },
            "Vertex normals")
        .def_property_readonly(
            "indices",
            [](const std::shared_ptr<MeshData>& data) {
                return asarray(data->indices(), 1, py::cast(data));
            },
            "Triangle indices")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    // Mesh
    py::class_<Mesh>(m, "Mesh")
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
        .def_property_readonly("mesh", &Shape::mesh, "Mesh description",
                               py::return_value_policy::reference_internal)
        .def_property_readonly("has_material", &Shape::hasMaterial, "Shape has material")
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
        .def("texture", &SceneGraph::texture, "Registered texture",
             py::return_value_policy::reference_internal)
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self)
        // pickle
        .def(pickle<SceneGraph>());
}
