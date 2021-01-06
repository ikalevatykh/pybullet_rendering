// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

#include <string>

namespace scene {

/**
 * @brief In-memory mesh data
 *
 */
class MeshData
{
  public:
    /**
     * @brief Construct a new MeshData object
     *
     */
    MeshData() noexcept = default;

    /**
     * @brief Construct a new MeshData object
     *
     * @param vertices - vertex coordinates
     * @param uvs - texture coordinates
     * @param normals - vertex normals
     * @param indices - triangle indices
     */
    MeshData(std::vector<float>&& vertices, std::vector<float>&& uvs, std::vector<float>&& normals,
             std::vector<int>&& indices) noexcept
        : _vertices(std::move(vertices)), _uvs(std::move(uvs)), _normals(std::move(normals)),
          _indices(std::move(indices))
    {
    }

    /**
     * @brief Vertex coordinates
     */
    const std::vector<float>& vertices() const { return _vertices; }

    /**
     * @brief Vertex texture coordinates
     */
    const std::vector<float>& uvs() const { return _uvs; }

    /**
     * @brief Vertex normals
     */
    const std::vector<float>& normals() const { return _normals; }

    /**
     * @brief Triangle indices
     */
    const std::vector<int>& indices() const { return _indices; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const MeshData& other) const
    {
        return _vertices == other._vertices && _uvs == other._uvs && _normals == other._normals &&
               _indices == other._indices;
    }
    bool operator!=(const MeshData& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_vertices, _uvs, _normals, _indices);
    }

  private:
    std::vector<float> _vertices;
    std::vector<float> _uvs;
    std::vector<float> _normals;
    std::vector<int> _indices;
};

/**
 * @brief Mesh description
 *
 */
class Mesh
{
  public:
    /**
     * @brief Construct a new empty Mesh object
     *
     */
    Mesh() noexcept = default;

    /**
     * @brief Construct a new Mesh object
     *
     * @param filename - mesh model file on disk
     */
    explicit Mesh(const std::string& filename) : _filename(filename) {}

    /**
     * @brief Construct a new Mesh object
     *
     * @param data - in-memory mesh data
     */
    explicit Mesh(const std::shared_ptr<MeshData>& data) : _data(data) {}

    /**
     * @brief Mesh file name
     */
    const std::string& filename() const { return _filename; }

    /**
     * @brief Mesh file name
     */
    const std::shared_ptr<MeshData>& data() const { return _data; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Mesh& other) const
    {
        return _filename == other._filename &&
               (_data == other._data || _data && other._data && *_data == *other._data);
    }
    bool operator!=(const Mesh& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_filename, _data);
    }

  private:
    std::string _filename;
    std::shared_ptr<MeshData> _data;
};

} // namespace scene
