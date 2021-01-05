// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

#include <string>

namespace scene {

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
    Mesh(std::string filename) : _inMemory(false), _filename(std::move(filename)) {}

    /**
     * @brief Flag: mesh geometry data stored in memory or on a disk
     * @todo implement in-memory meshes
     */
    bool inMemory() const { return _inMemory; }

    /**
     * @brief Mesh file name
     */
    const std::string& filename() const { return _filename; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Mesh& other) const
    {
        return _inMemory == other._inMemory && _filename == other._filename;
    }
    bool operator!=(const Mesh& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_inMemory, _filename);
    }

  private:
    bool _inMemory;
    std::string _filename;
};

} // namespace scene
