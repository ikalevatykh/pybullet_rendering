// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Math.h"
#include <utils/Serialization.h>

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
     * @param useMaterials - use or not materials associated with this mesh
     */
    Mesh(std::string filename, bool useMaterials)
        : _inMemory(false), _filename(std::move(filename)), _useMaterials(useMaterials)
    {
    }

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
     * @brief Flag: use or not materials associated with this mesh
     */
    bool useMaterials() const { return _useMaterials; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Mesh& other) const
    {
        return _filename == other._filename && _useMaterials == other._useMaterials;
    }
    bool operator!=(const Mesh& other) const { return !(*this == other); }

  private:
    bool _inMemory;
    std::string _filename;
    bool _useMaterials;

    NOP_STRUCTURE(Mesh, _filename, _useMaterials);
};

} // namespace scene
