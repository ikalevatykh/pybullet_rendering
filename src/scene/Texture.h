// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>
#include <utils/serialization.h>

#include <string>
#include <vector>

namespace scene {

/**
 * @brief Texture description
 *
 */
class Texture
{
  public:
    /**
     * @brief Construct a new Texture object
     *
     */
    Texture() noexcept {};

    /**
     * @brief Construct a new Texture object
     *
     * @param filename - image file name on disk
     */
    explicit Texture(const std::string& filename) : _inMemory(false), _filename(filename) {}

    /**
     * @brief Construct a new Texture object
     *
     * @param data - texture bitmap data
     * @param size - texture size
     */
    Texture(std::vector<uint8_t> data, const Size2i& size)
        : _inMemory(true), _data(std::move(data)), _size(size)
    {
    }

    /**
     * @brief @brief Flag: texture bitmap stored in memory or on a disk
     */
    bool inMemory() const { return _inMemory; }

    /**
     * @brief Texture file name
     */
    const std::string& filename() const { return _filename; }

    /**
     * @brief Texture bitmap (in memory texture)
     */
    const std::vector<uint8_t>& data() const { return _data; }

    /**
     * @brief Texture size (in memory texture)
     */
    const Size2i& size() const { return _size; }

    /**
     * @brief @brief Texture empty
     */
    bool empty() const { return !_inMemory && _filename.empty(); }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Texture& other) const
    {
        return _inMemory == other._inMemory && _filename == other._filename;
    }
    bool operator!=(const Texture& other) const { return !(*this == other); }

  private:
    bool _inMemory;
    std::string _filename;
    std::vector<uint8_t> _data;
    Size2i _size;

    NOP_STRUCTURE(Texture, _inMemory, _filename, _data, _size);
};

} // namespace scene
