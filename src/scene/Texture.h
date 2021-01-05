// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

#include <memory>
#include <string>
#include <vector>

namespace scene {

/**
 * @brief Bitmap data
 *
 */
class Bitmap
{
  public:
    /**
     * @brief Construct a new Bitmap object
     *
     */
    Bitmap() noexcept : _size{0, 0} {};

    /**
     * @brief Construct a new Bitmap object
     *
     * @param data - bitmap data
     * @param size - bitmap size
     */
    Bitmap(std::vector<uint8_t>&& data, const Size2i& size) : _data(std::move(data)), _size(size) {}

    /**
     * @brief Bitmap rows
     */
    ssize_t rows() const { return _size[0]; }

    /**
     * @brief Bitmap cols
     */
    ssize_t cols() const { return _size[1]; }

    /**
     * @brief Bitmap channels
     */
    ssize_t channels() const
    {
        auto sq = _size[0] * _size[1];
        return sq > 0 ? _data.size() / sq : 0;
    }

    /**
     * @brief Bitmap data
     */
    const std::vector<uint8_t>& data() const { return _data; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Bitmap& other) const
    {
        return _size == other._size && _data == other._data;
    }
    bool operator!=(const Bitmap& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_size, _data);
    }

  private:
    Size2i _size;
    std::vector<uint8_t> _data;
};

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
    Texture() noexcept = default;

    /**
     * @brief Construct a new Texture object
     *
     * @param filename - image file name on disk
     */
    explicit Texture(const std::string& filename) : _filename{filename} {}

    /**
     * @brief Construct a new Texture object
     *
     * @param data - texture bitmap data
     * @param size - texture size
     */
    Texture(std::vector<uint8_t>&& data, const Size2i& size)
        : _bitmap(std::make_shared<Bitmap>(std::move(data), size))
    {
    }

    /**
     * @brief Texture file name
     */
    const std::string& filename() const { return _filename; }

    /**
     * @brief Texture bitmap (in memory texture)
     */
    const std::shared_ptr<Bitmap>& bitmap() const { return _bitmap; }

    /**
     * @brief @brief Texture empty
     */
    bool empty() const { return _filename.empty() && !_bitmap; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Texture& other) const
    {
        return _filename == other._filename &&
               (_bitmap == other._bitmap || _bitmap && other._bitmap && *_bitmap == *other._bitmap);
    }
    bool operator!=(const Texture& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_filename, _bitmap);
    }

  private:
    std::string _filename;
    std::shared_ptr<Bitmap> _bitmap;
};

} // namespace scene
