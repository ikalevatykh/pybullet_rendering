// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Shape.h"

#include <vector>

namespace scene {

/**
 * @brief Scene node
 *
 */
class Node
{
  public:
    /**
     * @brief Construct a new Node object
     */
    Node() noexcept = default;

    /**
     * @brief Construct a new Node object
     */
    Node(int body, int link, std::vector<Shape>&& shapes, bool noCache = false)
        : _body(body), _link(link), _noCache(noCache), _shapes(std::move(shapes))
    {
    }

    /**
     * @brief Body index
     */
    int body() const { return _body; }

    /**
     * @brief Link index
     */
    int link() const { return _link; }

    /**
     * @brief Disable caching for shapes associated with this node
     */
    bool noCache() const { return _noCache; }

    /**
     * @brief Vector of object's shapes
     *
     * @return const container
     */
    const std::vector<Shape>& shapes() const { return _shapes; }

    /**
     * @brief Specific shape
     *
     * @param index - shape index
     * @throw std::out_of_range - if no such element exists
     * @return Shape&
     */
    Shape& shape(int index) { return _shapes.at(index); }
    /** @overload */
    const Shape& shape(int index) const { return _shapes.at(index); }

    /**
     * @brief Comparison operators
     */
    bool operator==(const Node& other) const
    {
        return _body == other._body && _link == other._link && _noCache == other._noCache &&
               _shapes == other._shapes;
    }
    bool operator!=(const Node& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_body, _link, _noCache, _shapes);
    }

  private:
    int _body;
    int _link;
    bool _noCache;
    std::vector<Shape> _shapes;
};

} // namespace scene
