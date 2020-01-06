// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Node.h"
// #include "SceneAssets.h"
#include <utils/Serialization.h>

#include <algorithm>
#include <map>
#include <set>

namespace scene {

/**
 * @brief Scene graph description
 *
 */
class SceneGraph
{
  public:
    /**
     * @brief Map id - node
     *
     * @return const container
     */
    const std::map<int, Node>& nodes() const { return _nodes; }

    /**
     * @brief Append an object to the scene
     *
     * @param nodeId - unique node id
     * @param node - node description
     */
    void appendNode(int nodeId, Node node) { _nodes.emplace(nodeId, std::move(node)); }

    /**
     * @brief Remove an object from the scene
     *
     * @param nodeId - unique node id
     */
    void removeNode(int nodeId) { _nodes.erase(nodeId); }

    /**
     * @brief Change shape texture
     *
     * @param nodeId - unique nodeId id
     * @param shapeIndex - shape index inside the node
     * @param texture - shape diffuse texture
     */
    void changeShapeTexture(int nodeId, int shapeIndex, int textureId)
    {
        _nodes
            .at(nodeId) //
            .shape(shapeIndex)
            .material()
            .diffuseTexture(textureId);
    }

    /**
     * @brief Change shape color
     *
     * @param nodeId - unique nodeId id
     * @param shapeIndex - shape index inside the node
     * @param color - shape diffuse color
     */
    void changeShapeColor(int nodeId, int shapeIndex, const Color4f& color)
    {
        _nodes
            .at(nodeId) //
            .shape(shapeIndex)
            .material()
            .diffuseColor(color);
    }

    /**
    * @brief Registered texture
    *
    * @param textureId - unique texture id
    * @throw std::out_of_range - if no such element exists
    * @return Texture
    */
    const Texture& texture(int textureId) const { return _textures.at(textureId); }

    /**
     * @brief Register new texture
     *
     * @param Texture - texture to register
     * @return unique texture index
     */
    int registerTexture(const Texture& texture)
    {
        auto it = std::find_if( //
            begin(_textures), end(_textures), //
            [&texture](const auto& t) { return t == texture; });
        if (it != end(_textures))
            return std::distance(begin(_textures), it);

        _textures.emplace_back(texture);
        return _textures.size() - 1;
    }

    /**
     * @brief Clear scene graph
     *
     */
    void clear()
    {
        _nodes.clear();
        _textures.clear();
    }

    /**
     * @brief Comparison operators
     */
    bool operator==(const SceneGraph& other) const
    {
        return _nodes == other._nodes && _textures == other._textures;
    }
    bool operator!=(const SceneGraph& other) const { return !(*this == other); }

  private:
    std::map<int, Node> _nodes;
    // assets
    std::vector<Texture> _textures;

    NOP_STRUCTURE(SceneGraph, _nodes, _textures);
};

} // namespace scene
