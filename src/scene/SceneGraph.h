// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Node.h"

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
    void changeShapeTexture(int nodeId, int shapeIndex, const std::shared_ptr<Texture>& texture)
    {
        auto& shape = _nodes.at(nodeId).shape(shapeIndex);
        auto material = !shape.material() ? std::make_shared<Material>() : shape.material();
        material->setDiffuseTexture(texture);
        shape.setMaterial(material);
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
        auto& shape = _nodes.at(nodeId).shape(shapeIndex);
        auto material = !shape.material() ? std::make_shared<Material>() : shape.material();
        material->setDiffuseColor(color);
        shape.setMaterial(material);
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

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_nodes, _textures);
    }

  private:
    std::map<int, Node> _nodes;
    // assets
    std::vector<Texture> _textures;
};

} // namespace scene
