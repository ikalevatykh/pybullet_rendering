// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <utils/math.h>

#include <map>

namespace scene {

/**
 * @brief Scene state
 *
 * Contatins poses for all movable objects on a scene
 */
class SceneState
{
  public:
    /**
     * @brief Append a state for a node
     *
     * @param nodeId - unique node id
     * @param node - node description
     */
    void appendNode(int nodeId) { _poses.emplace(nodeId, Affine3f::Identity()); }

    /**
     * @brief Remove a state fora node
     *
     * @param nodeId - unique node id
     */
    void removeNode(int nodeId) { _poses.erase(nodeId); }

    /**
     * @brief Remove all states
     *
     */
    void clear() { _poses.clear(); }

    /**
     * @brief Number of elements in state
     *
     * @return int
     */
    int size() const { return _poses.size(); }

    const std::map<int, Affine3f>& poses() const { return _poses; }

    /**
     * @brief Pose for a specific node
     *
     * @param id - unique node id
     * @throw std::out_of_range - if no such element exists
     * @return Affine3f&
     */
    const Affine3f& pose(int nodeId) const { return _poses.at(nodeId); }

    /**
     * @brief Update pose for a specific node
     *
     * @param id - unique node id
     * @throw std::out_of_range - if no such element exists
     */
    void setPose(int nodeId, const Affine3f& pose) { _poses.at(nodeId) = pose; }

    /**
     * @brief Comparison operators
     */
    bool operator==(const SceneState& other) const { return _poses == other._poses; }
    bool operator!=(const SceneState& other) const { return !(*this == other); }

    /**
     * @brief Serialization
     */
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(_poses);
    }

  private:
    std::map<int, Affine3f> _poses;
};

} // namespace scene
