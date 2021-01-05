// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <cereal/archives/binary.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <sstream>

template <class T>
inline std::string BinarySerialize(const T& object)
{
    std::ostringstream ss;
    cereal::BinaryOutputArchive archive{ss};
    archive(object);
    return ss.str();
}

template <class T>
inline void BinaryDeserialize(const std::string& buffer, T& object)
{
    std::istringstream ss{buffer};
    cereal::BinaryInputArchive archive{ss};
    archive(object);
}
