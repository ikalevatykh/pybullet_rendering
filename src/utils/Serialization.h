// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <nop/protocol.h>
#include <nop/serializer.h>
#include <nop/structure.h>
#include <nop/utility/buffer_reader.h>
#include <nop/utility/buffer_writer.h>
#include <nop/utility/stream_reader.h>
#include <nop/utility/stream_writer.h>
#include <nop/value.h>

#include <sstream>
#include <vector>

template <class T>
inline std::size_t Serialize(const T& value, void* buffer, std::size_t size)
{
    nop::Serializer<nop::BufferWriter> serializer{buffer, size};
    auto status = nop::Protocol<T>::Write(&serializer, value);
    if (!status)
        return 0;
    else
        return serializer.writer().size();
}

template <class T>
inline bool Deserialize(T* value, const std::vector<uint8_t>& buffer)
{
    nop::Deserializer<nop::BufferReader> deserializer{buffer.data(), buffer.size()};
    auto status = nop::Protocol<T>::Read(&deserializer, value);
    if (!status)
        return false;
    else
        return true;
}

template <class T>
inline std::string Serialize(const T& value)
{
    nop::Serializer<nop::StreamWriter<std::stringstream>> serializer;
    auto status = nop::Protocol<T>::Write(&serializer, value);
    if (!status)
        return std::string{};
    else
        return serializer.writer().stream().str();
}

template <class T>
inline bool Deserialize(T* value, const std::string& buffer)
{
    nop::Deserializer<nop::StreamReader<std::stringstream>> deserializer{buffer};
    auto status = nop::Protocol<T>::Read(&deserializer, value);
    if (!status)
        return false;
    else
        return true;
}
