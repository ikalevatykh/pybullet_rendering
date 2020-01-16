// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

// local imports
#include "RenderingPlugin.h"
#include "RenderingInterface.h"

// bullet imports
#include <SharedMemory/SharedMemoryPublic.h>
#include <SharedMemory/plugins/b3PluginContext.h>

// std imports
#include <algorithm>
#include <cstring>
#include <map>

/**
 * @brief Global map physicsClientId -> RenderingingInterface
 *
 */
static std::map<int, RenderingInterface*> gRenderingInterfaces;

/**
 * @brief Set renderer for a specific client
 *
 */
void gSetRenderer(const std::shared_ptr<render::BaseRenderer>& renderer, int physicsClientId)
{
    gRenderingInterfaces.at(physicsClientId)->setRenderer(renderer);
}

B3_SHARED_API int initPlugin_RenderingPlugin(struct b3PluginContext* context)
{
    auto render = new RenderingInterface();
    context->m_userPointer = render;
    return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API void exitPlugin_RenderingPlugin(struct b3PluginContext* context)
{
    auto render = (RenderingInterface*)context->m_userPointer;

    auto it = std::find_if( //
        begin(gRenderingInterfaces), end(gRenderingInterfaces),
        [&render](const auto& it) { return it.second == render; });
    if (it != end(gRenderingInterfaces))
        gRenderingInterfaces.erase(it);

    delete render;
    context->m_userPointer = 0;
}

B3_SHARED_API UrdfRenderingInterface*
    getRenderInterface_RenderingPlugin(struct b3PluginContext* context)
{
    auto render = (RenderingInterface*)context->m_userPointer;
    return render;
}

B3_SHARED_API int executePluginCommand_RenderingPlugin(struct b3PluginContext* context,
                                                       const struct b3PluginArguments* arguments)
{
    auto render = (RenderingInterface*)context->m_userPointer;

    if (0 == strcmp(arguments->m_text, "register")) {
        int physicsClientId = arguments->m_ints[0];
        gRenderingInterfaces.emplace(physicsClientId, render);
        return 0;
    }

    return -1;
}
