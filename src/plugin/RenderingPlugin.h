// Copyright (c) 2019-2020 INRIA.
// This source code is licensed under the LGPLv3 license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <SharedMemory/plugins/b3PluginAPI.h>

#ifdef __cplusplus
extern "C"
{
#endif

	B3_SHARED_API int initPlugin_RenderingPlugin(struct b3PluginContext *context);
	B3_SHARED_API void exitPlugin_RenderingPlugin(struct b3PluginContext *context);
	B3_SHARED_API struct UrdfRenderingInterface *getRenderInterface_RenderingPlugin(struct b3PluginContext *context);
	B3_SHARED_API int executePluginCommand_RenderingPlugin(struct b3PluginContext *context, const struct b3PluginArguments *arguments);

#ifdef __cplusplus
};
#endif
