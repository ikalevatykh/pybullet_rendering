# Copyright (c) 2019-2020 INRIA.
# This source code is licensed under the LGPLv3 license found in the
# LICENSE file in the root directory of this source tree.

file(GLOB_RECURSE scene_SOURCES "*.cpp")
add_library(scene STATIC ${scene_SOURCES})
