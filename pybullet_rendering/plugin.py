# Copyright (c) 2019-2020 INRIA.
# This source code is licensed under the LGPLv3 license found in the
# LICENSE file in the root directory of this source tree.

from typing import Union

import pybullet as pb
from pybullet_utils.bullet_client import BulletClient

from .bindings import BaseRenderer
from .bindings import __file__ as plugin_lib_file
from .bindings import set_renderer


class RenderingPlugin:
    """"A wrapper for rendering plugin."""

    def __init__(self, client: Union[int, BulletClient] = 0, renderer: BaseRenderer = None):
        """Load plugin.

        Arguments:
            physicsClientId {int or BulletClient} -- physics client

        Keyword Arguments:
            renderer {BaseRenderer} -- renderer to load  (default: None)
        """
        self._client_id = client if isinstance(client, int) else client._client
        # check the simulation world is empty
        num_bodies = pb.getNumBodies(physicsClientId=self._client_id)
        assert num_bodies == 0, 'Cannot load the rendering plugin: simulation world is not empty'
        # load plugin
        self._plugin_id = pb.loadPlugin(plugin_lib_file,
                                        '_RenderingPlugin',
                                        physicsClientId=self._client_id)
        assert self._plugin_id != -1, 'Cannot load the rendering plugin'
        # workaround to connect python bindings with a physicsClientId
        retcode = pb.executePluginCommand(self._plugin_id,
                                          "register",
                                          intArgs=[self._client_id],
                                          physicsClientId=self._client_id)
        assert retcode != -1, 'Cannot register render interface'
        # bind a renderer
        self._renderer = None
        if renderer is not None:
            self.set_renderer(renderer)
        # workaround to keep a link to the renderer on the Python side
        if isinstance(client, BulletClient):
            setattr(client, '_rendering_plugin', self)

    @property
    def renderer(self):
        """Loaded renderer.

        Returns:
            renderer {BaseRenderer} -- loaded renderer
        """
        return self._renderer

    def set_renderer(self, renderer: BaseRenderer):
        """Bind a renderer to a local physics client (DIRECT connection).

        Arguments:
            renderer {Renderer} -- Renderer
        """
        set_renderer(renderer, self._client_id)
        self._renderer = renderer

    def unload(self):
        """Unload plugin."""
        if self._plugin_id != -1:
            pb.unloadPlugin(self._plugin_id, physicsClientId=self._client_id)
            self._plugin_id = -1
            self._renderer = None
