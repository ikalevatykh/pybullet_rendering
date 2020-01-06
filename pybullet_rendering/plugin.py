import os
import pybullet as pb

from .bindings import *


class RenderingPlugin:
    """"A wrapper for rendering plugin"""

    def __init__(self, physicsClientId=0):
        """Load plugin

        Arguments:
            physicsClientId {int or BulletClient} -- physics client
        """
        if not isinstance(physicsClientId, int):
            physicsClientId = physicsClientId._client

        from .bindings import __file__ as plugin_lib

        plugin = pb.loadPlugin(
            plugin_lib, '_RenderingPlugin', physicsClientId=physicsClientId)
        assert plugin != -1, 'Cannot load PyBullet plugin'

        retcode = pb.executePluginCommand(
            plugin, "register", intArgs=[physicsClientId], physicsClientId=physicsClientId)
        assert retcode != -1, 'Cannot register render interface'

        self._client = physicsClientId
        self._plugin = plugin

    def setLocalRenderer(self, renderer:Renderer):
        """Bind a renderer to a local physics client (DIRECT connection)

        Arguments:
            renderer {Renderer} -- Renderer
        """
        setRenderer(renderer, self._client)
