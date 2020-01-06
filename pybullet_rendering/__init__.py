#!/usr/bin/env python3





import builtins

__version__ = "0.1.0"

if not getattr(builtins, "__PLUGIN_SETUP__", False):
    from .plugin import *
