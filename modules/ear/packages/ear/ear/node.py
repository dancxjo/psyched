"""Compatibility shim exposing EarNode from the transcriber implementation."""

from __future__ import annotations

from .transcriber_node import EarNode, TranscriberNode, main

__all__ = ["EarNode", "TranscriberNode", "main"]
