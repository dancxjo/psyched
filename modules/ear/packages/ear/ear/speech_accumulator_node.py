"""Backwards-compatibility shim for the renamed segment accumulator node."""
from __future__ import annotations

from .segment_accumulator_node import (  # noqa: F401
    SegmentAccumulator,
    SegmentAccumulatorNode,
    main,
)

__all__ = ['SegmentAccumulator', 'SegmentAccumulatorNode', 'main']
