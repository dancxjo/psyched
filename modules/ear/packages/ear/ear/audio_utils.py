"""Shared helpers for manipulating raw PCM payloads.

These utilities intentionally avoid ROS-specific types so they can be unit
tested without sourcing the ROS environment.
"""
from __future__ import annotations

from collections.abc import Iterable, Sequence
from typing import Any


def coerce_pcm_bytes(payload: Any) -> bytes:
    """Return *payload* as a ``bytes`` object.

    The Ear pipeline frequently passes audio around using
    :class:`std_msgs.msg.ByteMultiArray`, which exposes its ``data`` field as a
    sequence of integers.  Downstream consumers such as the VAD and
    transcription backends, however, expect a contiguous byte buffer.  This
    helper normalises the various container types we may encounter into a
    ``bytes`` object.

    Parameters
    ----------
    payload:
        The raw audio payload.  This can be a ``bytes`` object, ``bytearray``,
        :class:`memoryview`, any object exposing ``tolist`` (e.g. ``numpy``
        arrays), or a plain iterable of integers.

    Returns
    -------
    bytes
        The payload re-encoded as bytes suitable for audio processing.

    Raises
    ------
    TypeError
        If the payload cannot be interpreted as a byte sequence.

    Examples
    --------
    >>> coerce_pcm_bytes([0, 255])
    b'\x00\xff'
    >>> coerce_pcm_bytes(memoryview(b'hi'))
    b'hi'
    """
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, bytearray):
        return bytes(payload)
    if isinstance(payload, memoryview):
        return payload.tobytes()

    if hasattr(payload, 'tobytes'):
        try:
            return bytes(payload.tobytes())  # type: ignore[arg-type]
        except Exception:
            pass

    if hasattr(payload, 'tolist'):
        try:
            payload = payload.tolist()
        except Exception as exc:  # pragma: no cover - defensive
            raise TypeError('Failed to convert payload via tolist()') from exc

    if isinstance(payload, Sequence) or isinstance(payload, Iterable):
        # Fast-path: many callers pass a sequence of integers which bytes()
        # can consume directly (e.g. list[int]). If the sequence contains
        # bytes-like objects (for example b'\xd8' elements) bytes() will
        # raise; in that case fall back to flattening each element into a
        # contiguous bytearray.
        try:
            return bytes(payload)  # type: ignore[arg-type]
        except Exception:
            out = bytearray()
            for item in payload:
                # integer values are appended directly
                if isinstance(item, int):
                    out.append(item & 0xFF)
                    continue

                # bytes-like objects: extend with their bytes
                if isinstance(item, (bytes, bytearray, memoryview)):
                    out.extend(bytes(item))
                    continue

                # fallback: try to coerce to int
                try:
                    out.append(int(item) & 0xFF)
                except Exception as exc:  # pragma: no cover - defensive
                    raise TypeError(f'Unsupported item in payload sequence: {type(item)!r}') from exc

            return bytes(out)

    raise TypeError(f'Unsupported audio payload type: {type(payload)!r}')


__all__ = ['coerce_pcm_bytes']
