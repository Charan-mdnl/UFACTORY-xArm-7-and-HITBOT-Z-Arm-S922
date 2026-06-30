"""Grasp/release detection and gripper-signal generation.

The paper defines reference "rest" vs "in-hand" stages by thresholding the
hand-object distance (Sec. 3.2, Transition Reward).  Stage A already encodes
that as a per-frame boolean ``contact`` flag; here we turn it into clean grasp
*segments* and a smooth gripper command.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GraspSegment:
    """A contiguous span ``[start, end]`` (inclusive) where the object is held."""

    start: int
    end: int

    def __len__(self) -> int:
        return self.end - self.start + 1


def detect_segments(contact: np.ndarray, min_len: int = 2) -> list[GraspSegment]:
    """Find contiguous in-contact spans, dropping ones shorter than ``min_len``.

    Short blips are reconstruction noise (a single frame of spurious contact),
    so we discard them rather than command a meaningless grasp.
    """
    contact = np.asarray(contact, dtype=bool)
    segments: list[GraspSegment] = []
    start = None
    for i, c in enumerate(contact):
        if c and start is None:
            start = i
        elif not c and start is not None:
            if i - start >= min_len:
                segments.append(GraspSegment(start, i - 1))
            start = None
    if start is not None and len(contact) - start >= min_len:
        segments.append(GraspSegment(start, len(contact) - 1))
    return segments


def contact_to_gripper(contact: np.ndarray,
                       segments: list[GraspSegment] | None = None,
                       lead: int = 2) -> np.ndarray:
    """Build a gripper command in [0,1] (0 open, 1 closed) from contact.

    The gripper is told to *start closing* ``lead`` frames before the grasp
    onset so it is shut by the time the tool reaches the object — the discrete
    analogue of the paper's warmup, where the hand pre-shapes before contact.
    """
    contact = np.asarray(contact, dtype=bool)
    n = len(contact)
    if segments is None:
        segments = detect_segments(contact)
    g = np.zeros(n)
    for seg in segments:
        lo = max(0, seg.start - lead)
        g[lo:seg.end + 1] = 1.0
    return g


def transition_indices(segments: list[GraspSegment]) -> dict:
    """Return the key inflection frames per segment (grasp onset / release)."""
    return {
        "grasp": [s.start for s in segments],
        "release": [s.end for s in segments],
    }
