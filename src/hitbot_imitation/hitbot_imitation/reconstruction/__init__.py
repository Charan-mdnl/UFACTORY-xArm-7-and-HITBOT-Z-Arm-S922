"""Stage A: monocular RGB video -> 4D hand-object trajectory.

Importing this package does NOT import the heavy foundation-model backends;
those are loaded lazily inside the concrete adapters when instantiated.
"""
from . import adapters, alignment, guided_tracking, pipeline  # noqa: F401
