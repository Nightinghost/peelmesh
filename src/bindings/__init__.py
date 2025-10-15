try:
    from .peelmesh import *
except ImportError as e:
    raise ImportError(
        "Failed to import the native extension module 'peelmesh'. "
        "Make sure it was built and installed correctly."
    ) from e

__all__ = ["TriangleMesh", "PeelMeshPipeline"]
