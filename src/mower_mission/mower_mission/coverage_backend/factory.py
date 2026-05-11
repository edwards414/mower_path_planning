"""Backend factory — select and instantiate a CoverageBackend by name."""

from __future__ import annotations

import logging

_log = logging.getLogger(__name__)


def create_backend(name: str, allow_fallback: bool = True):
    """Return a CoverageBackend for *name* ('python' or 'rust').

    When allow_fallback is False and the Rust extension is unavailable,
    raise RuntimeError instead of falling back to Python.
    """
    if name == 'rust':
        try:
            from .rust_backend import RustBackend
            return RustBackend()
        except ImportError as exc:
            if allow_fallback:
                _log.warning(
                    'Rust backend unavailable (%s), falling back to Python', exc,
                )
                from .python_backend import PythonBackend
                return PythonBackend()
            raise RuntimeError(
                f'Rust backend requested but not available: {exc}'
            ) from exc
    elif name == 'python':
        from .python_backend import PythonBackend
        return PythonBackend()
    else:
        raise ValueError(
            f'Unknown coverage_backend "{name}"; expected "python" or "rust"'
        )
