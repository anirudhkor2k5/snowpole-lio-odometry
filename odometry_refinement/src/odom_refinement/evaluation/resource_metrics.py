"""Runtime bookkeeping for offline experiments.

These numbers describe the machine that ran the script. They are not
edge-device measurements and they are not Faster-LIO runtime.
"""

from __future__ import annotations

import os
import platform
import resource
import time
from contextlib import contextmanager


def peak_rss_mib() -> float | None:
    usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # Linux reports kilobytes.
    if usage <= 0:
        return None
    return float(usage / 1024.0)


@contextmanager
def timed():
    start = time.perf_counter()
    try:
        yield lambda: time.perf_counter() - start
    finally:
        pass


def package_versions() -> dict:
    versions = {}
    for name in ("numpy", "scipy", "matplotlib", "sklearn", "pyproj", "xgboost", "yaml"):
        try:
            module = __import__(name)
        except Exception:
            versions[name] = None
            continue
        versions[name] = getattr(module, "__version__", "present")
    return versions


def host_description() -> dict:
    return {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "machine": platform.machine(),
        "processor": platform.processor() or None,
        "cpu_count": os.cpu_count(),
    }
