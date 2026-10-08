"""Bootstrap that must run before anything imports the package under test.

Kept at the repository root, and deliberately free of project imports, so that
every test module can rely on it regardless of how much of the package exists
yet — and so the SDK path can be overridden from the environment without editing
a file that lives next to the tests.

``src/`` is put on the path rather than relying on an installed distribution:
the SDK's declared ``eclipse-zenoh`` dependency is never imported by its library
code and is not present in this environment, so ``pip install`` of either
package is not a prerequisite for running the suite.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
SRC = REPO_ROOT / "src"
#: The SDK is vendored under ``src/litegrip``, so ``SRC`` alone puts it on the
#: path.  ``LITEGRIP_SDK_PATH`` points the suite at a different checkout instead
#: — a developer testing against SDK HEAD — and goes ahead of ``SRC`` so that
#: one is the ``litegrip`` the tests import.
SDK_OVERRIDE = os.environ.get("LITEGRIP_SDK_PATH")

# Inserted last-first, because every insert lands at the front.
for _path in reversed([p for p in (str(SRC), SDK_OVERRIDE) if p]):
    if _path not in sys.path:
        sys.path.insert(0, _path)

# Must be set before any Qt import happens.  There is no display here, and the
# offscreen platform plugin is the one that runs without one.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
