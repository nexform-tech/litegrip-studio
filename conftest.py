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
#: A checkout beside this one, which is where a workspace holding both
#: repositories puts it; LITEGRIP_SDK_PATH overrides that.
SDK = Path(os.environ.get("LITEGRIP_SDK_PATH") or REPO_ROOT.parent / "lite-grip")

for _path in (str(SRC), str(SDK)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

# Must be set before any Qt import happens.  There is no display here, and the
# offscreen platform plugin is the one that runs without one.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
