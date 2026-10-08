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
#:
#: The order below is the whole mechanism, and it is easy to get backwards:
#: now that ``SRC`` *is* a copy of the SDK, a suite that puts ``SRC`` first
#: imports the vendored one whatever the environment says, and the override
#: becomes a variable that does nothing.  It did nothing, which is why
#: ``tests/test_vendored_sdk.py::TestAMachineWithNothingButThisCheckout::
#: test_the_override_also_wins_over_the_bootstrap`` pins this order from a child
#: interpreter.
SDK_OVERRIDE = os.environ.get("LITEGRIP_SDK_PATH")

# Inserted last-first, because every insert lands at the front — so the first
# entry of this list is the one that ends up ahead of the others.  A candidate
# already on the path is moved rather than left where it is: an inherited
# ``PYTHONPATH`` listing ``src`` first would otherwise keep the vendored copy in
# front, which is the same silence in a harder-to-see place.
for _path in reversed([p for p in (SDK_OVERRIDE, str(SRC)) if p]):
    while _path in sys.path:
        sys.path.remove(_path)
    sys.path.insert(0, _path)

# Must be set before any Qt import happens.  There is no display here, and the
# offscreen platform plugin is the one that runs without one.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
