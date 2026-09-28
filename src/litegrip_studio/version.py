"""Which build is running.

The console is a control panel, and the first question asked of one that behaved
oddly is which binary it was — so the version is stamped into the artifact at
build time by ``build.sh``, which writes ``_version.py`` next to this file.  That
file is generated and not committed, so a source checkout has no stamp and says
``+source`` instead of inventing one.

Neither number here is a release version.  Releases are automated by
semantic-release from the commit history and the git tag is their only source of
truth (AGENTS.md §3); ``BASE_VERSION`` is where the build stamp starts counting,
and it is deliberately not read from ``pyproject.toml``, whose version field is a
placeholder semantic-release owns.

There is no ``git`` fallback probe, unlike the sibling motor tool.  A source run
can ask git directly, and a console that spawns a subprocess on every start to
learn its own name would be paying for information the person running it already
has.
"""

from __future__ import annotations

#: Where a build stamp starts.  Not a release number — see the module docstring.
BASE_VERSION = "0.1.0"

#: Appended when there is no stamp, so a source run cannot be mistaken for a
#: built artifact.  PEP 440 local version syntax, which is also what the stamp's
#: ``+g<sha>`` suffix uses.
SOURCE_SUFFIX = "+source"


def resolve_version() -> str:
    """The version string for this process.

    The stamp wins; ``BASE_VERSION`` plus :data:`SOURCE_SUFFIX` is the answer when
    the stamp is absent, which is the normal case for a checkout.
    """
    try:
        from ._version import __version__ as stamped
    except ImportError:
        return BASE_VERSION + SOURCE_SUFFIX
    return str(stamped)
