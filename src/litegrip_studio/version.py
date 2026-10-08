"""Which build is running.

The console is a control panel, and the first question asked of one that behaved
oddly is which binary it was — so the version is stamped into the artifact at
build time by ``build.sh``, which writes ``_version.py`` next to this file.  That
file is generated and not committed, so a source checkout has no stamp and says
``+source`` instead of inventing one.

Neither number here is a release version.  Releases are automated by
semantic-release from the commit history and the git tag is their only source of
truth (AGENTS.md §3).  A build stamp starts from the nearest ``v*`` tag when the
tree has one, so an artifact can be matched to a release at a glance;
``BASE_VERSION`` is what it starts from when it does not.  It is deliberately not
read from ``pyproject.toml``, whose version field is a placeholder
semantic-release owns.

There is no ``git`` fallback probe, unlike the sibling motor tool.  A source run
can ask git directly, and a console that spawns a subprocess on every start to
learn its own name would be paying for information the person running it already
has.
"""

from __future__ import annotations

#: Where a build stamp starts when git has no ``v*`` tag to offer.  Not a release
#: number — see the module docstring.
#:
#: ``0.0.0`` rather than a plausible-looking release: this used to be ``0.1.0``,
#: which a source run reports as ``0.1.0+source``, and ``v0.1.0`` is a real tag
#: in this repository.  A current checkout therefore read as the first release to
#: anyone comparing versions across machines.  ``0.0.0`` is the same "no release
#: yet" placeholder ``pyproject.toml`` uses, and it cannot collide with a tag
#: semantic-release would ever write.
BASE_VERSION = "0.0.0"

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
