"""The Qt layer, and the only place PyQt5 is imported other than the worker.

Widgets here are deliberately thin.  Every decision that could be wrong in a way
that matters — what the gate permits, when a command is issued, what a stopped
motor does — is made in :mod:`litegrip_studio.core` and merely *displayed*
here.  A widget that decided anything would be a widget that could decide it
wrongly, and this is the layer where that is hardest to test.

The one thing the UI does own is *when* the operator's intent becomes a command:
a drag becomes a :class:`~litegrip_studio.core.commands.MoveToMm` on release,
not on every pixel.  That is an interaction decision, and it lives with the
interaction.
"""
