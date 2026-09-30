"""The console's colours, and the stylesheet built from them.

Named here rather than spelled at each call site because several of them carry
meaning across widgets: the slider's thin line and the target readout are the
same colour, the actual-position triangle and the "达到" state are the same
green.  A colour that means "the jaws really are here" must not be a different
green in the plot than it is on the slider.

Two things about this module are deliberate.

*The tokens are litearm-studio's.*  The console shares the arm console's design
language: the same seven-step ink ramp, the same three-step line ramp, the same
semantic triple of a colour with its ``_SOFT`` fill and its ``_LINE`` edge, and
the same radii and type scale.  ``BACKGROUND``, ``PANEL``, ``TEXT`` and friends
below are the local spellings of ``--app-bg``, ``--card``, ``--ink`` and the
rest of that token set, so a change made there has an obvious counterpart here.
What is *not* shared is the meaning layer — this console's ``TARGET`` and
``ACTUAL`` are the gripper's commanded and measured position, and litearm-studio
has no equivalent because an arm console has no jaws.

*The default is dark, and light is one call away.*  A bench under a machine
tends to be dark, so dark is what the console opens in when nobody has said
otherwise — the same theme it opened in before there was a choice, which is why
"no preference" and "an unreadable preference" resolve to the same thing.  A dim
console is also the one that does not light the operator up while they are
looking at a mechanism.  :func:`set_theme` switches the whole application, and
:func:`subscribe` lets a widget that has already painted the old palette
re-render itself — without it, every readout that was written as rich text would
keep the colours of the theme it was written under.

The palette is applied to the whole application rather than per widget, so a Qt
dialog the console did not write (a file chooser, a message box) does not come
back bright white in the middle of it.
"""

from __future__ import annotations

import inspect
import weakref
from dataclasses import dataclass
from typing import Any, Callable

DARK = "dark"
LIGHT = "light"

#: What the console opens in when nobody has expressed a preference.
DEFAULT_THEME = DARK

@dataclass(frozen=True)
class Palette:
    """One theme's worth of tokens.

    The field names are the token names, lowercased; :func:`_bind` exports each
    of them to a module-level constant in upper case.  Keeping them in a table
    rather than in the stylesheet is what makes the two themes provably
    symmetrical: a token added to one and forgotten in the other is a
    :class:`TypeError` at import, not a colour that silently goes missing.
    """

    name: str
    label: str

    # ── surfaces ────────────────────────────────────────────────────────────
    background: str  # the shell, and the inset behind an embedded log
    panel: str  # a card: the surface a group box or a dock is drawn on
    panel_alt: str  # inputs, the tab track, a section band
    elevated: str  # hovered rows, a menu, the selected tab
    hover: str  # a translucent wash over a panel
    groove: str  # the slider's unfilled track, and a progress bar's trough
    overlay: str  # a floating chip over the 3D-free panels

    # ── lines ───────────────────────────────────────────────────────────────
    border: str
    border_strong: str
    border_soft: str

    # ── ink ─────────────────────────────────────────────────────────────────
    text: str  # the strongest text: titles and the number being watched
    text_strong: str  # button labels, sub-headings
    text_muted: str  # captions, labels, the section name on a card
    text_soft: str
    text_subtle: str
    text_faint: str  # placeholders, chart ticks
    text_disabled: str

    # ── meaning ─────────────────────────────────────────────────────────────
    target: str  # commanded position: the slider line, the setpoint trace
    target_soft: str
    target_line: str
    actual: str  # measured position: the triangle, the position trace
    actual_soft: str
    actual_line: str
    grasped: str  # a grasp is being held
    grasped_soft: str
    grasped_line: str
    warn: str
    warn_soft: str
    warn_line: str
    error: str
    error_soft: str
    error_line: str

    # ── the filled danger ───────────────────────────────────────────────────
    # ``error`` above is an *ink*: a colour chosen so that danger text is
    # legible on this theme's surfaces, which on the dark theme makes it a pale
    # salmon.  Filling a button with it gives a pale pink pill with dark text on
    # it — the opposite of alarming — so a control that is *filled* with danger
    # takes these instead: a saturated red, and the ink that sits on it.  Two
    # tokens rather than one, because "the colour danger is written in" and "the
    # colour danger is painted in" are different jobs, and conflating them is
    # exactly the mistake this pair exists to prevent.
    danger_solid: str
    danger_solid_fg: str


#: Dark: the console's own palette, and the one litearm-studio's ``.dark`` block
#: defines.  The surfaces run page < card < band < raised, so a card floats on
#: the shell and the selected tab floats on its track.
_DARK = Palette(
    name=DARK,
    label="深色",
    background="#0e151d",
    panel="#1a2330",
    panel_alt="#232e3b",
    elevated="#2c3745",
    hover="rgba(255, 255, 255, 0.05)",
    groove="#232e3b",
    overlay="rgba(14, 21, 29, 0.86)",
    border="#2c3745",
    border_strong="#3d4a5c",
    border_soft="#232e3b",
    text="#e7edf6",
    text_strong="#ccd6e4",
    text_muted="#a7b4c7",
    text_soft="#98a6ba",
    text_subtle="#8f9db2",
    text_faint="#6f7d90",
    text_disabled="#56637a",
    target="#7aaeff",
    target_soft="#16233a",
    target_line="#294067",
    actual="#4ade80",
    actual_soft="#16301f",
    actual_line="#245034",
    grasped="#a78bfa",
    grasped_soft="#241a3d",
    grasped_line="#3f2f66",
    warn="#f5c76b",
    warn_soft="#33280f",
    warn_line="#5c4718",
    error="#f08a8f",
    error_soft="#35171a",
    error_line="#5c2a2e",
    danger_solid="#e5484d",
    danger_solid_fg="#ffffff",
)

#: Light: the same token names with the same relationships, inverted.  Kept for
#: a bench under daylight or a projector, where a dark console is the one that
#: cannot be read.
_LIGHT = Palette(
    name=LIGHT,
    label="浅色",
    background="#f6f7f9",
    panel="#ffffff",
    panel_alt="#f1f4f9",
    elevated="#ffffff",
    hover="rgba(23, 33, 47, 0.04)",
    groove="#e4e9f0",
    overlay="rgba(255, 255, 255, 0.82)",
    border="#e4e9f0",
    border_strong="#c3cbd6",
    border_soft="#f1f4f9",
    text="#17212f",
    text_strong="#3c4a5c",
    text_muted="#46596f",
    text_soft="#5d6b7d",
    text_subtle="#6b7787",
    text_faint="#9aa6b6",
    text_disabled="#b3bcc8",
    target="#2563eb",
    target_soft="#eaf2ff",
    target_line="#cddcfa",
    actual="#137a44",
    actual_soft="#eaf7ef",
    actual_line="#c6e9d3",
    grasped="#7c3aed",
    grasped_soft="#f3ecff",
    grasped_line="#ddd0fb",
    warn="#8a6410",
    warn_soft="#fff7e2",
    warn_line="#f2c86b",
    error="#c62b30",
    error_soft="#fdecec",
    error_line="#f3c9ca",
    danger_solid="#c62b30",
    danger_solid_fg="#ffffff",
)

PALETTES: dict[str, Palette] = {DARK: _DARK, LIGHT: _LIGHT}

#: The palette every module-level constant below currently holds.
_ACTIVE: Palette = _DARK

# ── geometry ────────────────────────────────────────────────────────────────
#: The card radius, and the base every other radius is derived from: litearm's
#: ``--radius: 0.625rem`` lands here once the fluid root font size is taken out
#: of the sum.
RADIUS = 8
#: A group box is a card and has one step more, the way litearm's cards do.
RADIUS_CARD = 10
#: Anything that should read as a pill: a status chip, a tab.
RADIUS_PILL = 13

#: The inner padding of a card, and the gap between cards.
PAD = 10
GAP = 10

#: The margins a tab page lays its content out in.
#:
#: Zero *horizontally*, because the tab widget already sits inside the window's
#: own gutter: a page that adds another one indents its first card relative to
#: the header card directly above it, which reads as a mistake — and did, until
#: a reviewer asked whether it was one.  The top gap is the page's, to keep the
#: first card off the tab strip.
#:
#: Nothing here sets this by default: a bare ``QVBoxLayout`` carries the style's
#: own margins (9px on this platform), so leaving it out silently indents the
#: page.  Pages assign this explicitly for that reason.
PAGE_MARGINS = (0, GAP, 0, 0)

#: The UI face, and the face numbers are set in.
#:
#: The first family that exists wins, and the list is the one litearm-studio
#: asks the browser for — a CJK face is named before the Latin fallbacks so the
#: Chinese labels are not drawn from a substituted font.
UI_FAMILY = (
    "system-ui, PingFang SC, HarmonyOS Sans SC, Microsoft YaHei, "
    "Noto Sans CJK SC, DejaVu Sans, sans-serif"
)

#: Monospaced, so a number that changes at 50 Hz does not make the layout
#: twitch: proportional digits would shift every neighbouring label as the
#: value goes from 9.9 to 10.0.
MONO_FAMILY = "JetBrains Mono, DejaVu Sans Mono, Consolas, monospace"

#: Body size, and the size a caption or a chart tick is set in.
FONT_PX = 13
FONT_SMALL_PX = 11


#: The colours a severity is painted in: the text, the wash behind it and the
#: edge of it.  One table, so a banner and a chip of the same severity cannot
#: disagree.  Refreshed by :func:`_bind` on every theme change.
SEVERITY: dict[str, tuple[str, str, str]] = {}


def _severity_table(palette: Palette) -> dict[str, tuple[str, str, str]]:
    """The severity triples for ``palette``."""
    return {
        "ok": (palette.actual, palette.actual_soft, palette.actual_line),
        "info": (palette.target, palette.target_soft, palette.target_line),
        "warn": (palette.warn, palette.warn_soft, palette.warn_line),
        "error": (palette.error, palette.error_soft, palette.error_line),
    }


def severity_colours(severity: str) -> tuple[str, str, str]:
    """``(text, fill, edge)`` for a severity, defaulting to the muted ink."""
    return SEVERITY.get(severity, (_ACTIVE.text, _ACTIVE.panel_alt, _ACTIVE.border))


def ink(severity: str) -> str:
    """The text colour for a severity name.

    ``muted`` is accepted alongside the four severities because a state table
    has a "nothing is happening" entry as well as the four that mean something,
    and that entry wants the ordinary secondary ink rather than a colour.
    """
    if severity == "muted":
        return _ACTIVE.text_muted
    return severity_colours(severity)[0]


def _bind(palette: Palette) -> None:
    """Publish ``palette`` as the module-level constants.

    Rebinding the globals rather than looking the tokens up through a function
    is what lets every existing call site — and every ``theme.ERROR`` in a test —
    keep working while the theme underneath it moves.
    """
    global _ACTIVE
    _ACTIVE = palette
    namespace = globals()
    for field, value in palette.__dict__.items():
        if field in ("name", "label"):
            # Descriptive, and already reachable as current_theme()/label();
            # exporting them as NAME and LABEL would only add two more names
            # that mean nothing a caller would guess.
            continue
        namespace[field.upper()] = value
    # The aliases the rest of the console reads: "disabled" text under the name
    # the widgets use, and the green that means "measured" under the name the
    # success states use.  ``OK is ACTUAL`` is a promise the tests hold us to.
    namespace["DISABLED"] = palette.text_disabled
    namespace["OK"] = palette.actual
    namespace["OK_SOFT"] = palette.actual_soft
    namespace["OK_LINE"] = palette.actual_line
    namespace["SEVERITY"] = _severity_table(palette)


_bind(_ACTIVE)

# ── the switch ──────────────────────────────────────────────────────────────
#: Widgets that have already painted the old palette.  Weak, because a subscriber
#: outliving its widget would be called on a deleted C++ object the next time the
#: theme changed — and because a test suite builds a great many windows.
_subscribers: list[Any] = []


def subscribe(callback: Callable[[], None]) -> Callable[[], None]:
    """Call ``callback`` after every theme change; the result unsubscribes.

    Pass a **bound method** of the widget that has to repaint — that is held
    weakly, so the widget going away unsubscribes it.  A plain function is held
    for as long as the process runs, which is what a module-level hook wants and
    is a leak for anything else.
    """
    ref: Any = (
        weakref.WeakMethod(callback) if inspect.ismethod(callback)
        else callback
    )
    _subscribers.append(ref)

    def unsubscribe() -> None:
        try:
            _subscribers.remove(ref)
        except ValueError:
            pass

    return unsubscribe


def _notify() -> None:
    """Tell every live subscriber, dropping the ones whose widget has gone."""
    alive: list[Any] = []
    for ref in _subscribers:
        callback = ref() if isinstance(ref, weakref.WeakMethod) else ref
        if callback is None:
            continue
        try:
            callback()
        except RuntimeError:
            # The Qt object behind the bound method has been deleted; the
            # weakref has not noticed yet because the Python wrapper is alive.
            continue
        alive.append(ref)
    _subscribers[:] = alive


def current_theme() -> str:
    """The name of the palette in force."""
    return _ACTIVE.name


def label() -> str:
    """The palette's name in the operator's language, for a button."""
    return _ACTIVE.label


def resolve(name: str | None) -> str:
    """A stored preference turned into a theme name.

    ``None`` or anything unrecognised means the operator has not chosen, and the
    console opens in :data:`DEFAULT_THEME` — the same theme it opened in before
    there was a choice, so a preference file that was never written and one that
    was hand-edited into nonsense behave identically.
    """
    return name if name in PALETTES else DEFAULT_THEME


def set_theme(name: str, *, app: Any = None) -> str:
    """Switch the tokens to ``name``, and tell the repainters.

    An unknown name is ignored rather than raising: the theme is a preference,
    and a preferences file edited by hand must not be able to stop the console
    from opening.  The name actually in force is returned either way.

    Pass ``app`` — a ``QApplication`` — to push the palette and the stylesheet
    to it as well, which is what a caller changing the theme of a running
    console wants.  It is opt-in rather than looked up, for two reasons: pushing
    the stylesheet re-polishes every widget in the process and costs tens of
    milliseconds, which is not a price to pay for moving the tokens in a test;
    and doing it *here*, before the subscribers are told, is what keeps a widget
    that repaints itself from repainting against the previous chrome.
    """
    palette = PALETTES.get(name)
    if palette is None or palette.name == _ACTIVE.name:
        return _ACTIVE.name
    _bind(palette)
    if app is not None:
        apply(app)
    _notify()
    return _ACTIVE.name


def toggle_theme(*, app: Any = None) -> str:
    """Switch to the other theme, and say which one that was."""
    return set_theme(LIGHT if _ACTIVE.name == DARK else DARK, app=app)


# ── the stylesheet ──────────────────────────────────────────────────────────
def stylesheet() -> str:
    """The application-wide stylesheet.

    Structured the way litearm-studio's CSS is: surfaces first, then the line
    and ink ramps, then the components.  The one Qt-specific decision worth
    naming is that **no rule paints a bare ``QWidget``**.  A rule on ``QWidget``
    cascades to every label and every plain container, which is how the console
    previously ended up with an inset rectangle behind each of its labels; here
    the page colour comes from the application palette and only the widgets that
    are actually a surface are given one.
    """
    p = _ACTIVE
    return f"""
    /* ── type ──────────────────────────────────────────────────────────── */
    QWidget {{
        font-family: {UI_FAMILY};
        font-size: {FONT_PX}px;
        color: {p.text};
    }}
    QMainWindow, QDialog {{ background: {p.background}; }}

    /* Label roles.  A page names what a label *is* once — a field name, a
       monospaced figure — and the colour and the face come from here, so a
       label does not have to be re-written when the palette moves. */
    QLabel[role="field"] {{ color: {p.text_muted}; }}
    QLabel[role="mono"] {{ font-family: {MONO_FAMILY}; }}
    QLabel[role="readout"] {{
        font-family: {MONO_FAMILY};
        font-size: {FONT_PX}px;
        font-weight: 600;
        color: {p.text};
    }}
    QLabel[role="caption"] {{ color: {p.text_muted}; font-size: {FONT_SMALL_PX}px; }}
    QLabel[role="title"] {{ color: {p.text}; font-weight: 600; }}
    /* The E-stop fills a card taller than a normal button, and a 13px label in
       the middle of a slab that size reads as an oversight. */
    QPushButton[role="estop"] {{ font-size: {FONT_PX + 3}px; letter-spacing: 1px; }}

    /* ── cards ─────────────────────────────────────────────────────────── */
    QGroupBox {{
        background: {p.panel};
        border: 1px solid {p.border};
        border-radius: {RADIUS_CARD}px;
        margin-top: 0px;
        padding: 30px 12px 12px 12px;
        font-weight: 600;
    }}
    QGroupBox::title {{
        subcontrol-origin: padding;
        subcontrol-position: top left;
        left: 12px;
        top: 8px;
        padding: 0;
        color: {p.text_muted};
        font-size: {FONT_SMALL_PX + 1}px;
        font-weight: 600;
    }}
    QGroupBox::indicator {{ width: 0px; height: 0px; }}

    /* ── buttons ───────────────────────────────────────────────────────── */
    QPushButton {{
        background: {p.panel_alt};
        border: 1px solid {p.border};
        border-radius: {RADIUS}px;
        padding: 5px 14px;
        min-height: 18px;
        color: {p.text_strong};
        font-weight: 500;
    }}
    QPushButton:hover:enabled {{
        background: {p.elevated};
        border-color: {p.border_strong};
        color: {p.text};
    }}
    QPushButton:pressed:enabled {{
        background: {p.panel_alt};
        color: {p.text};
    }}
    QPushButton:focus {{ outline: none; border-color: {p.target}; }}
    QPushButton:disabled {{
        color: {p.text_disabled};
        background: {p.panel};
        border-color: {p.border_soft};
    }}
    /* The two emphasised buttons.  A "primary" is the console's ink on a
       surface — litearm's primary button is the near-black (light) or
       near-white (dark) block, not a saturated accent — and a "danger" is the
       one solid red left in the interface, kept solid because the two controls
       that carry it both stop a motor. */
    QPushButton[accent="true"], QPushButton[variant="primary"] {{
        background: {p.text};
        color: {p.background};
        border: 1px solid {p.text};
        font-weight: 600;
    }}
    QPushButton[accent="true"]:hover:enabled,
    QPushButton[variant="primary"]:hover:enabled {{
        background: {p.text_strong};
        border-color: {p.text_strong};
    }}
    QPushButton[accent="true"]:disabled,
    QPushButton[variant="primary"]:disabled {{
        background: {p.panel_alt};
        border-color: {p.border_soft};
        color: {p.text_disabled};
    }}
    QPushButton[danger="true"], QPushButton[variant="destructive"] {{
        background: {p.danger_solid};
        color: {p.danger_solid_fg};
        border: 1px solid {p.danger_solid};
        font-weight: 600;
    }}
    /* Hovering brightens the edge rather than darkening the fill: a filled red
       that goes darker under the cursor makes the one control that must never
       be missed look like it has gone away. */
    QPushButton[danger="true"]:hover:enabled,
    QPushButton[variant="destructive"]:hover:enabled {{
        background: {p.danger_solid};
        border-color: {p.danger_solid_fg};
    }}
    QPushButton[danger="true"]:pressed:enabled,
    QPushButton[variant="destructive"]:pressed:enabled {{
        background: {p.error_line};
        border-color: {p.error_line};
        color: {p.danger_solid_fg};
    }}
    QPushButton[danger="true"]:disabled,
    QPushButton[variant="destructive"]:disabled {{
        background: {p.panel_alt};
        border-color: {p.border_soft};
        color: {p.text_disabled};
    }}
    /* ── the segmented control, in the status line ─────────────────────── */
    /* The same shape the tab strip uses: a muted track with the active item
       floating on it.  Used by the theme switch, whose two names have to be
       visible at once — a single toggle labelled with the state it is in
       cannot say whether it describes now or what a click will do. */
    #Segmented {{
        background: {p.panel_alt};
        border: 1px solid {p.border};
        border-radius: {RADIUS - 1}px;
    }}
    QPushButton[segment="true"] {{
        background: transparent;
        border: none;
        border-radius: {RADIUS - 3}px;
        min-height: 12px;
        padding: 2px 10px;
        color: {p.text_muted};
        font-weight: 500;
    }}
    QPushButton[segment="true"]:hover:enabled {{
        background: {p.hover};
        color: {p.text};
    }}
    QPushButton[segment="true"]:pressed:enabled {{
        background: transparent;
    }}
    QPushButton[segment="true"]:checked {{
        background: {p.elevated};
        color: {p.text};
        font-weight: 600;
    }}
    QPushButton[variant="ghost"] {{
        background: transparent;
        border-color: transparent;
        color: {p.text_muted};
    }}
    QPushButton[variant="ghost"]:hover:enabled {{
        background: {p.hover};
        color: {p.text};
    }}

    /* ── inputs ────────────────────────────────────────────────────────── */
    QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QAbstractSpinBox {{
        background: {p.panel_alt};
        border: 1px solid {p.border};
        border-radius: {RADIUS - 1}px;
        padding: 4px 8px;
        min-height: 18px;
        color: {p.text};
        selection-background-color: {p.target};
        selection-color: {p.background};
    }}
    QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
        border-color: {p.target};
    }}
    QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled,
    QComboBox:disabled {{
        color: {p.text_disabled};
        background: {p.panel};
    }}
    QComboBox::drop-down {{ border: none; width: 18px; }}
    QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{
        background: transparent;
        border: none;
        width: 16px;
    }}
    QAbstractSpinBox::up-arrow {{ width: 7px; height: 7px; }}
    QAbstractSpinBox::down-arrow {{ width: 7px; height: 7px; }}

    /* ── checkboxes ────────────────────────────────────────────────────── */
    QCheckBox {{ spacing: 8px; color: {p.text_muted}; background: transparent; }}
    QCheckBox:hover {{ color: {p.text}; }}
    QCheckBox::indicator {{
        width: 15px;
        height: 15px;
        border: 1px solid {p.border_strong};
        border-radius: 4px;
        background: {p.panel_alt};
    }}
    QCheckBox::indicator:hover {{ border-color: {p.text_faint}; }}
    QCheckBox::indicator:checked {{
        background: {p.target};
        border-color: {p.target};
    }}
    QCheckBox::indicator:disabled {{
        border-color: {p.border_soft};
        background: {p.panel};
    }}

    /* ── the slider ────────────────────────────────────────────────────── */
    QSlider::groove:horizontal {{
        height: 6px;
        background: {p.groove};
        border-radius: 3px;
    }}
    QSlider::sub-page:horizontal {{
        background: {p.target};
        border-radius: 3px;
    }}
    QSlider::handle:horizontal {{
        width: 14px;
        height: 14px;
        margin: -5px 0;
        border-radius: 7px;
        background: {p.text};
        border: 2px solid {p.panel};
    }}
    QSlider::handle:horizontal:hover {{ background: {p.text_strong}; }}
    QSlider::handle:horizontal:disabled {{ background: {p.text_disabled}; }}
    QSlider::sub-page:horizontal:disabled {{ background: {p.border_strong}; }}

    /* ── tabs, as litearm's segmented control ──────────────────────────── */
    QTabWidget::pane {{
        border: none;
        background: transparent;
        top: 0px;
    }}
    QTabBar {{ background: transparent; }}
    QTabBar::tab {{
        background: transparent;
        border: none;
        border-radius: {RADIUS - 1}px;
        padding: 7px 18px;
        margin: 2px 4px 2px 0;
        color: {p.text_muted};
        font-weight: 500;
    }}
    QTabBar::tab:hover:!selected {{ background: {p.hover}; color: {p.text}; }}
    QTabBar::tab:selected {{
        background: {p.elevated};
        color: {p.text};
        font-weight: 600;
    }}
    QTabBar::tab:disabled {{ color: {p.text_disabled}; }}

    /* ── tables ────────────────────────────────────────────────────────── */
    QTableWidget, QTableView {{
        background: {p.panel};
        alternate-background-color: {p.panel_alt};
        gridline-color: {p.border};
        border: 1px solid {p.border};
        border-radius: {RADIUS}px;
    }}
    QHeaderView::section {{
        background: {p.panel_alt};
        border: none;
        border-right: 1px solid {p.border};
        border-bottom: 1px solid {p.border};
        padding: 6px 8px;
        color: {p.text_muted};
        font-weight: 600;
    }}

    /* ── the log, a console set into whatever holds it ─────────────────── */
    QPlainTextEdit, QTextEdit {{
        background: {p.background};
        border: 1px solid {p.border};
        border-radius: {RADIUS}px;
        padding: 6px;
        font-family: {MONO_FAMILY};
        font-size: {FONT_SMALL_PX}px;
        color: {p.text_soft};
        selection-background-color: {p.target};
        selection-color: {p.background};
    }}

    /* ── the progress bar ──────────────────────────────────────────────── */
    QProgressBar {{
        background: {p.groove};
        border: none;
        border-radius: 6px;
        height: 12px;
        text-align: center;
        color: {p.text_muted};
        font-size: {FONT_SMALL_PX}px;
        font-weight: 600;
    }}
    QProgressBar::chunk {{
        background: {p.target};
        border-radius: 6px;
    }}

    /* ── scrolling ─────────────────────────────────────────────────────── */
    QScrollArea {{ background: transparent; border: none; }}
    QScrollArea > QWidget > QWidget {{ background: transparent; }}
    QAbstractScrollArea::corner {{ background: transparent; }}
    QScrollBar:vertical {{
        background: transparent; width: 10px; margin: 0px;
    }}
    QScrollBar::handle:vertical {{
        background: {p.border_strong}; border-radius: 5px; min-height: 28px;
    }}
    QScrollBar::handle:vertical:hover {{ background: {p.text_faint}; }}
    QScrollBar:horizontal {{
        background: transparent; height: 10px; margin: 0px;
    }}
    QScrollBar::handle:horizontal {{
        background: {p.border_strong}; border-radius: 5px; min-width: 28px;
    }}
    QScrollBar::handle:horizontal:hover {{ background: {p.text_faint}; }}
    QScrollBar::add-line, QScrollBar::sub-line,
    QScrollBar::add-page, QScrollBar::sub-page {{
        background: transparent; height: 0px; width: 0px; border: none;
    }}

    /* ── the window's own furniture ────────────────────────────────────── */
    QDockWidget {{
        color: {p.text_muted};
        titlebar-close-icon: none;
        titlebar-normal-icon: none;
    }}
    /* The log dock draws its own title bar, so the card is these two rules
       rather than a ``QDockWidget::title``: the header carries the top half of
       the border and the body the bottom half, and between them they are the
       same panel the group boxes are.  Removing the style's own title rule
       without replacing it is what left the dock as the one flat region in the
       window — a label and a console sitting on the page colour. */
    #LogTitle {{
        background: {p.panel};
        border: 1px solid {p.border};
        border-bottom: none;
        border-top-left-radius: {RADIUS_CARD}px;
        border-top-right-radius: {RADIUS_CARD}px;
    }}
    #LogBody {{
        background: {p.panel};
        border: 1px solid {p.border};
        border-top: none;
        border-bottom-left-radius: {RADIUS_CARD}px;
        border-bottom-right-radius: {RADIUS_CARD}px;
    }}
    /* The bottom row needs no surface, and must not be given one: a strip down
       here is cut off at both ends by the window's gutter, which is what made
       the footer look clipped when it had one.  The status bar is kept only
       because it is the one thing QMainWindow puts below the dock area, so it
       is hollowed out to nothing but a place to hold the row. */
    QStatusBar {{
        background: transparent;
        border: none;
    }}
    QStatusBar::item {{ border: none; }}
    #StateLine {{ background: transparent; border: none; }}
    QToolTip {{
        background: {p.elevated};
        color: {p.text};
        border: 1px solid {p.border};
        border-radius: 6px;
        padding: 6px 8px;
    }}

    /* ── menus, for the file chooser Qt draws itself ───────────────────── */
    QMenu {{
        background: {p.elevated};
        border: 1px solid {p.border};
        border-radius: {RADIUS}px;
        padding: 4px;
    }}
    QMenu::item {{ padding: 6px 14px; border-radius: 6px; }}
    QMenu::item:selected {{ background: {p.hover}; }}
    QMenu::separator {{ height: 1px; background: {p.border}; margin: 4px 8px; }}
    QToolBar {{ background: {p.panel}; border: none; }}
    """


def qt_palette():
    """The application palette, so unstyled Qt chrome matches the theme.

    A stylesheet reaches the widgets this console writes; the palette is what
    reaches the ones it does not — the file chooser's list, a message box's
    buttons, the window background behind an empty tab.
    """
    from PyQt5.QtGui import QColor, QPalette

    p = _ACTIVE
    palette = QPalette()
    roles = {
        QPalette.Window: p.background,
        QPalette.WindowText: p.text,
        QPalette.Base: p.panel,
        QPalette.AlternateBase: p.panel_alt,
        QPalette.Text: p.text,
        QPalette.Button: p.panel_alt,
        QPalette.ButtonText: p.text,
        QPalette.BrightText: p.error,
        QPalette.Highlight: p.target,
        QPalette.HighlightedText: p.background,
        QPalette.ToolTipBase: p.elevated,
        QPalette.ToolTipText: p.text,
        QPalette.Link: p.target,
        QPalette.Light: p.border_strong,
        QPalette.Midlight: p.border,
        QPalette.Mid: p.border,
        QPalette.Dark: p.border_soft,
        QPalette.Shadow: p.background,
    }
    # Qt 5.12 and up; skipped rather than raising on an older Qt, which is not a
    # reason for the console to fail to start.
    placeholder = getattr(QPalette, "PlaceholderText", None)
    if placeholder is not None:
        roles[placeholder] = p.text_faint
    for role, colour in roles.items():
        palette.setColor(role, QColor(colour))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        palette.setColor(QPalette.Disabled, role, QColor(p.text_disabled))
    return palette


def apply(app: Any) -> None:
    """Push the palette and the stylesheet to ``app``.

    Separate from :func:`set_theme` because the two answer different questions:
    which theme is in force, and which application has been told about it.  A
    console starting up needs both — the tokens before the first widget is
    built, and the push once there is an application to push to.
    """
    app.setPalette(qt_palette())
    app.setStyleSheet(stylesheet())


def level_colour(level: str) -> str:
    """The colour for an alert or log level.

    Unknown levels are muted rather than an error colour: something arriving
    that this module has not been taught about is not the operator's emergency.
    """
    return {
        "info": TEXT_MUTED,
        "debug": TEXT_DISABLED,
        "warn": WARN,
        "warning": WARN,
        "error": ERROR,
        "fatal": ERROR,
    }.get(level, TEXT_MUTED)
