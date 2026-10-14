# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Skipper Flipper look for the tkinter GUI: a 1928 black-and-white cartoon feel.

Only the visuals (grainy paper, heavy ink outlines, film-strip perforations,
inverted-flash buttons, title-card banner) are borrowed; no characters.
A Benchy-themed pass is planned for when the firmware work starts.

Pure standard library. If tools/assets/art.png is missing the GUI falls back
to a text title, so nothing here is required to run the tools.
"""
import os
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

PAPER = "#e9e6dc"       # aged film-paper white
PAPER_DARK = "#d3cfc2"
LIGHT = "#f7f5ee"
INK = "#141414"
GREY = "#6b6b6b"
HIGHLIGHT = "#cfcabb"

# First one installed wins; all optional. DejaVu Sans Bold is the safe fallback.
DISPLAY_FONTS = ("Chewy", "Lucky Guy", "Bangers", "Fredoka One", "Comic Neue",
                 "Comic Sans MS", "DejaVu Sans")
MONO_FONTS = ("DejaVu Sans Mono", "Liberation Mono", "Courier New", "Courier")

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def _first_installed(root, wanted, default):
    have = set(tkfont.families(root))
    for name in wanted:
        if name in have:
            return name
    return default


def apply(root):
    """Style root + ttk. Returns a dict with the fonts the GUI should use."""
    st = ttk.Style(root)
    try:
        st.theme_use("clam")
    except tk.TclError:
        pass
    display = _first_installed(root, DISPLAY_FONTS, "Helvetica")
    mono = _first_installed(root, MONO_FONTS, "Courier")
    fonts = {
        "display": (display, 11, "bold"),
        "title": (display, 20, "bold"),
        "small": (display, 9, "bold"),
        "mono": (mono, 10),
        "mono_bold": (mono, 10, "bold"),
        "mono_italic": (mono, 10, "italic"),
    }
    root.configure(bg=PAPER)
    root.option_add("*TCombobox*Listbox.background", LIGHT)
    root.option_add("*TCombobox*Listbox.foreground", INK)
    root.option_add("*TCombobox*Listbox.selectBackground", INK)
    root.option_add("*TCombobox*Listbox.selectForeground", PAPER)

    st.configure(".", background=PAPER, foreground=INK, fieldbackground=LIGHT,
                 bordercolor=INK, lightcolor=PAPER, darkcolor=PAPER_DARK,
                 troughcolor=PAPER_DARK, focuscolor=INK, insertcolor=INK,
                 selectbackground=INK, selectforeground=PAPER)
    st.configure("TButton", padding=(10, 4), background=LIGHT, borderwidth=2,
                 relief="raised", font=fonts["display"])
    st.configure("Small.TButton", padding=(6, 1), font=fonts["small"])
    st.map("TButton",
           background=[("pressed", INK), ("active", INK), ("disabled", PAPER_DARK)],
           foreground=[("pressed", PAPER), ("active", PAPER), ("disabled", GREY)])
    st.configure("TLabelframe", background=PAPER, bordercolor=INK, borderwidth=2)
    st.configure("TLabelframe.Label", background=PAPER, foreground=INK, font=fonts["display"])
    st.configure("TCheckbutton", background=PAPER, indicatorbackground=LIGHT,
                 indicatorforeground=INK)
    st.map("TCheckbutton", background=[("active", PAPER)])
    for w in ("TCombobox", "TEntry", "TSpinbox"):
        st.configure(w, fieldbackground=LIGHT, background=LIGHT, bordercolor=INK,
                     arrowcolor=INK, padding=3)
    st.map("TCombobox", fieldbackground=[("readonly", LIGHT)])
    st.configure("TScrollbar", background=LIGHT, troughcolor=PAPER_DARK, bordercolor=INK,
                 arrowcolor=INK)
    st.map("TScrollbar", background=[("active", GREY)])
    st.configure("Status.TLabel", background=PAPER, foreground=INK)
    st.configure("Version.TLabel", background=INK, foreground=PAPER, padding=(8, 2),
                 font=fonts["mono_bold"])
    st.configure("Muted.TLabel", background=PAPER, foreground=GREY)
    # reversed-film badge for warnings such as "compile-only / unverified"
    st.configure("Badge.TLabel", background=INK, foreground=PAPER, padding=(4, 2),
                 font=fonts["mono_bold"])
    st.configure("TNotebook", background=PAPER, bordercolor=INK, borderwidth=2, tabmargins=(4, 4, 4, 0))
    st.configure("TNotebook.Tab", background=PAPER_DARK, foreground=INK, padding=(16, 4),
                 bordercolor=INK, font=fonts["display"])
    st.map("TNotebook.Tab", background=[("selected", LIGHT), ("active", HIGHLIGHT)],
           foreground=[("selected", INK)])
    return fonts


def style_text(text, fonts):
    """Output box: ink on paper; tools are told apart by weight, not colour."""
    text.configure(bg=LIGHT, fg=INK, insertbackground=INK, selectbackground=INK,
                   selectforeground=PAPER, relief="flat", borderwidth=0,
                   highlightthickness=2, highlightbackground=INK, highlightcolor=INK,
                   font=fonts["mono"], padx=6, pady=4)
    text.tag_configure("ts", foreground="#8a8a8a")
    text.tag_configure("app", foreground=GREY)
    text.tag_configure("build", foreground=INK)
    text.tag_configure("fw", foreground=INK, background="#e2dfd3")
    text.tag_configure("probe", foreground=INK, font=fonts["mono_bold"])
    text.tag_configure("fake", foreground="#3c3c3c", font=fonts["mono_italic"])
    text.tag_configure("hint", foreground=INK, background=HIGHLIGHT, font=fonts["mono_bold"])
    # errors flash like a reversed film frame
    text.tag_configure("err", foreground=PAPER, background=INK, font=fonts["mono_bold"])
    text.tag_raise("err")
    text.tag_raise("sel")


def load_banner(root, name="art.png"):
    """Title-card image, or None (caller shows a text title instead)."""
    path = os.path.join(ASSETS, name)
    try:
        return tk.PhotoImage(master=root, file=path)
    except (tk.TclError, OSError):
        return None


class FilmStrip(tk.Canvas):
    """A strip of sprocket holes, like the edge of a reel."""

    def __init__(self, master, height=18):
        super().__init__(master, height=height, bg=INK, highlightthickness=0, bd=0)
        self.bind("<Configure>", self._draw)

    def _draw(self, _event=None):
        self.delete("all")
        h = int(self["height"])
        hole_w, hole_h, step = 12, h - 8, 24
        x = 8
        while x < self.winfo_width():
            self.create_rectangle(x, 4, x + hole_w, 4 + hole_h, fill=PAPER, outline=PAPER)
            x += step


def make_header(root, fonts, heading, lines, badge=None):
    """Header band: the whole title artwork (dolphin skipper, paddle boat, Benchys) on the
    left, heading + WIP badge + short text on the right. Falls back to text only.
    Returns the frame; keep a reference to it (it holds the image)."""
    fr = tk.Frame(root, bg=PAPER)
    fr.art = load_banner(root)
    if fr.art:
        tk.Label(fr, image=fr.art, bg=INK, bd=0, highlightthickness=2,
                 highlightbackground=INK).pack(side="left", padx=(8, 10), pady=4)
    col = tk.Frame(fr, bg=PAPER)
    col.pack(side="left", fill="both", expand=True, padx=(0, 8), pady=4)
    tk.Label(col, text=heading, font=fonts["title"], bg=PAPER, fg=INK, anchor="w").pack(fill="x")
    if badge:
        ttk.Label(col, text=" " + badge + " ", style="Badge.TLabel", wraplength=560,
                  justify="left").pack(anchor="w", pady=(4, 6))
    tk.Label(col, text="\n".join(lines), font=fonts["small"], bg=PAPER, fg=INK, anchor="nw",
             justify="left", wraplength=560).pack(fill="x")
    return fr


def set_icon(root):
    """Window/taskbar icon from the repo's icons/ folder (the black-and-white SF chip). Optional."""
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        "icons", "icon-64.png")
    try:
        root._sf_icon = tk.PhotoImage(master=root, file=path)
        root.iconphoto(True, root._sf_icon)
    except (tk.TclError, OSError):
        pass
