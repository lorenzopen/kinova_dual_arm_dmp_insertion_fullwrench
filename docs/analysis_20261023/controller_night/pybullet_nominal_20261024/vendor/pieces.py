"""The seven tetrominoes as CELL SETS, and their rotations.

Same literals as `generate_tetris_obj.py` (which builds the bodies), kept here so
the placement search needs no PyBullet: if one list changes, change both.
Cells are (column, row), row 0 at the bottom - the shelf's convention too.
"""
import numpy as np

# name: cells, and the arcade colour (only so pieces are tellable apart)
PIECES = {
    "I": ([(0, 0), (1, 0), (2, 0), (3, 0)], (0.00, 0.85, 0.90, 1.0)),
    "O": ([(0, 0), (1, 0), (0, 1), (1, 1)], (0.90, 0.85, 0.00, 1.0)),
    "T": ([(0, 0), (1, 0), (2, 0), (1, 1)], (0.65, 0.00, 0.85, 1.0)),
    "S": ([(0, 0), (1, 0), (1, 1), (2, 1)], (0.00, 0.75, 0.20, 1.0)),
    "Z": ([(1, 0), (2, 0), (0, 1), (1, 1)], (0.90, 0.10, 0.10, 1.0)),
    "J": ([(0, 0), (1, 0), (2, 0), (0, 1)], (0.10, 0.20, 0.90, 1.0)),
    "L": ([(0, 0), (1, 0), (2, 0), (2, 1)], (0.95, 0.50, 0.00, 1.0)),
}


def normalise(cells):
    """Shift a cell set so its lowest column and row are 0, and sort it."""
    c = np.asarray(sorted(cells), int)
    return tuple(map(tuple, c - c.min(axis=0)))


def rotations(name):
    """The DISTINCT rotations of a piece, in quarter turns from 0.

    Returns [(quarter_turns, cells), ...]: 1 for O, 2 for I/S/Z, 4 for T/J/L.
    Duplicates are dropped, so the search never evaluates the same shape twice.
    """
    cells = PIECES[name][0]
    out, seen = [], set()
    for k in range(4):
        n = normalise(cells)
        if n not in seen:
            seen.add(n)
            out.append((k, n))
        cells = [(-r, c) for c, r in cells]        # quarter turn, counter-clockwise
    return out


def width_height(cells):
    c = np.asarray(cells, int)
    return int(c[:, 0].max()) + 1, int(c[:, 1].max()) + 1


def turn(cells, k):
    """The cell set after `k` quarter turns counter-clockwise, NOT normalised.
    It rotates, never mirrors, so an S never comes out as a Z."""
    out = [tuple(map(int, c)) for c in cells]
    for _ in range(k % 4):
        out = [(-r, c) for c, r in out]
    return out


def bag(rng):
    """The 7-bag, how Tetris deals (Guideline, 2001): shuffle all seven, deal
    them, refill. Unlike i.i.d. draws it bounds the gap between two of a kind at
    12, and every seven moves exercise each piece exactly once."""
    names = list(PIECES)
    while True:
        for n in rng.permutation(names):
            yield str(n)


# The Guideline colours (since 2001). Only visual: a screenshot reads at a glance.
COLOURS = dict(I=(0.00, 0.94, 0.94, 1.0),      # cyan
               O=(0.94, 0.94, 0.00, 1.0),      # yellow
               T=(0.63, 0.00, 0.94, 1.0),      # purple
               S=(0.00, 0.94, 0.00, 1.0),      # green
               Z=(0.94, 0.00, 0.00, 1.0),      # red
               J=(0.00, 0.00, 0.94, 1.0),      # blue
               L=(0.94, 0.63, 0.00, 1.0))      # orange
