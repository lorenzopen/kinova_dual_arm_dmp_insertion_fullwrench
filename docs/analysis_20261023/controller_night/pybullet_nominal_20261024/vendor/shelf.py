"""THE SHELF: the Tetris grid STANDING UP, in the world XZ plane.

Columns along +x, rows up +z, depth along +y; a piece is lowered into its slot
from above. Vertical because of the hands: one pad on the front face, one on the
back, so they never meet a neighbour or a side wall - how a book is shelved. No
back and no front panel: the pads live there. The price: the moment is about
the PAD AXIS, held by the tri-pad's torsion alone (CLAUDE.md §8).
"""
import os

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rot

from pieces import PIECES, normalise, turn

WALL = 0.020                    # [m] shelf panel thickness
# Friction is the EFFECTIVE coefficient of a pair; PyBullet multiplies the two
# bodies' values, so the per-body one is derived (as `handling.MU`).
MU_PIECE_PIECE = 0.50           # [-] a piece against another piece, and a panel
MU_PIECE_FLOOR = 0.30           # [-] a piece against the shelf floor
MU = float(np.sqrt(MU_PIECE_PIECE))          # per-body, a piece
# [m] DECLARED air between neighbouring pieces: with none, any drift overlaps
# them (0.68 mm of penetration at spawn threw a piece 430 mm).
SEAM = 0.002
# [m] air to a SIDE WALL, NOT the seam: a neighbour yields, a panel never does, so
# it must cover the chain's error (4.5 mm median, 12 mm p90); tied to the seam, 2 of
# 22 seated. 20 mm since 2026-09-10, unmeasured against 10 (CLAUDE.md §8a).
WALL_GAP = 0.020
# [m] how far sideways a SEATED piece stands off its cell: median of 102 placements
# (mean 7.0, p90 12.2 mm). The next slot is `SEAM / 2 - PLACED_OFF` per side,
# negative: the insertion has to make room.
PLACED_OFF = 0.0045
# [m] radius of the ROUNDED top inner edge of each side wall: a lead-in for the
# ~1.3 deg the inversion never measures (2.7 mm per side on a 240 mm piece); a
# square edge hooked a J and levered it to 134 mm and 90 deg.
LEAD_IN = 0.020
# [m] 45 deg chamfer on the in-plane corners of each column (pad faces untouched):
# a square corner lands face-on-face and stops dead (1339 N vertical, 0.1 N
# sideways, `33_wedge_probe.py`); Yang Lin's model already cuts 8.0 mm. Default
# since 2026-09-03; 0 (square corners) is the old arm.
CHAMFER = 0.008
MESHES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")


def prism(hx, hy, hz, c):
    """An OBJ for one column: a box `2hx x 2hy x 2hz`, its four xy corners cut
    at 45 deg by `c` (xy = the shelf plane in the piece frame, so the pad faces
    stay flat). Returns the file path, written once."""
    os.makedirs(MESHES, exist_ok=True)
    path = os.path.join(MESHES, "col_%d_%d_%d_%d.obj"
                        % tuple(round(1e5 * v) for v in (hx, hy, hz, c)))
    if os.path.exists(path):
        return path
    prof = [(hx, hy - c), (hx - c, hy), (-(hx - c), hy), (-hx, hy - c),
            (-hx, -(hy - c)), (-(hx - c), -hy), (hx - c, -hy), (hx, -(hy - c))]
    n = len(prof)
    with open(path, "w") as f:
        out = []
        for z in (-hz, hz):
            out += ["v %.6f %.6f %.6f" % (x, y, z) for x, y in prof]
        for i in range(n):                       # the sides
            j = (i + 1) % n
            out.append("f %d %d %d %d" % (i + 1, j + 1, j + 1 + n, i + 1 + n))
        out.append("f " + " ".join(str(k) for k in range(n, 0, -1)))
        out.append("f " + " ".join(str(k) for k in range(n + 1, 2 * n + 1)))
        f.write(chr(10).join(out) + chr(10))
    return path


def columns(parts):
    """[(half, origin)] one box per COLUMN of cells, so the long flanks the
    insertion slides along have no chamfer notch. The I is ONE box (2026-09-24);
    other pieces in odd rotations still slide on a notched flank (open).
    """
    ys = {round(1e5 * float(q["origin"][1])) for q in parts}
    if len(ys) == 1 and len(parts) > 1:            # one straight row: the I
        lo = min(float(q["origin"][0]) - float(q["half"][0]) for q in parts)
        hi = max(float(q["origin"][0]) + float(q["half"][0]) for q in parts)
        h0, o0 = (np.asarray(parts[0][k], float) for k in ("half", "origin"))
        return [(np.array([(hi - lo) / 2, h0[1], h0[2]]),
                 np.array([(hi + lo) / 2, o0[1], o0[2]]))]
    cols = {}
    for q in parts:
        h, o = np.asarray(q["half"], float), np.asarray(q["origin"], float)
        cols.setdefault(round(1e5 * o[0]), []).append((h, o))
    out = []
    for _x, run in sorted(cols.items()):
        lo = min(o[1] - h[1] for h, o in run)
        hi = max(o[1] + h[1] for h, o in run)
        h0, o0 = run[0]
        out.append((np.array([h0[0], (hi - lo) / 2, h0[2]]),
                    np.array([o0[0], (hi + lo) / 2, o0[2]])))
    return out


def slab(half, pos, colour):
    """One static panel."""
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=colour)
    b = p.createMultiBody(0.0, col, vis, pos)
    p.changeDynamics(b, -1, lateralFriction=MU)
    return b


def inner_faces(cols, pitch):
    """[m] x of the two side walls' inner faces: WALL_GAP beyond the outermost
    piece faces, which sit half a seam inside the grid lines."""
    return -WALL_GAP + SEAM / 2, cols * pitch + WALL_GAP - SEAM / 2


def fillet(x_corner, z_top, depth, inward, radius=LEAD_IN):
    """The rounded top edge of one side wall: a cylinder along the depth, a
    radius inside the wall and below its top, so tangent to both."""
    c = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=depth)
    v = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=depth,
                            rgbaColor=[0.55, 0.57, 0.62, 1.0])
    q = Rot.from_euler("x", 90, degrees=True).as_quat()
    b = p.createMultiBody(0.0, c, v,
                          [x_corner - inward * radius, 0.0, z_top - radius],
                          q.tolist())
    p.changeDynamics(b, -1, lateralFriction=MU)
    return b


# [-] per-body floor friction. A smooth shelf is HOW a neighbour gets out of the
# way: pushed, it slides and stays (at the pads' 0.7 a 1 kg piece held rigid to
# 6.9 N, then slid free: `13_neighbour_stiffness.py`).
FLOOR_MU = MU_PIECE_FLOOR / MU                # per-body, the floor
def build(rows, cols, pitch, depth):
    """Floor and two fixed side walls with a rounded lead-in on top; no back and
    no front (the pads go there). Returns the shelf's body ids."""
    h = rows * (pitch - SEAM)      # rows are pitched by the CELL, see `pose`
    x0, x1 = inner_faces(cols, pitch)
    grey = [0.45, 0.47, 0.52, 1.0]
    floor = slab([(x1 - x0) / 2 + WALL, depth / 2, WALL / 2],
                 [(x0 + x1) / 2, 0.0, -WALL / 2], [0.30, 0.32, 0.36, 1.0])
    p.changeDynamics(floor, -1, lateralFriction=FLOOR_MU)
    # the panel stops a radius short and the fillet finishes it: no square corner
    flat = h - LEAD_IN
    # ids returned, so a caller can tell the shelf from the parked hands
    ids = (floor,
           slab([WALL / 2, depth / 2, flat / 2], [x0 - WALL / 2, 0.0, flat / 2], grey),
           slab([WALL / 2, depth / 2, flat / 2], [x1 + WALL / 2, 0.0, flat / 2], grey),
           fillet(x0, h, depth, +1.0),
           fillet(x1, h, depth, -1.0))
    if ANCHOR:
        for i in ids:
            p.changeDynamics(i, -1, frictionAnchor=1)
    return ids


def flank_profile(board, move, rows=None):
    """What REALLY stands beside this slot, row band by row band - one word per
    side (`flanks`) cannot say "tight below, open above".

    Returns (left, right), each a list of bands, lowest first:

        (j_lo, j_hi, gap, what)

    rows in the PIECE's frame (0 = its bottom), `gap` = empty columns to the
    obstacle (0 = touching), `what` = "piece" or "wall". Bands merge while gap
    and kind agree. Rows above the piece count too, since it descends through
    them; `rows` = how far up to look (default: the board's height).
    """
    r0, c0 = move["row"], move["col"]
    n_rows = board.shape[0] if rows is None else int(rows)
    n_cols = board.shape[1]
    js = sorted({cj for _ci, cj in move["cells"]})
    out = []
    for side in (-1, +1):
        bands = []
        for jj in range(min(js), n_rows - r0):
            mine = sorted(c0 + ci for ci, cj in move["cells"] if cj == jj)
            if not mine:
                # above the piece's own top, its widest row is what sweeps past
                mine = sorted(c0 + ci for ci, cj in move["cells"])
            edge = mine[0] if side < 0 else mine[-1]
            rr = r0 + jj
            span = (range(edge - 1, -1, -1) if side < 0
                    else range(edge + 1, n_cols))
            hit = next((c for c in span if rr < n_rows and board[rr, c]), None)
            if hit is not None:
                gap, what = abs(edge - hit) - 1, "piece"
            else:
                # no piece means the panel, at that distance - never "open", a
                # fiction once the whole difficulty of a Z (CLAUDE.md §7.10)
                gap, what = (edge if side < 0 else n_cols - 1 - edge), "wall"
            if bands and bands[-1][2] == gap and bands[-1][3] == what:
                bands[-1] = (bands[-1][0], jj, gap, what)
            else:
                bands.append((jj, jj, gap, what))
        out.append([tuple(b) for b in bands])
    return out[0], out[1]


def flanks(board, move):
    """(left, right), each "wall", "piece" or "open": the column just outside
    the piece's footprint, over the rows it will occupy. It names the trained
    world."""
    cells = move["cells"]
    r0, c0 = move["row"], move["col"]
    rows = sorted({r0 + r for _c, r in cells})
    w = max(c for c, _r in cells) + 1
    out = []
    for col in (c0 - 1, c0 + w):
        if col < 0 or col >= board.shape[1]:
            out.append("wall")
        elif any(board[r, col] for r in rows if r < board.shape[0]):
            out.append("piece")
        else:
            out.append("open")
    return tuple(out)


def pose(name, move, pitch, thick):
    """(position, Rotation) that puts the piece on the cells `move` chose: the
    placer's in-plane turn, then a quarter turn about x to stand it up; the
    position is the centroid of the target cells (checked to 1e-9 m).
    The grid is pitched `cell + seam` ACROSS but `cell` UP: stacked pieces rest
    face on face (a seam per row put the 7th move 35.5 mm short).
    """
    R = (Rot.from_euler("x", 90, degrees=True)
         * Rot.from_euler("z", 90 * move["turns"], degrees=True))
    cells = normalise(turn(PIECES[name][0], move["turns"]))
    world = np.array([[(move["col"] + i + 0.5) * pitch, 0.0,
                       (move["row"] + j + 0.5) * thick] for i, j in cells])
    return world.mean(axis=0), R


def entry(name, move, pitch, thick, rows, back_off=0.0):
    """Where the piece starts so that lowering it lands it on its cells: same
    pose, lifted clear above the shelf."""
    pos, R = pose(name, move, pitch, thick)
    return pos + np.array([0.0, 0.0, (rows * pitch - pos[2]) + back_off]), R


# A COMPLIANT contact on the pieces, which are cardboard. Rigid, a square corner
# stands on its neighbours (1355 N vertical, 1.2 N sideways); this one sinks to the
# flank and seats (164 N, 81 N sideways; `33_wedge_probe`). Softer broke the grasp,
# since the pads touch the same value (1206 mm, `pads 0`). None = rigid, the old arm.
# 2026-10-06 (Lorenzo): DEFAULT LOWERED to 8000 N/m, damping scaled with sqrt(k) to keep the
# same damping ratio (400 * sqrt(8000/30000) = 207). 30000 400 is the old arm
# (`--neighbour-contact 30000 400`). Measured before the change, I s7158 at a 50/75% slot:
# 120.5/120.1 mm at 30000, 119.9/119.3 at 8000 (super-ellipse pieces) - no effect there.
CONTACT = (8000.0, 207.0)       # (contactStiffness [N/m], contactDamping [Ns/m])
RESTITUTION = 0.0               # cardboard is inelastic; a big value is an arm
# FRICTION ANCHORS on the shelf and on every seated piece. Without them a seated
# S with a Z on it CREPT 2.9 mm in 60 s with nothing touching it (Bullet's
# friction drifts under a steady sideways load; 600 solver iterations: the
# same), and in a game the S and Z of seed 216 crept 3.9 mm into the O's slot
# while the O was carried. With anchors: 0.0 mm in 60 s
# (`campaign/overnight_2026_09_30/creep_probe.py`, 2026-10-01). Set by the callers.
ANCHOR = False


def soften(body, mu=None):
    """Apply `CONTACT` / `RESTITUTION` to one PIECE (panels stay rigid), in a
    separate `changeDynamics` call, and read the friction back: some
    combinations silently drop it (see `lib/world.build_body`)."""
    if CONTACT is None and RESTITUTION is None:
        return body
    kw = {}
    if CONTACT is not None:
        kw.update(contactStiffness=float(CONTACT[0]),
                  contactDamping=float(CONTACT[1]))
    if RESTITUTION is not None:
        kw.update(restitution=float(RESTITUTION))
    p.changeDynamics(body, -1, **kw)
    want = MU if mu is None else float(mu)
    got = p.getDynamicsInfo(body, -1)[1]
    if abs(got - want) > 1e-9:                   # wrong friction = wrong experiment
        raise RuntimeError(f"the compliant contact dropped the friction: "
                           f"asked {want}, read {got}")
    if ANCHOR:
        p.changeDynamics(body, -1, frictionAnchor=1)   # its own call, as above
    return body


def piece_shapes(obj, colour=None, visual=True):
    """(collision, visual) shape ids for one tetromino, CHAMFER included - so the
    pieces the robot handles get it too (until 2026-09-03 they did not)."""
    # union of boxes - never a hull, a tetromino is not convex
    parts = obj["parts"]
    if colour is None:
        colour = PIECES[obj["name"]][1]
    if CHAMFER > 0:
        # one convex prism per column, corners cut: still a union
        cols = columns(parts)
        where = [list(o) for _h, o in cols]
        shape = dict(shapeTypes=[p.GEOM_MESH] * len(cols),
                     fileNames=[prism(h[0], h[1], h[2], CHAMFER) for h, _o in cols],
                     meshScales=[[1, 1, 1]] * len(cols))
    else:
        where = [list(q["origin"]) for q in parts]
        shape = dict(shapeTypes=[p.GEOM_BOX] * len(parts),
                     halfExtents=[list(q["half"]) for q in parts])
    cs = p.createCollisionShapeArray(collisionFramePositions=where, **shape)
    vs = p.createVisualShapeArray(visualFramePositions=where,
                                  rgbaColors=[list(colour)] * len(where), **shape)
    cs = p.createCollisionShapeArray(collisionFramePositions=where, **shape)
    vs = (p.createVisualShapeArray(visualFramePositions=where,
                                   rgbaColors=[list(colour)] * len(where),
                                   **shape) if visual else -1)
    return cs, vs


def spawn(obj, pos, quat, fixed=False, mu=None):
    """A piece as a body. `fixed` only to carry one down kinematically before
    letting go; the default keeps it free."""
    # read late, not as a default argument: a run may change MU after import
    mu = MU if mu is None else mu
    cs, vs = piece_shapes(obj)
    b = p.createMultiBody(0.0 if fixed else float(obj["mass"]), cs, vs,
                          list(pos), list(quat))
    p.changeDynamics(b, -1, lateralFriction=mu)
    return soften(b, mu)


def support_margin(board, move, own=True):
    """[cells] how far the CoM is INSIDE the columns carrying the piece;
    negative = it tips (a -1.00 margin drifted 62 mm). `own=True` also counts a
    cell resting on the piece's own cell - wrong over a hole (see `stable`), kept
    for `grasp.stands`, where the nest rails carry the rest."""
    cells = move["cells"]
    r0, c0 = move["row"], move["col"]
    carried = [c for c, r in cells
               if (r0 + r == 0                                # on the shelf floor
                   or board[r0 + r - 1, c0 + c]               # on a filled cell
                   or (own and (c, r - 1) in cells))]         # on its own cell
    if not carried:
        return -np.inf
    lo, hi = c0 + min(carried), c0 + max(carried) + 1
    com = c0 + np.mean([c for c, _r in cells]) + 0.5
    return float(min(com - lo, hi - com))


def stable(board):
    """A `keep=` filter for `placement.choose`: reject what would tip over (a
    filter, never a weight: CLAUDE.md §7.6). Only cells on the floor or on a
    neighbour carry it, CoM strictly inside (2026-09-24: a Z on one cell, CoM on
    its edge, tipped 68 deg, 184 mm; the old rule counted the piece's own cells,
    commit b9f098a).
    """
    return lambda move: support_margin(board, move, own=False) > 0.0


def slot_air(cells, seam=None):
    """[m] per side, the air beside a piece THIS WIDE: a solid n-cell piece in n
    pitches leaves `n * SEAM / 2` per side (a constant `SEAM / 2` made Z, J and I
    look 2-4x tighter than they are). Measured to the neighbour's cell boundary,
    so it errs on the tight side."""
    s = SEAM if seam is None else float(seam)
    n = max(i for i, _j in cells) - min(i for i, _j in cells) + 1
    return n * s / 2


def support_profile(board, move):
    """WHAT THE PIECE LANDS ON, column by column (the model has no floor).

    One entry per column, `(i, j_lo, drop, what)`: `i` the column in the piece
    frame, `j_lo` the piece's lowest row there, `drop` the empty rows below it,
    `what` "piece" or "floor". NOT banded: columns end at different heights (an
    L's tooth), so the caller bands on the surface, which needs `j_lo`.
    """
    r0, c0 = move["row"], move["col"]
    out = []
    for ci in sorted({c for c, _j in move["cells"]}):
        j_lo = min(cj for a, cj in move["cells"] if a == ci)
        rr = r0 + j_lo                                  # the piece's own bottom
        below = [r for r in range(rr - 1, -1, -1) if board[r, c0 + ci]]
        if below:
            drop, what = rr - 1 - below[0], "piece"
        else:
            drop, what = rr, "floor"                    # down to the shelf itself
        out.append((ci, j_lo, drop, what))
    return out


def model_support(board, move):
    """`support_profile` in the model frame: the half turn mirrors the columns,
    the support is AHEAD of the tip, and `j_lo` becomes that column's HIGHEST row.
    """
    wide = max(ci for ci, _cj in move["cells"]) + 1
    tall = max(cj for _ci, cj in move["cells"]) + 1
    return sorted((wide - 1 - ci, tall - 1 - j_lo, drop, what)
                  for ci, j_lo, drop, what in support_profile(board, move))


def model_profile(board, move):
    """`flank_profile` in Yang Lin's model frame (the half turn of
    `06_pipeline.to_shelf`): left and right swap and the rows reverse, since in
    the model the tip enters first. Only the bands beside the piece cross over.
    """
    tall = max(cj for _ci, cj in move["cells"]) + 1
    left, right = flank_profile(board, move)

    def flip(bands):
        out = []
        for j_lo, j_hi, gap, what in bands:
            if j_lo >= tall:
                continue
            lo, hi = j_lo, min(j_hi, tall - 1)
            out.append((tall - 1 - hi, tall - 1 - lo, gap, what))
        return sorted(out)

    return flip(right), flip(left)


def model_flanks(walls):
    """`flanks()` in Yang Lin's model frame: the half turn swaps left and right."""
    return tuple(reversed(tuple(walls)))


def snap(target, eye_dir, dist, size=(760, 620)):
    """Camera placed by hand: the shelf is open on both faces, so any eye off
    the plane sees into it."""
    e = np.asarray(eye_dir, float)
    eye = np.asarray(target, float) + dist * e / np.linalg.norm(e)
    view = p.computeViewMatrix(eye.tolist(), list(target), [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(50, size[0] / size[1], 0.02, 8.0)
    img = p.getCameraImage(size[0], size[1], view, proj,
                           renderer=p.ER_TINY_RENDERER)[2]
    return np.reshape(img, (size[1], size[0], 4))[:, :, :3]
