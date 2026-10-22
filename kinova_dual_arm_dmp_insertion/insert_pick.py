"""`insert`, but the arms PICK the piece themselves from a fixed corner of the shelf.

    python -m kinova_dual_arm_dmp_insertion.insert_pick check  W --arm R --slow 3 [--urdf gen3.urdf]
    python -m kinova_dual_arm_dmp_insertion.insert_pick insert W --arm R --slow 3 [--twist]

The piece stands on the shelf floor, in its SLOT orientation (as the dashed outline of
the board figure), against the end wall NEAR THE KINOVA BASES, in the shelf's mid-plane;
the board is EMPTY. `check` prints where exactly (gap from the wall, spacer under it if
the pads would touch the floor). Then:

    tare, nothing touching -> both tools above the piece, open -> [Enter: piece in place]
    -> down, close, squeeze in closed loop on the measured force -> lift 3 cm, then up to
    the height of the route start -> across and turned to the route start pose ->
    [Enter: board built] -> the route, the slow seat, the release: as `main insert`.

The arms hold the piece (the squeeze loop and the SOCP keep running) while you build
the board. Everything else, logs and safety included, is `main insert`.

`check` also solves the inverse kinematics of EVERY commanded flange pose on the
controller's URDF (reach_ik.py): no collision check, the lowest joint height is printed.
"""
import os
import sys
import threading

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import geometry as GE
from . import main as M
from .controller import PICKUP_N, Insertion

MARGIN = 0.010            # [m] pads clear of the end wall and of the floor by this much
LIFT = 0.030              # [m] first, slow, straight up off the floor


def pick_pose(r, turn_deg=0.0, wall_gap=0.0, riser=None):
    """The piece's pose at the pick (shelf frame) and what the user must do.
    Slot orientation (+ turn about the shelf y), lowest point on the floor (+ riser),
    the side towards the bases at `wall_gap` from the end wall, or further if the
    tri-pad would touch the wall."""
    R = Rot.from_euler("y", turn_deg, degrees=True) * r.R_target
    cells = np.asarray(r.cells, float)
    edge = min(np.linalg.norm(a - b) for i, a in enumerate(cells) for b in cells[i + 1:])
    h = 0.5 * edge
    corners = np.array([c + [sx * h, sy * h, 0.0] for c in cells for sx in (-1, 1) for sy in (-1, 1)])
    w = R.apply(corners)                                # piece frame (origin = CoM) -> shelf axes
    xmin, xmax, zmin = w[:, 0].min(), w[:, 0].max(), w[:, 2].min()
    # the grasp centre relative to the CoM, at this orientation
    g_rel = R.apply(r.grasp_R.inv().apply(r.centre - r.grasp_c))
    foot = GE.R_TRI + GE.PAD_R                          # the tri-pad's reach around the grasp point
    wall = float(r.shelf["inner_faces"][1]) * r.eff_scale
    x_touch = wall - xmax - float(wall_gap)
    x_pads = wall - g_rel[0] - foot - MARGIN            # tri-pad clear of the end wall
    cx = min(x_touch, x_pads)
    need = foot + MARGIN - (g_rel[2] - zmin)            # tri-pad clear of the floor
    lift = max(0.0, need) if riser is None else float(riser)
    c = np.array([cx, 0.0, lift - zmin])
    # will it stand by itself? the CoM over the bottom faces
    bottom = w[np.abs(w[:, 2] - zmin) < 1e-3]
    b0, b1 = bottom[:, 0].min(), bottom[:, 0].max()
    gap = wall - (cx + xmax)
    if b0 - 1e-3 <= 0.0 <= b1 + 1e-3:
        stands = "yes" if b1 - b0 >= 0.5 * (xmax - xmin) - 1e-3 else "balanced on a narrow base: hold it until the pads close"
    elif 0.0 > b1 and gap < 1e-3:
        stands = "leaning on the wall (it tips towards it)"
    else:
        stands = "NO: hold it until the pads close"
    return dict(c=c, R=R, wall_gap=gap, riser=lift, wall=wall, stands=stands,
                box_x=(cx + xmin, cx + xmax), box_z=(lift, lift + w[:, 2].max() - zmin),
                grasp=c + g_rel, pads_floor=c[2] + g_rel[2] - foot, pads_wall=wall - (c[0] + g_rel[0] + foot))


class PickInsertion(Insertion):
    def __init__(self, route, backend, pick=None, interactive=True, **kw):
        super().__init__(route, backend, **kw)
        self.pick = pick
        self.interactive = interactive
        if hasattr(backend, "held"):                    # the dry run: the piece starts at the pick
            backend.c, backend.R = pick["c"].copy(), pick["R"]

    def hold_until(self, msg):
        """Keep the loops running (squeeze, SOCP) at the current pose until [Enter]."""
        if not self.interactive:
            self.confirm(msg)
            return
        done = threading.Event()
        threading.Thread(target=lambda: (input(f"\n  >>> {msg} [Enter] "), done.set()), daemon=True).start()
        while not done.is_set():
            self.drive(self.c_cmd, self.R_cmd)

    def _goto(self, pL, qL, pR, qR, phase):
        """Both tools in a straight line from where they are (open_pose's way)."""
        self.phase = phase
        m = self.b.read()
        aL, aR = np.asarray(m["pL"]), np.asarray(m["pR"])
        bL, bR = Rot.from_quat(m["qL"]), Rot.from_quat(m["qR"])
        dist = max(np.linalg.norm(pL - aL), np.linalg.norm(pR - aR))
        turn = max((bL.inv() * qL).magnitude(), (bR.inv() * qR).magnitude())
        secs = max(dist / self.motion_speed, turn / self.turn_rate, 2.0)
        t0 = self.b.now()
        while True:
            u = min(1.0, (self.b.now() - t0) / secs)
            self.tick(aL + u * (pL - aL), bL * Rot.from_rotvec(u * (bL.inv() * qL).as_rotvec()),
                      aR + u * (pR - aR), bR * Rot.from_rotvec(u * (bR.inv() * qR).as_rotvec()), 3.0)
            if u >= 1.0:
                return

    def present(self):
        r, p = self.r, self.pick
        R0, Rp, cp = r.rots[0], p["R"], np.asarray(p["c"], float)
        # the hands as trained at the grasp pose, carried to the pick attitude: after the
        # transfer they are back at the trained roll about the pad axis
        dRp = Rp * r.grasp_R.inv()
        reach = float(np.linalg.norm(r.centre - self.cL))
        self.axis = dRp.apply(self.axis)
        self.qL, self.qR = dRp * self.qL, dRp * self.qR
        centre = cp + Rp.apply(r.grasp_R.inv().apply(r.centre - r.grasp_c))
        self.cL, self.cR = centre - reach * self.axis, centre + reach * self.axis
        z_top = max(float(r.grasp_c[2]), float(cp[2]))
        up = np.array([0.0, 0.0, z_top - cp[2]])
        self.b.tare()                                   # F/T zero, hands in free air
        self._goto(self.cL + up, self.qL, self.cR + up, self.qR, "to_above_pick")
        self.confirm(f"board EMPTY, piece in the corner near the bases, slot orientation, "
                     f"{1000 * p['wall_gap']:.0f} mm from the wall"
                     + (f", on a {1000 * p['riser']:.0f} mm spacer" if p["riser"] > 1e-3 else ""))
        self.b.tare()                                   # above it, nothing touching
        self._goto(self.cL, self.qL, self.cR, self.qR, "down_to_pick")
        self.approach()
        print(f"      approach: {self.normal_force():.1f} N measured")
        pickup = min(r.derived_N, PICKUP_N)
        self.squeeze(pickup)
        print(f"      squeeze: {self.normal_force():.1f} N measured, asked {2 * pickup:.1f}")
        self.freeze(cp, Rp)
        self.R_rel = Rp.inv() * Rot.from_quat(self.meas["qL"])
        self.R_cmd, self.c_cmd = Rp, cp
        self.phase = "lift"
        c1 = cp + [0.0, 0.0, LIFT]
        self.move(cp, Rp, c1, Rp, LIFT / (0.25 * self.motion_speed))
        self.phase = "carry_up"
        c2 = np.array([cp[0], cp[1], z_top])
        self.move(c1, Rp, c2, Rp, 0.0)
        self.phase = "carry_to_start"
        self.move(c2, Rp, r.grasp_c, R0, 0.0)
        self.phase = "hold_build"
        self.hold_until("the piece is at the route start: BUILD THE BOARD now (neighbours as in the figure), "
                        "hands away, then")


def ik_report(a, cfg, rows, every=25):
    """IK of every `every`-th commanded flange pose, per arm and phase, on the controller's URDF."""
    from .geometry import Frame
    from .reach_ik import Chain, check_path
    out = []
    for side, cols in (("left", slice(1, 8)), ("right", slice(8, 15))):
        urdf = a.urdf or cfg[side]["urdf_path"]
        if not os.path.isfile(urdf):
            out.append(f"{side}: IK skipped, no URDF at {urdf} (--urdf <gen3.urdf>)")
            continue
        acfg = cfg[side]
        shelf = Frame(acfg["shelf_in_base"]["xyz"], acfg["shelf_in_base"]["quat"])
        ee_in_tool = Frame(acfg["tool_in_ee"]["xyz"], acfg["tool_in_ee"]["quat"]).inverse()
        chain = Chain(urdf, acfg.get("base_frame", "base_link"), acfg["ee_frame"])
        poses, phases = [], []
        for row in rows[::every]:
            v = np.asarray(row[cols], float)
            pb, qb = shelf.apply(v[:3], Rot.from_quat(v[3:]))
            poses.append((pb + qb.apply(ee_in_tool.t), (qb * ee_in_tool.R).as_matrix()))
            phases.append(row[15])
        res = check_path(chain, poses)
        out.append(f"{side} IK (flange short of its target, max per phase; lowest joint above the base):")
        for ph in dict.fromkeys(phases):
            r = [x for x, f in zip(res, phases) if f == ph]
            ep = max(x[0] for x in r)
            low = min(x[2] for x in r)
            flag = "" if ep < 0.002 else ("  (impedance absorbs it)" if ep < 0.02 else "  !! OUT OF REACH")
            out.append(f"    {ph:16s} {1000 * ep:5.1f} mm, joints >= {1000 * low:4.0f} mm{flag}")
    return out


def main():
    def extra(ap):
        ap.add_argument("--pick-turn", type=float, default=0.0,
                        help="[deg] the piece at the pick turned this much about the shelf y from its slot "
                             "orientation (default 0: exactly as in the slot)")
        ap.add_argument("--wall-gap", type=float, default=0.0,
                        help="[m] the piece this far from the end wall near the bases (default 0: touching, "
                             "or as far as the tri-pad needs)")
        ap.add_argument("--riser", type=float, default=None,
                        help="[m] a spacer under the piece (default: only if the pads would touch the floor)")
        ap.add_argument("--urdf", default=None,
                        help="URDF for the IK in `check` (default: cell.yaml urdf_path)")
        ap.set_defaults(log_tag="_pick")

    state = {}

    def make(a, r, b, **kw):
        p = pick_pose(r, a.pick_turn, a.wall_gap, a.riser)
        state.update(a=a, p=p)
        print(f"  PICK: piece in its slot orientation{' turned %+.0f deg' % a.pick_turn if a.pick_turn else ''}, "
              f"against the end wall near the bases (shelf x {1000 * p['wall']:.0f} mm), gap "
              f"{1000 * p['wall_gap']:.0f} mm"
              + (f", on a {1000 * p['riser']:.0f} mm spacer" if p["riser"] > 1e-3 else ", on the floor")
              + f"; it spans shelf x {1000 * p['box_x'][0]:.0f}..{1000 * p['box_x'][1]:.0f}, "
                f"z {1000 * p['box_z'][0]:.0f}..{1000 * p['box_z'][1]:.0f} mm")
        print(f"        stands by itself: {p['stands']}; "
              f"tri-pad {1000 * p['pads_wall']:.0f} mm from the wall, {1000 * p['pads_floor']:.0f} mm above the floor")
        return PickInsertion(r, b, pick=p, interactive="confirm" not in kw, **kw)

    if len(sys.argv) > 1 and sys.argv[1] == "check":
        # the dry run, then the IK of everything it commanded
        orig = M.reach_report

        def reach_and_ik(route, cfg, rows):
            lines = orig(route, cfg, rows)
            try:
                lines += ik_report(state["a"], cfg, rows)
            except Exception as e:                       # the IK is a report, never a blocker
                lines.append(f"IK skipped: {e}")
            return lines
        M.reach_report = reach_and_ik
    M.main(sys.argv[1:], doc=__doc__, extra=extra, make=make)


if __name__ == "__main__":
    main()
