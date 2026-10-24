"""Play one Tetris game exactly as game.py does, and RECORD it for the thesis figures.

    python trace_game.py --out DIR -- --seed 7025 [game.py options]

REPORT ONLY. Nothing here changes a decision of the game: the hands' methods
are wrapped to note which step is running, and every 10th control tick the
forces and the estimate are written down. Ground truth (piece pose, true CoM)
is read for REPORTING the error, never fed back (CLAUDE.md, CONSTRAINT 7).

Writes to DIR:
    trace.npz      one row per sample: move, step, time, forces, moment, estimate
    snaps/*.png    camera snapshots at the nest, the hang, the mouth, the release
    boards.json    game.py --dump-boards (the planned board and the real pile)
    game.csv       game.py --csv
"""
import os
import runpy
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GAME_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "..", "lorenzo_data_analysis",
                                         "tetris_dual_arm", "tetris"))

argv = sys.argv[1:]
out = argv[argv.index("--out") + 1]
game_args = argv[argv.index("--") + 1:]
os.makedirs(os.path.join(out, "snaps"), exist_ok=True)

ROOT = os.path.dirname(GAME_DIR)
sys.path[:0] = [GAME_DIR, ROOT, os.path.join(ROOT, "spider_ee")]
os.chdir(GAME_DIR)
import pybullet as p                                   # noqa: E402
from scipy.spatial.transform import Rotation as Rot    # noqa: E402
import arms as ARMS                                    # noqa: E402
import handling as H                                   # noqa: E402
import placement as PL                                 # noqa: E402
import staging as ST                                   # noqa: E402
from lib.tool import wrench_on_body                    # noqa: E402

STEPS = ["start", "transit", "approach", "squeeze", "move", "roll", "hang", "route", "seat",
         "release", "descent"]
EVERY = 10                     # one sample every 10 ticks = 10 ms of simulation
STATE = dict(move=0, step="start", stage="g1", hang=0, snapped=set())
ROWS = []
_, NEST = ARMS.stations()


def snap(tag, target, dist=1.05, side=35.0, up=28.0, size=(900, 700)):
    """One camera picture (TinyRenderer, headless), saved once per (move, tag)."""
    key = (STATE["move"], tag)
    if key in STATE["snapped"]:
        return
    STATE["snapped"].add(key)
    s, u = np.radians(side), np.radians(up)
    t = np.asarray(target, float)
    eye = t + dist * np.array([np.cos(u) * np.sin(s), -np.cos(u) * np.cos(s), np.sin(u)])
    view = p.computeViewMatrix(eye.tolist(), t.tolist(), [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(40.0, size[0] / size[1], 0.05, 12.0)
    img = p.getCameraImage(size[0], size[1], viewMatrix=view, projectionMatrix=proj,
                           renderer=p.ER_TINY_RENDERER)[2]
    rgb = np.reshape(np.asarray(img, np.uint8), (size[1], size[0], 4))[:, :, :3]
    import cv2
    cv2.imwrite(os.path.join(out, "snaps", f"m{STATE['move']}_{tag}.png"),
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def shelf_centre():
    return [0.33, 0.0, 0.40]


# ------------------------------------------------------------- which step is running
def wrap(name, step):
    orig = getattr(H.Hands, name)

    def run(self, *a, **k):
        STATE["depth"] = STATE.get("depth", 0) + 1
        outer = STATE["step"]
        if STATE["depth"] == 1:              # a nested call keeps the outer step's label
            STATE["step"] = step
            if step == "move" and k.get("watch"):
                STATE["step"] = "seat" if self.seat_goal is not None else "descent"
        body = k.get("body")
        if body is None:
            body = next((x for x in a if isinstance(x, int)), None)
        if step == "approach" and body is not None:
            pos = p.getBasePositionAndOrientation(body)[0]
            # seen from the far side of the nest (+y), above: from the shelf side a post hid it
            snap("nest" if STATE["stage"] == "g2" else "feeder", [pos[0], pos[1], pos[2] + 0.05],
                 side=150.0, up=38.0)
        if step == "hang":
            STATE["hang"] += 1
            if STATE["stage"] == "g2":
                snap("hang", list(np.asarray(a[0], float)), dist=0.9)
        if step == "route":
            snap("mouth", shelf_centre(), dist=1.45, side=12.0, up=10.0)
        try:
            res = orig(self, *a, **k)
        finally:
            STATE["depth"] -= 1
            if STATE["depth"]:
                STATE["step"] = outer
        if step == "release" and STATE["stage"] == "g2":
            snap("released", shelf_centre(), dist=1.45, side=12.0, up=10.0)
        if step == "release" and STATE["stage"] == "g1":
            pos = p.getBasePositionAndOrientation(body)[0] if body is not None else [NEST[0], NEST[1], 0.1]
            snap("in_nest", [NEST[0], NEST[1], 0.12], side=150.0, up=38.0)
        return res
    setattr(H.Hands, name, run)


for name, step in (("approach", "approach"), ("squeeze", "squeeze"), ("move", "move"),
                   ("hang", "hang"), ("follow", "route"), ("tip_up", "roll"),
                   ("lower_until_seated", "seat"), ("release", "release"),
                   ("transit", "transit")):
    wrap(name, step)

_stand = ST.stand_up


def stand_up(*a, **k):
    STATE["stage"], STATE["standing_up"] = "g1", True
    try:
        return _stand(*a, **k)
    finally:
        STATE["stage"], STATE["standing_up"] = "g2", False


_init = H.Hands.__init__


def hands_init(self, *a, **k):
    # a pair built OUTSIDE stand_up is grasp #2 - also for the I and the O,
    # which the robot never stands up (the I spawns in the nest, the station stands the O)
    if not STATE.get("standing_up"):
        STATE["stage"] = "g2"
    _init(self, *a, **k)


H.Hands.__init__ = hands_init


ST.stand_up = stand_up
_choose = PL.choose


def choose(*a, **k):
    STATE["move"] += 1
    STATE["stage"] = "g1"
    return _choose(*a, **k)


PL.choose = choose
# game.py imports these by module, so the wrappers are what it calls

# ------------------------------------------------------------- the sample
_tick = H.Hands.tick


def tick(self, tL, tR, qL, qR, k_tilt, body=None):
    _tick(self, tL, tR, qL, qR, k_tilt, body)
    if body is None or self.k % EVERY:
        return
    try:
        pos, quat = p.getBasePositionAndOrientation(body)   # truth, report only
    except Exception:
        return
    pole = 0.5 * (self.L.plate_pose()[0] + self.R.plate_pose()[0])
    F, T = wrench_on_body(body, [self.L, self.R], pole)     # what the pads apply
    ext = self.external_force(body)
    axis = np.asarray(self.axis, float)
    r_est = np.full(3, np.nan)
    r_true = np.full(3, np.nan)
    unobs = np.full(3, np.nan)
    n_est = 0
    if getattr(self, "est", None) is not None and getattr(self, "_collect", False) \
            and STATE["step"] == "hang":
        Rp = Rot.from_quat(self.L.plate_pose()[1])
        r_true = Rp.inv().apply(np.asarray(pos) - pole)     # CoM from the pads, plate frame
        n_est = len(self.est._g)
        if self.k % 50 == 0:
            try:
                r_est = np.asarray(self.est.com()[0], float)
                u = self.est.unobserved()
                if len(u):
                    unobs = np.asarray(u[0], float)
            except Exception:
                pass
    wext = self.w_ext[0] if getattr(self, "w_ext", None) is not None else (np.nan,) * 3
    ROWS.append(np.concatenate([
        [STATE["move"], ["g1", "g2"].index(STATE["stage"]),
         STEPS.index(STATE["step"]), H.STEPS[0] * H.DT, STATE["hang"],
         self.L.normal_force(body), self.R.normal_force(body),
         float(self._target) if getattr(self, "_target", None) else np.nan,
         int(bool(getattr(self, "jammed", False))), n_est],
        pos, quat, F, T, ext, axis, np.asarray(wext, float), r_est, r_true, unobs,
        getattr(self, "c_cmd", None) if getattr(self, "c_cmd", None) is not None else (np.nan,) * 3]))


H.Hands.tick = tick
COLUMNS = ("move stage step t hang fL fR target jammed n_est "
           "px py pz qx qy qz qw Fx Fy Fz Tx Ty Tz Ex Ey Ez ax ay az Wx Wy Wz "
           "rex rey rez rtx rty rtz ux uy uz cx cy cz").split()

sys.argv = ["game.py"] + game_args + ["--dump-boards", os.path.join(out, "boards.json"),
                                      "--csv", os.path.join(out, "game.csv")]
try:
    runpy.run_path(os.path.join(GAME_DIR, "game.py"), run_name="__main__")
finally:
    np.savez_compressed(os.path.join(out, "trace.npz"), data=np.array(ROWS, float),
                        columns=np.array(COLUMNS))
    print(f"  trace -> {out} ({len(ROWS)} samples)")
