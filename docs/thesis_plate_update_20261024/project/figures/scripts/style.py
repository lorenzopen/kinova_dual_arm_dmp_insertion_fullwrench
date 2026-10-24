"""One look for every thesis figure: serif, few lines, no top/right frame."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

FIG = Path(__file__).resolve().parents[1]           # thesis_english_2026/figures
REPO = FIG.parents[1]                                 # TESI_antiG

plt.rcParams.update({
    "font.family": "serif", "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9,
    "legend.fontsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.spines.top": False, "axes.spines.right": False, "pdf.fonttype": 42,
    "savefig.bbox": "tight", "axes.grid": False, "legend.frameon": False,
})

# piece colours of the simulation (pieces.COLOURS), so a T is purple everywhere
PIECE = {"I": "#00bcd4", "O": "#e6c800", "T": "#a000c8", "S": "#00b030",
         "Z": "#e01818", "J": "#1a33e6", "L": "#f28000"}
LEFT, RIGHT = "#1f6fb2", "#d9622b"                    # the two hands
GREY = "0.45"
HELD, FREE = "#1f6fb2", "#d9622b"                     # UCM: held (r < 1) / free (r > 1)


def save(fig, rel):
    """figures/<chapter>/<name>.pdf and .png"""
    out = FIG / rel
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out.with_suffix(".pdf"))
    fig.savefig(out.with_suffix(".png"), dpi=200)
    plt.close(fig)
    print("  ->", out.with_suffix(".pdf").relative_to(FIG))
