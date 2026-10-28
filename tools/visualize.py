"""Plot a simulated LiDAR frame (the CSV written by lidarsim_cli).

    python tools/visualize.py out/frame_000000.csv [--color label|intensity|range] [--out frame.png]

Left: bird's-eye view (x forward, y left). Right: side view (x vs z).
"""
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np

LABEL_COLORS = {
    "ground": "#8c8c8c",
    "building": "#4c72b0",
    "vehicle": "#dd8452",
    "pole": "#55a868",
    "pedestrian": "#c44e52",
    "wall": "#8172b3",
    "fog": "#b0b0b0",
}


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    intensity = np.array([float(r["intensity"]) for r in rows])
    rng = np.array([float(r["range"]) for r in rows])
    labels = np.array([r["label"] for r in rows])
    return xyz, intensity, rng, labels


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--color", default="label", choices=["label", "intensity", "range"])
    ap.add_argument("--out", default=None, help="save to this PNG instead of showing a window")
    ap.add_argument("--extent", type=float, default=60, help="half-width of the bird's-eye view, metres")
    args = ap.parse_args()

    xyz, intensity, rng, labels = load(args.csv)
    fig, (bev, side) = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw={"width_ratios": [1.1, 1]})

    if args.color == "label":
        for name, color in LABEL_COLORS.items():
            m = labels == name
            if m.any():
                bev.scatter(xyz[m, 0], xyz[m, 1], s=0.3, c=color, label=f"{name} ({m.sum():,})")
                side.scatter(xyz[m, 0], xyz[m, 2], s=0.3, c=color)
        bev.legend(markerscale=12, loc="upper right", fontsize=8)
    else:
        values = intensity if args.color == "intensity" else rng
        sc = bev.scatter(xyz[:, 0], xyz[:, 1], s=0.3, c=values, cmap="viridis")
        side.scatter(xyz[:, 0], xyz[:, 2], s=0.3, c=values, cmap="viridis")
        fig.colorbar(sc, ax=bev, label=args.color)

    bev.set(title=f"Bird's-eye view ({len(xyz):,} points)", xlabel="x (m)", ylabel="y (m)",
            xlim=(-args.extent, args.extent), ylim=(-args.extent, args.extent))
    bev.set_aspect("equal")
    side.set(title="Side view", xlabel="x (m)", ylabel="z (m)", xlim=(-args.extent, args.extent))
    for ax in (bev, side):
        ax.grid(alpha=0.2)
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=130)
        print("wrote", args.out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
