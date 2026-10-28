"""Turn the test and benchmark outputs in validation/ into README figures in docs/.

    python tools/plot_validation.py

Reads validation/noise_errors.csv, validation/fog_range.csv and validation/bench.csv
(written by lidarsim_tests and lidarsim_bench, run from the repo root).
"""
import csv
import math
import os

import matplotlib.pyplot as plt
import numpy as np


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def noise_histogram(sigma=0.02):
    errors = np.array([float(r["error_m"]) for r in read("validation/noise_errors.csv")])
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(errors * 100, bins=80, density=True, alpha=0.7, label=f"simulated ({len(errors):,} returns)")
    x = np.linspace(-4 * sigma, 4 * sigma, 400)
    pdf = np.exp(-0.5 * (x / sigma) ** 2) / (sigma * math.sqrt(2 * math.pi))
    ax.plot(x * 100, pdf / 100, "k", lw=1.5, label=f"N(0, {sigma * 100:.0f} cm) model")
    ax.set(xlabel="range error (cm)", ylabel="density", title="Range noise vs. configured Gaussian")
    ax.legend()
    fig.tight_layout()
    fig.savefig("docs/noise_histogram.png", dpi=130)


def analytic_max_range(visibility, rho=0.8, min_power=1 / 150**2):
    """Largest r with rho * exp(-2 alpha r) / r^2 >= min_power (bisection, as in test_fog.cpp)."""
    alpha = math.log(20) / visibility
    lo, hi = 0.01, 1000.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if math.log(rho) - 2 * alpha * mid - 2 * math.log(mid) - math.log(min_power) >= 0:
            lo = mid
        else:
            hi = mid
    return lo


def fog_range():
    rows = [r for r in read("validation/fog_range.csv") if float(r["visibility_m"]) > 0]
    vis = [float(r["visibility_m"]) for r in rows]
    curve = np.geomspace(15, 700, 200)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(curve, [analytic_max_range(v) for v in curve], "k-", label="link-budget equation")
    ax.plot(vis, [float(r["simulated_max_range_m"]) for r in rows], "o", label="simulator (1 cm sweep)")
    ax.set(xscale="log", xlabel="fog visibility (m)", ylabel="max detection range (m)",
           title="Detection range in fog (rho = 0.8 target)")
    ax.set_xticks(vis)
    ax.set_xticklabels([f"{v:g}" for v in vis])
    ax.minorticks_off()
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("docs/fog_range.png", dpi=130)


def speedup():
    rows = read("validation/bench.csv")
    names = [r["method"].replace("_", " ") for r in rows]
    rates = [float(r["rays_per_s"]) / 1e6 for r in rows]
    fig, ax = plt.subplots(figsize=(6, 3.5))
    bars = ax.bar(names, rates, color=["#999999", "#4c72b0", "#2a4d80"])
    ax.set(yscale="log", ylabel="M rays / s (log scale)", title="Ray tracing throughput")
    for b, r in zip(bars, rates):
        ax.text(b.get_x() + b.get_width() / 2, r * 1.15, f"{r:.3g}", ha="center", fontsize=9)
    fig.tight_layout()
    fig.savefig("docs/throughput.png", dpi=130)


if __name__ == "__main__":
    os.makedirs("docs", exist_ok=True)
    noise_histogram()
    fog_range()
    speedup()
    print("wrote docs/noise_histogram.png, docs/fog_range.png, docs/throughput.png")
