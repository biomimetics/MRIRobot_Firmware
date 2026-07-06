#!/usr/bin/env python3
"""Sweep Small_DeltaP_Controller's cost weights and compare the downstream
effect on pulse count, settling time, and overshoot frequency/magnitude for
a closed-loop position step (PositionPidController -> Small_DeltaP_Controller
-> FakeMotorPlant, see src/Main_Test_DeltaP_Sweep.lf). The delta-position
analogue of scripts/sweep_svc_weights.py.

Overshoot frequency/magnitude is tracked here but wasn't in the old SVC
sweep script, since pulse_mpc.c's motion-debt design never reasoned about
overshoot explicitly -- it's the whole point of this design's asymmetric
cost function (dpos_pulse_mpc_planning.md §4), so it's worth watching
directly rather than only inferring it from settling time.

---- Quickstart ----
    cd vel_control_mri_arm
    python3 scripts/sweep_deltap_weights.py --build
That one command flips PRINT_DPOS to 1 in src_c/dpos_pulse_mpc.h, builds
src/Main_Test_DeltaP_Sweep.lf with lfc-dev, reverts PRINT_DPOS to 0 (the
checked-in source is never left mutated), then runs the default weight grid
against a 1.0 rad position step. It opens an interactive plot window at the
end (pass --no-show to skip).

Output, all under csv_data/deltap_sweep/:
    summary.csv          one row per weight combination: pulse_count,
                          settling_time_s, converged, final_error,
                          overshoot_count, max_overshoot
    pareto_scatter.png    settling time vs. pulse count, colored by
                          overshoot_count, Pareto-optimal combinations
                          connected/labeled
    heatmaps.png          pulse count over the W_error/k_overshoot grid,
                          one panel per C_overshoot value
    runs/run_NNNN.csv     the raw per-sample log for each combination (same
                          columns DeltaPCsvLogger.lf always writes -- plot
                          any one of these individually with
                          scripts/plot_deltap_test.py)

Usage:
    cd vel_control_mri_arm
    python3 scripts/sweep_deltap_weights.py --build
    # subsequent sweeps can skip --build if src/Main_Test_DeltaP_Sweep.lf and
    # dpos_pulse_mpc.c/h haven't changed:
    python3 scripts/sweep_deltap_weights.py --grid my_grid.json

Custom sweep grids are JSON objects mapping a subset of
{"W_error", "k_overshoot", "C_overshoot", "sigma0"} to a list of values, e.g.:
    {"W_error": [0.5, 1.0, 2.0], "k_overshoot": [5.0, 10.0], "sigma0": [0.01]}
Any key omitted falls back to the default grid's values for that weight.
Other useful flags: --target-position, --tolerance (settling-time band),
--pid-kp/--pid-ki/--pid-kd, --timeout (per-run wall-clock kill switch),
--output-dir. Run with --help for the full list.
"""

import argparse
import itertools
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
BINARY_PATH = PROJECT_DIR / "bin" / "Main_Test_DeltaP_Sweep"
LF_SOURCE = "src/Main_Test_DeltaP_Sweep.lf"
DPOS_PULSE_MPC_H = PROJECT_DIR / "src_c" / "dpos_pulse_mpc.h"

DEFAULT_GRID = {
    "W_error": [0.5, 1.0, 2.0],
    "k_overshoot": [5.0, 10.0, 20.0],
    "C_overshoot": [0.005, 0.01, 0.02],
    "sigma0": [0.01],
}

# State column values -- mirrors the MotorState enum in src_c/motor_model.h.
MOTOR_STOPPED = 0
MOTOR_STARTING = 1


def load_grid(grid_path):
    grid = dict(DEFAULT_GRID)
    if grid_path:
        with open(grid_path) as f:
            user_grid = json.load(f)
        grid.update(user_grid)
    return grid


def find_lfc():
    lfc = shutil.which("lfc-dev")
    if lfc:
        return lfc
    fallback = PROJECT_DIR.parent / "resources" / "lingua-franca" / "bin" / "lfc-dev"
    if fallback.exists():
        return str(fallback)
    sys.exit(
        "Could not find lfc-dev on PATH or at resources/lingua-franca/bin/lfc-dev. "
        "Add it to PATH (see top-level README.md) or pass --lfc <path>."
    )


def build(lfc_path):
    """Temporarily flips PRINT_DPOS to 1, builds Main_Test_DeltaP_Sweep, then
    reverts dpos_pulse_mpc.h -- only the *compiled binary* needs
    PRINT_DPOS=1 (baked in at build time), so the source is safe to revert
    immediately after building, same as sweep_svc_weights.py's dance."""
    original = DPOS_PULSE_MPC_H.read_text()
    if "#define PRINT_DPOS 0" not in original:
        sys.exit(
            f"Expected '#define PRINT_DPOS 0' in {DPOS_PULSE_MPC_H}, didn't find it -- "
            "check the file hasn't changed and edit this script's assumption if it has."
        )
    try:
        DPOS_PULSE_MPC_H.write_text(original.replace("#define PRINT_DPOS 0", "#define PRINT_DPOS 1", 1))
        print(f"Building {LF_SOURCE} with PRINT_DPOS=1...")
        subprocess.run([lfc_path, LF_SOURCE], cwd=PROJECT_DIR, check=True)
    finally:
        DPOS_PULSE_MPC_H.write_text(original)
    if not BINARY_PATH.exists():
        sys.exit(f"Build finished but {BINARY_PATH} is missing -- check lfc-dev output above.")


def run_one(weights, target_position, pid_gains, csv_path, timeout_s):
    env = os.environ.copy()
    env["DPOS_W_ERROR"] = str(weights["W_error"])
    env["DPOS_K_OVERSHOOT"] = str(weights["k_overshoot"])
    env["DPOS_C_OVERSHOOT"] = str(weights["C_overshoot"])
    env["DPOS_SIGMA0"] = str(weights["sigma0"])
    env["DPOS_TARGET_POSITION"] = str(target_position)
    env["DPOS_CSV_PATH"] = str(csv_path)
    for k, v in pid_gains.items():
        if v is not None:
            env[f"DPOS_PID_{k.upper()}"] = str(v)

    subprocess.run(
        [str(BINARY_PATH)], cwd=PROJECT_DIR, env=env,
        capture_output=True, text=True, timeout=timeout_s,
    )


def compute_metrics(csv_path, target_position, tolerance):
    df = pd.read_csv(csv_path)

    state = df["state"]
    pulse_count = int(((state.shift(1) == MOTOR_STOPPED) & (state == MOTOR_STARTING)).sum())

    err = (df["position"] - target_position)
    abs_err = err.abs()
    within = abs_err <= tolerance
    if within.all():
        settling_time = 0.0
        converged = True
    else:
        last_bad_idx = within[~within].index[-1]
        if last_bad_idx == df.index[-1]:
            settling_time = np.nan
            converged = False
        else:
            settling_time = df["time_s"].iloc[last_bad_idx + 1] - df["time_s"].iloc[0]
            converged = True

    # Overshoot: position crosses past target_position (sign of err flips
    # relative to the initial approach direction) -- the metric this design
    # cares about that the old SVC sweep never tracked (dpos_pulse_mpc_planning.md
    # §4's whole point is penalizing this asymmetrically).
    initial_sign = np.sign(err.iloc[0]) if err.iloc[0] != 0 else 1.0
    overshot = (np.sign(err) * initial_sign) < 0
    overshoot_count = int((overshot & ~overshot.shift(1, fill_value=False)).sum())
    max_overshoot = float(abs_err[overshot].max()) if overshot.any() else 0.0

    return {
        "pulse_count": pulse_count,
        "settling_time_s": settling_time,
        "converged": converged,
        "final_error": float(abs_err.iloc[-1]),
        "overshoot_count": overshoot_count,
        "max_overshoot": max_overshoot,
    }


def pareto_front(df, x_col, y_col):
    """Rows that minimize both x_col and y_col -- no other row is at least
    as good on both and strictly better on one."""
    pts = df.dropna(subset=[x_col, y_col]).sort_values(x_col)
    front_rows = []
    best_y = np.inf
    for _, row in pts.iterrows():
        if row[y_col] < best_y:
            front_rows.append(row)
            best_y = row[y_col]
    return pd.DataFrame(front_rows)


def make_pareto_plot(results, out_path):
    fig, ax = plt.subplots(figsize=(8, 6))
    converged = results[results.converged]
    not_converged = results[~results.converged]

    sc = ax.scatter(
        converged.settling_time_s, converged.pulse_count,
        c=converged.overshoot_count, cmap="viridis", s=60, label="_nolegend_",
    )
    fig.colorbar(sc, ax=ax, label="overshoot count")

    if len(not_converged):
        ax.scatter(
            [ax.get_xlim()[1]] * len(not_converged), not_converged.pulse_count,
            marker="x", color="red", label=f"did not converge ({len(not_converged)})",
        )

    front = pareto_front(results, "settling_time_s", "pulse_count")
    ax.plot(front.settling_time_s, front.pulse_count, "r--", linewidth=1.5, label="Pareto front")
    for _, row in front.iterrows():
        ax.annotate(
            f"We={row.W_error:g}\nKo={row.k_overshoot:g}\nCo={row.C_overshoot:g}",
            (row.settling_time_s, row.pulse_count),
            textcoords="offset points", xytext=(6, 6), fontsize=7,
        )

    ax.set_xlabel("settling time (s)")
    ax.set_ylabel("pulse count")
    ax.set_title("Cost-weight sweep: pulse count vs. settling time (color = overshoot count)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Wrote {out_path}")
    return fig


def make_heatmaps(results, out_path):
    c_overshoot_values = sorted(results.C_overshoot.unique())
    fig, axes = plt.subplots(1, len(c_overshoot_values), figsize=(5 * len(c_overshoot_values), 4.5), squeeze=False)
    axes = axes[0]

    vmin, vmax = results.pulse_count.min(), results.pulse_count.max()
    im = None
    for ax, co in zip(axes, c_overshoot_values):
        sub = results[results.C_overshoot == co]
        pivot = sub.pivot_table(index="k_overshoot", columns="W_error", values="pulse_count")
        im = ax.imshow(pivot.values, cmap="magma", vmin=vmin, vmax=vmax, aspect="auto")
        ax.set_xticks(range(len(pivot.columns)))
        ax.set_xticklabels([f"{v:g}" for v in pivot.columns])
        ax.set_yticks(range(len(pivot.index)))
        ax.set_yticklabels([f"{v:g}" for v in pivot.index])
        ax.set_xlabel("W_error")
        ax.set_ylabel("k_overshoot")
        ax.set_title(f"C_overshoot={co:g}")

    if im is not None:
        fig.colorbar(im, ax=axes, label="pulse count", shrink=0.8)
    fig.suptitle("Pulse count across weight space")
    fig.savefig(out_path)
    print(f"Wrote {out_path}")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--grid", type=Path, default=None, help="JSON file overriding the default weight grid")
    parser.add_argument("--target-position", type=float, default=1.0, help="rad, step target for the PID loop")
    parser.add_argument("--tolerance", type=float, default=0.02, help="rad, settling-time tolerance band")
    parser.add_argument("--pid-kp", type=float, default=None)
    parser.add_argument("--pid-ki", type=float, default=None)
    parser.add_argument("--pid-kd", type=float, default=None)
    parser.add_argument("--timeout", type=float, default=60.0, help="wall-clock seconds per run before killing it")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_DIR / "csv_data" / "deltap_sweep")
    parser.add_argument("--build", action="store_true", help="rebuild Main_Test_DeltaP_Sweep (flips PRINT_DPOS, builds, reverts)")
    parser.add_argument("--lfc", type=str, default=None, help="path to lfc-dev, only used with --build")
    parser.add_argument("--no-show", action="store_true", help="save plots without opening an interactive window")
    args = parser.parse_args()

    if args.build:
        build(args.lfc or find_lfc())
    elif not BINARY_PATH.exists():
        sys.exit(f"{BINARY_PATH} not found -- run with --build first (see script docstring).")

    grid = load_grid(args.grid)
    combos = list(itertools.product(*grid.values()))
    keys = list(grid.keys())

    runs_dir = args.output_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for i, combo in enumerate(combos):
        weights = dict(zip(keys, combo))
        csv_path = runs_dir / f"run_{i:04d}.csv"
        print(f"[{i + 1}/{len(combos)}] {weights}", end=" -> ", flush=True)
        try:
            run_one(
                weights, args.target_position,
                {"kp": args.pid_kp, "ki": args.pid_ki, "kd": args.pid_kd},
                csv_path, args.timeout,
            )
        except subprocess.TimeoutExpired:
            print("TIMED OUT, skipping")
            continue

        metrics = compute_metrics(csv_path, args.target_position, args.tolerance)
        row = {**weights, **metrics, "csv_path": str(csv_path)}
        rows.append(row)
        print(
            f"pulses={metrics['pulse_count']} settle={metrics['settling_time_s']:.2f}s "
            f"overshoots={metrics['overshoot_count']}"
            if metrics["converged"] else f"pulses={metrics['pulse_count']} DID NOT CONVERGE",
        )

    results = pd.DataFrame(rows)
    summary_path = args.output_dir / "summary.csv"
    results.to_csv(summary_path, index=False)
    print(f"\nWrote {summary_path}")

    front = pareto_front(results, "settling_time_s", "pulse_count")
    print("\nPareto-optimal weight combinations (minimize both pulse count and settling time):")
    print(front[["W_error", "k_overshoot", "C_overshoot", "sigma0", "pulse_count", "settling_time_s", "overshoot_count"]].to_string(index=False))

    make_pareto_plot(results, args.output_dir / "pareto_scatter.png")
    make_heatmaps(results, args.output_dir / "heatmaps.png")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
