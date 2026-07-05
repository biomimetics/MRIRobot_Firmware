#!/usr/bin/env python3
"""Sweep Small_Velocity_Controller's MPC cost weights and compare the
downstream effect on pulse count vs. settling time for a closed-loop
position step (PositionPidController -> Small_Velocity_Controller ->
FakeMotorPlant, see src/Main_Test_SVC_Sweep.lf).

The goal this is built for: find weight combinations that minimize total
pulse count while still converging to the target position as fast as
possible -- these two objectives trade off against each other, so the
script reports the Pareto-optimal combinations rather than a single "best"
answer.

---- Quickstart ----
    cd vel_control_mri_arm
    python3 scripts/sweep_svc_weights.py --build
That one command flips PRINT_SVC to 1 in src_c/pulse_mpc.h, builds
src/Main_Test_SVC_Sweep.lf with lfc-dev, reverts PRINT_SVC to 0 (the
checked-in source is never left mutated), then runs the default 3x3x3
W_debt/W_switch/W_pulse grid (27 runs) against a 1.0 rad position step.
It opens an interactive plot window at the end (pass --no-show to skip).

Output, all under csv_data/sweep/:
    summary.csv          one row per weight combination: pulse_count,
                          settling_time_s, converged, final_error,
                          weights_clamped (true if PulseMPC_ValidateWeights
                          silently capped this combination -- see
                          src_c/pulse_mpc.c -- meaning the run doesn't
                          actually reflect the requested weights)
    pareto_scatter.png    settling time vs. pulse count, Pareto-optimal
                          combinations connected/labeled
    heatmaps.png          pulse count over the W_debt/W_switch grid,
                          one panel per W_pulse value
    runs/run_NNNN.csv     the raw per-sample log for each combination (same
                          columns CsvLogger.lf always writes -- plot any one
                          of these individually with scripts/plot_svc_test.py)

Usage:
    cd vel_control_mri_arm
    python3 scripts/sweep_svc_weights.py --build
    # subsequent sweeps can skip --build if src/Main_Test_SVC_Sweep.lf and
    # pulse_mpc.c/h haven't changed:
    python3 scripts/sweep_svc_weights.py --grid my_grid.json

Custom sweep grids are JSON objects mapping a subset of
{"W_debt", "W_switch", "W_pulse"} to a list of values, e.g.:
    {"W_debt": [0.5, 1.0, 2.0], "W_switch": [0.02, 0.05], "W_pulse": [0.001]}
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
BINARY_PATH = PROJECT_DIR / "bin" / "Main_Test_SVC_Sweep"
LF_SOURCE = "src/Main_Test_SVC_Sweep.lf"
PULSE_MPC_H = PROJECT_DIR / "src_c" / "pulse_mpc.h"

DEFAULT_GRID = {
    "W_debt": [0.5, 1.0, 2.0],
    "W_switch": [0.02, 0.05, 0.1],
    "W_pulse": [0.0005, 0.001, 0.005],
}

# State column values -- mirrors the MotorState enum in src_c/pulse_mpc.h.
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
    """Temporarily flips PRINT_SVC to 1, builds Main_Test_SVC_Sweep, then
    reverts pulse_mpc.h -- only the *compiled binary* needs PRINT_SVC=1
    (baked in at build time), so the source is safe to revert immediately
    after building, same as the manual dance documented in
    Main_Test_SVC.lf/Main_Test_SVC_Csv.lf's header comments."""
    original = PULSE_MPC_H.read_text()
    if "#define PRINT_SVC 0" not in original:
        sys.exit(
            f"Expected '#define PRINT_SVC 0' in {PULSE_MPC_H}, didn't find it -- "
            "check the file hasn't changed and edit this script's assumption if it has."
        )
    try:
        PULSE_MPC_H.write_text(original.replace("#define PRINT_SVC 0", "#define PRINT_SVC 1", 1))
        print(f"Building {LF_SOURCE} with PRINT_SVC=1...")
        subprocess.run([lfc_path, LF_SOURCE], cwd=PROJECT_DIR, check=True)
    finally:
        PULSE_MPC_H.write_text(original)
    if not BINARY_PATH.exists():
        sys.exit(f"Build finished but {BINARY_PATH} is missing -- check lfc-dev output above.")


def run_one(weights, target_position, pid_gains, csv_path, timeout_s):
    env = os.environ.copy()
    env["SVC_W_DEBT"] = str(weights["W_debt"])
    env["SVC_W_SWITCH"] = str(weights["W_switch"])
    env["SVC_W_PULSE"] = str(weights["W_pulse"])
    env["SVC_TARGET_POSITION"] = str(target_position)
    env["SVC_CSV_PATH"] = str(csv_path)
    for k, v in pid_gains.items():
        if v is not None:
            env[f"SVC_PID_{k.upper()}"] = str(v)

    proc = subprocess.run(
        [str(BINARY_PATH)], cwd=PROJECT_DIR, env=env,
        capture_output=True, text=True, timeout=timeout_s,
    )
    clamped = "Capping W_debt" in proc.stdout or "Capping W_switch" in proc.stdout
    return clamped


def compute_metrics(csv_path, target_position, tolerance):
    df = pd.read_csv(csv_path)

    state = df["state"]
    pulse_count = int(((state.shift(1) == MOTOR_STOPPED) & (state == MOTOR_STARTING)).sum())

    err = (df["position"] - target_position).abs()
    within = err <= tolerance
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

    return {
        "pulse_count": pulse_count,
        "settling_time_s": settling_time,
        "converged": converged,
        "final_error": float(err.iloc[-1]),
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
        c=converged.W_debt, cmap="viridis", s=60, label="_nolegend_",
    )
    fig.colorbar(sc, ax=ax, label="W_debt")

    if len(not_converged):
        ax.scatter(
            [ax.get_xlim()[1]] * len(not_converged), not_converged.pulse_count,
            marker="x", color="red", label=f"did not converge ({len(not_converged)})",
        )

    front = pareto_front(results, "settling_time_s", "pulse_count")
    ax.plot(front.settling_time_s, front.pulse_count, "r--", linewidth=1.5, label="Pareto front")
    for _, row in front.iterrows():
        ax.annotate(
            f"Wd={row.W_debt:g}\nWs={row.W_switch:g}\nWp={row.W_pulse:g}",
            (row.settling_time_s, row.pulse_count),
            textcoords="offset points", xytext=(6, 6), fontsize=7,
        )

    ax.set_xlabel("settling time (s)")
    ax.set_ylabel("pulse count")
    ax.set_title("MPC weight sweep: pulse count vs. settling time")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Wrote {out_path}")
    return fig


def make_heatmaps(results, out_path):
    w_pulse_values = sorted(results.W_pulse.unique())
    fig, axes = plt.subplots(1, len(w_pulse_values), figsize=(5 * len(w_pulse_values), 4.5), squeeze=False)
    axes = axes[0]

    vmin, vmax = results.pulse_count.min(), results.pulse_count.max()
    im = None
    for ax, wp in zip(axes, w_pulse_values):
        sub = results[results.W_pulse == wp]
        pivot = sub.pivot_table(index="W_switch", columns="W_debt", values="pulse_count")
        im = ax.imshow(pivot.values, cmap="magma", vmin=vmin, vmax=vmax, aspect="auto")
        ax.set_xticks(range(len(pivot.columns)))
        ax.set_xticklabels([f"{v:g}" for v in pivot.columns])
        ax.set_yticks(range(len(pivot.index)))
        ax.set_yticklabels([f"{v:g}" for v in pivot.index])
        ax.set_xlabel("W_debt")
        ax.set_ylabel("W_switch")
        ax.set_title(f"W_pulse={wp:g}")

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
    parser.add_argument("--output-dir", type=Path, default=PROJECT_DIR / "csv_data" / "sweep")
    parser.add_argument("--build", action="store_true", help="rebuild Main_Test_SVC_Sweep (flips PRINT_SVC, builds, reverts)")
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
            clamped = run_one(
                weights, args.target_position,
                {"kp": args.pid_kp, "ki": args.pid_ki, "kd": args.pid_kd},
                csv_path, args.timeout,
            )
        except subprocess.TimeoutExpired:
            print("TIMED OUT, skipping")
            continue

        metrics = compute_metrics(csv_path, args.target_position, args.tolerance)
        metrics["weights_clamped"] = clamped
        row = {**weights, **metrics, "csv_path": str(csv_path)}
        rows.append(row)
        print(
            f"pulses={metrics['pulse_count']} settle={metrics['settling_time_s']:.2f}s"
            if metrics["converged"] else f"pulses={metrics['pulse_count']} DID NOT CONVERGE",
            "(weights clamped)" if clamped else "",
        )

    results = pd.DataFrame(rows)
    summary_path = args.output_dir / "summary.csv"
    results.to_csv(summary_path, index=False)
    print(f"\nWrote {summary_path}")

    front = pareto_front(results, "settling_time_s", "pulse_count")
    print("\nPareto-optimal weight combinations (minimize both pulse count and settling time):")
    print(front[["W_debt", "W_switch", "W_pulse", "pulse_count", "settling_time_s"]].to_string(index=False))

    make_pareto_plot(results, args.output_dir / "pareto_scatter.png")
    make_heatmaps(results, args.output_dir / "heatmaps.png")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
