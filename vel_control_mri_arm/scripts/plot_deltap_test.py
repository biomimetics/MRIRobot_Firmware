#!/usr/bin/env python3
"""Plot the CSV produced by Main_Test_DeltaP_Csv.lf (via DeltaPCsvLogger) --
a PositionPidController-driven step-response target driving
Small_DeltaP_Controller, run through FakeMotorPlant so the plot shows the
simulated real motor response. The delta-position analogue of
plot_svc_test.py.

Produces a 3-panel figure: remaining error + commanded velocity, simulated
position, and planned pulse run_duration/direction shaded by pass-through
vs. small-velocity (bang-bang) mode.

---- Quickstart ----
    cd vel_control_mri_arm
    # one-time (or whenever dpos_pulse_mpc.c/h or Main_Test_DeltaP_Csv.lf change):
    #   set `#define PRINT_DPOS 1` in src_c/dpos_pulse_mpc.h, then:
    lfc-dev src/Main_Test_DeltaP_Csv.lf
    #   revert PRINT_DPOS back to 0 once built (only build-time matters)
    ./bin/Main_Test_DeltaP_Csv
    python3 scripts/plot_deltap_test.py

This script itself needs no PRINT_DPOS/build step -- it just reads whatever
CSV Main_Test_DeltaP_Csv.lf's last run produced (default path below). Also
works on any CSV with the same columns, e.g. a single manual run of
Main_Test_DeltaP_Sweep.lf (see that file's header comment) or one of
scripts/sweep_deltap_weights.py's csv_data/deltap_sweep/runs/run_NNNN.csv
files:
    python3 scripts/plot_deltap_test.py csv_data/deltap_sweep/runs/run_0000.csv

Usage:
    python3 scripts/plot_deltap_test.py [path/to/csv]
"""

import sys

import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_CSV_PATH = "csv_data/deltap_test_log.csv"

# Mirrors the MotorState enum in src_c/motor_model.h.
MOTOR_STATE_NAMES = {
    0: "STOPPED",
    1: "STARTING",
    2: "RUNNING",
    3: "STOPPING",
}


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV_PATH
    df = pd.read_csv(csv_path)

    fig, (ax_err, ax_pos, ax_pulse) = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

    ax_err.set_title("Remaining error vs. commanded velocity")
    ax_err.set_ylabel("rad / rad/s")
    ax_err.plot(df.time_s, df.remaining_error, label="remaining error (rad)", color="tab:red")
    ax_err.plot(df.time_s, df.command_velocity, label="commanded velocity (rad/s)", color="tab:blue", alpha=0.6)
    ax_err.axhline(0.0, color="tab:gray", linestyle=":", linewidth=1)
    ax_err.legend(loc="upper right")

    ax_pos.set_title("Simulated position (FakeMotorPlant)")
    ax_pos.set_ylabel("rad")
    ax_pos.plot(df.time_s, df.position, color="tab:green")

    ax_pulse.set_title("Planned pulse run_duration (signed by direction), shaded by mode")
    ax_pulse.set_xlabel("time (s)")
    ax_pulse.set_ylabel("run_duration (s), signed")
    signed_duration = df.planned_run_duration * df.planned_dir
    ax_pulse.plot(df.time_s, signed_duration, color="tab:purple")

    # Shade small-velocity (bang-bang) regions -- pass_through == 0.
    in_small_vel = df.pass_through == 0
    ax_pulse.fill_between(
        df.time_s, ax_pulse.get_ylim()[0], ax_pulse.get_ylim()[1],
        where=in_small_vel, color="tab:red", alpha=0.1,
        label="small-velocity mode", step="post",
    )
    ax_pulse.legend(loc="upper right")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
