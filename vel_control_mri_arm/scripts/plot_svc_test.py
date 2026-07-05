#!/usr/bin/env python3
"""Plot the CSV produced by Main_Test_SVC_Csv.lf (via CsvLogger) -- a
triangle-wave velocity command driving Small_Velocity_Controller, run
through FakeMotorPlant so the plot shows the simulated real motor response.

Produces a 3-panel figure: desired/commanded/simulated-real velocity,
simulated position, and motionDebt shaded by pass-through vs.
small-velocity (bang-bang) mode.

---- Quickstart ----
    cd vel_control_mri_arm
    # one-time (or whenever pulse_mpc.c/h or Main_Test_SVC_Csv.lf change):
    #   set `#define PRINT_SVC 1` in src_c/pulse_mpc.h, then:
    lfc-dev src/Main_Test_SVC_Csv.lf
    #   revert PRINT_SVC back to 0 once built (only build-time matters)
    ./bin/Main_Test_SVC_Csv
    python3 scripts/plot_svc_test.py

This script itself needs no PRINT_SVC/build step -- it just reads whatever
CSV Main_Test_SVC_Csv.lf's last run produced (default path below). Also
works on any CSV with the same columns, e.g. a single manual run of
Main_Test_SVC_Sweep.lf (see that file's header comment) or one of
scripts/sweep_svc_weights.py's csv_data/sweep/runs/run_NNNN.csv files:
    python3 scripts/plot_svc_test.py csv_data/sweep/runs/run_0000.csv

Usage:
    python3 scripts/plot_svc_test.py [path/to/csv]
"""

import sys

import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_CSV_PATH = "csv_data/svc_test_log.csv"

# Mirrors the MotorState enum in src_c/pulse_mpc.h.
MOTOR_STATE_NAMES = {
    0: "STOPPED",
    1: "STARTING",
    2: "RUNNING",
    3: "STOPPING",
}


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV_PATH
    df = pd.read_csv(csv_path)

    fig, (ax_vel, ax_pos, ax_debt) = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

    ax_vel.set_title("Velocity: desired vs. commanded vs. simulated real")
    ax_vel.set_ylabel("rad/s")
    ax_vel.plot(df.time_s, df.desired_velocity, label="desired", color="tab:gray", linestyle="--")
    ax_vel.plot(df.time_s, df.command_velocity, label="commanded", color="tab:blue")
    ax_vel.plot(df.time_s, df.real_velocity, label="simulated real", color="tab:orange")
    ax_vel.legend(loc="upper right")

    ax_pos.set_title("Simulated position (FakeMotorPlant)")
    ax_pos.set_ylabel("rad")
    ax_pos.plot(df.time_s, df.position, color="tab:green")

    ax_debt.set_title("motionDebt, shaded by pass-through vs. small-velocity mode")
    ax_debt.set_xlabel("time (s)")
    ax_debt.set_ylabel("motionDebt (rad)")
    ax_debt.plot(df.time_s, df.motion_debt, color="tab:red")

    # Shade small-velocity (bang-bang) regions -- pass_through == 0.
    in_small_vel = df.pass_through == 0
    ax_debt.fill_between(
        df.time_s, ax_debt.get_ylim()[0], ax_debt.get_ylim()[1],
        where=in_small_vel, color="tab:red", alpha=0.1,
        label="small-velocity mode", step="post",
    )
    ax_debt.legend(loc="upper right")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
