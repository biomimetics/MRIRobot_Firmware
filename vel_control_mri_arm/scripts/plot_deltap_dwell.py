#!/usr/bin/env python3
"""Plot the CSV produced by Main_Test_DeltaP_Dwell.lf (via DeltaPCsvLogger) --
the repeated triangle-with-dwell position target driving
Small_DeltaP_Controller, run through FakeMotorPlant so the plot shows the
simulated real motor response as it repeatedly closes on (and dwells at) a
moving target -- the small-velocity/remaining-error-convergence region
Main_Test_DeltaP.lf's one-shot step response barely exercises.

Produces a 3-panel figure: real target position (logged directly from
DeltaPSignalGenerator, not reconstructed) vs. simulated actual position,
remaining error + commanded velocity, and planned pulse run_duration (signed
by direction), shaded by pass-through vs. small-velocity mode.

target_position is only meaningful for CSVs written by
Main_Test_DeltaP_Dwell.lf -- DeltaPCsvLogger.lf's columns are shared with
Main_Test_DeltaP_Csv.lf (which has no signal generator), where this column
just reads 0 throughout. Earlier versions of this script reconstructed the
target as `position + remaining_error`, which silently smeared out the flat
dwell region: remainingError goes stale during a RUNNING pulse
(dpos_pulse_mpc.c only refreshes it at STOPPED), so that reconstruction
doesn't track the live target during a pulse.

---- Quickstart ----
    cd vel_control_mri_arm
    # one-time (or whenever dpos_pulse_mpc.c/h or Main_Test_DeltaP_Dwell.lf change):
    #   set `#define PRINT_DPOS 1` in src_c/dpos_pulse_mpc.h, then:
    lfc-dev src/Main_Test_DeltaP_Dwell.lf
    #   revert PRINT_DPOS back to 0 once built (only build-time matters)
    ./bin/Main_Test_DeltaP_Dwell
    python3 scripts/plot_deltap_dwell.py

This script itself needs no PRINT_DPOS/build step -- it just reads whatever
CSV Main_Test_DeltaP_Dwell.lf's last run produced (default path below).

Usage:
    python3 scripts/plot_deltap_dwell.py [path/to/csv]
"""

import sys

import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_CSV_PATH = "csv_data/deltap_dwell_log.csv"

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

    fig, (ax_pos, ax_err, ax_pulse) = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

    ax_pos.set_title("Target vs. simulated actual position")
    ax_pos.set_ylabel("rad")
    ax_pos.plot(df.time_s, df.target_position, label="target", color="tab:gray", linestyle="--")
    ax_pos.plot(df.time_s, df.position, label="simulated actual", color="tab:green")
    ax_pos.legend(loc="upper right")

    ax_err.set_title("Remaining error vs. commanded velocity")
    ax_err.set_ylabel("rad / rad/s")
    ax_err.plot(df.time_s, df.remaining_error, label="remaining error (rad)", color="tab:red")
    ax_err.plot(df.time_s, df.command_velocity, label="commanded velocity (rad/s)", color="tab:blue", alpha=0.6)
    ax_err.axhline(0.0, color="tab:gray", linestyle=":", linewidth=1)
    ax_err.legend(loc="upper right")

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
