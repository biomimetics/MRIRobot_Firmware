# TODO: finish the raw-observation / epoch-estimation switchover

Context: switching PulseMotorModelEstimator away from residuals (model -
observation) to raw PulseModelObservation output (per-field valid flags +
dud flag), pooled/tested/stepped in epochs by
PulseMotorModelEstimatorMixture_Bank. Also added dual onset detection
(position + velocity threshold), dud-pulse rejection, and per-pulse/
per-epoch logging to chase down the large variance and missing-pulse
problems seen on hardware (see chat: joints 3-6 showing suspiciously
tight, near-zero V_real residuals -- almost certainly noise-latched onsets
on pulses that never actually moved).

## Status of files already edited

- `src_c/pulse_motor_model.h` / `.c` — DONE. Added `PulseOnsetSource` enum,
  `PulseModelObservation` struct (per-field valid flags + `dud` +
  `onset_source` + `running_window_s` + `position_delta` +
  `planned_run_duration`), `PulseOnsetSource_Name`,
  `PulseMotorModel_ClampToPhysicalBounds` + `PULSE_MODEL_*` bound macros.
- `src_c/stats.h` / `.c` — DONE. Added `SampleStats_MeanDiffersFrom`
  (one-sample z-test) and `SampleStats_IsOutlier`.
- `PulseMotorModelEstimator.lf` — DONE (full rewrite). Emits
  `observation_output: PulseModelObservation` instead of
  `residual_output: PulseMotorModel`. Dual onset detection (position
  threshold + velocity threshold via `onset_velocity_fraction *
  model_.V_real.mean`), watched through STARTING/RUNNING/STOPPING (not just
  STARTING/RUNNING — recovers pulses whose real dead time exceeds the FSM's
  modeled T_start). Dud rejection via `dud_position_delta_factor`. Position-
  onset latency compensation (subtract `threshold/V` from the raw onset
  time). V_real quality floor `v_real_min_window_s` separate from the
  degenerate-math floor `min_running_duration_s`. Per-pulse outcome logging
  (`PRINT_PME_OBS`) + cumulative counters (duds, onset source counts, onset/
  tstop-missed, v_real-short-window, watchdog resets) printed every
  `PME_COUNTER_PRINT_INTERVAL` pulses. Takes `motor_model_input` now
  (applied immediately, not latched).
- `PulseMotorModelEstimatorMixture_Bank.lf` — MOSTLY DONE, one loose end
  (see below). Epoch-based test/step/broadcast logic over pooled per-field
  stats, dud/outlier exclusion, per-joint T_start shrinkage blend, epoch +
  heartbeat diagnostic printouts.

## Remaining work

1. **Fix `epoch_observation_count_` reset (bug — do this first).**
   In `PulseMotorModelEstimatorMixture_Bank.lf`, the epoch-close block ends
   by resetting `self->epoch_stats_` via `PulseMotorModelSampleStats_Init`
   but does **not** reset `self->epoch_observation_count_` back to 0. Since
   the epoch-close gate is `if (self->epoch_observation_count_ <
   self->epoch_min_samples) return;`, leaving it unreset means every epoch
   after the first closes immediately (count only ever grows). Add
   `self->epoch_observation_count_ = 0;` right next to the
   `PulseMotorModelSampleStats_Init(&self->epoch_stats_);` line at the very
   end of the `reaction(estimators.observation_output)` reaction.

2. **Update `PulseMotorModelEstimator_Bank.lf`'s port type.**
   This is the plain (non-mixture) per-joint-passthrough bank, kept for
   test harnesses. It still has (or had, as of last edit):
   ```
   output[7] observation_output: PulseMotorModel
   ```
   from an earlier intermediate step — needs to become
   `PulseMotorModel_Observation` → actually `PulseModelObservation` (match
   the struct name in pulse_motor_model.h) to match
   `PulseMotorModelEstimator.lf`'s new `observation_output:
   PulseModelObservation` port type. Also double check it has
   `input[7] motor_model_input: PulseMotorModel` wired through to
   `estimators.motor_model_input` (added to `PulseMotorModelEstimator` in
   this same round of changes) — grep for `motor_model_input` in that file
   to confirm the wiring line exists.

3. **`bank_index` parameter/state referenced in printfs doesn't exist yet.**
   `PulseMotorModelEstimator.lf`'s printf calls (both `PRINT_PME_DEBUG` and
   `PRINT_PME_OBS` blocks, and the watchdog printf) reference
   `self->bank_index` to tag log lines with the joint number, but **no such
   constructor parameter or state field was ever added to the reactor**.
   This will fail to compile. Need to either:
   - add `bank_index: int = 0` as a constructor parameter, set
     `self->bank_index_ = self->bank_index;` in `reaction(startup)` (state
     field, not the parameter itself, following this codebase's
     `foo_: type` convention), and have
     `PulseMotorModelEstimatorMixture_Bank.lf` /
     `PulseMotorModelEstimator_Bank.lf` instantiate with
     `new[7] PulseMotorModelEstimator(bank_index = ...)` — LF `new[n]`
     banks don't automatically give each instance a distinct parameter
     value, so check how `Small_DeltaP_Pulse_Controller_Bank.lf` numbers
     its instances (if it does) for the idiom already used in this repo; or
   - simplest fallback: drop `self->bank_index` from all the printfs in
     `PulseMotorModelEstimator.lf` and identify joints only at the
     `PulseMotorModelEstimatorMixture_Bank.lf` level (where the `for (int i
     = 0; i < estimators_width; i++)` loop already has `i` available for
     its own outlier-discard printf). This is the safer/faster fix if bank
     indexing isn't already a solved pattern elsewhere in the codebase.

4. **Full build verification (must do after 1-3).**
   ```
   cd vel_control_mri_arm
   export PATH="/home/linny/mri/MRIRobot_Firmware/resources/lingua-franca/bin:$PATH"
   make build        # lfc-dev codegen + cmake + cross-compile
   ```
   Confirm `bin*/Main.elf` (or wherever the linked output lands per the
   Makefile) is produced with no new warnings beyond the pre-existing
   `LF_TIME_BUFFER_LENGTH redefined` / `microstep_t` format-string noise
   already present before this change (grep those out when scanning
   output). If `make build` fails on the generator itself (as it briefly
   did earlier this session with a `NullPointerException: Cannot read
   field "children"`), that was caused by a zero-delay causality cycle
   introduced by wiring `pulse_motor_model_estimator.motor_model_output`
   back into `encoder_state_estimator.motor_model_input` and
   `deltap_bank.motor_model_update_input` — both connections in `Main.lf`
   already have `after 0` appended to break that cycle; if the NPE
   resurfaces, check whether the same fix needs to be reapplied or extended
   to a new feedback edge.

5. **Sanity-check `Main.lf` wiring still matches the renamed port.**
   `Main.lf` wires `pulse_motor_model_estimator.motor_model_output ->
   encoder_state_estimator.motor_model_input after 0` and `->
   deltap_bank.motor_model_update_input after 0` — those should be
   unaffected by the observation-type rename (they're `PulseMotorModel`,
   not `PulseModelObservation`), but re-grep `Main.lf` for
   `residual_output`/`observation_output` just in case any stale reference
   survived the earlier partial edits this session.

## Not yet started (originally discussed, lower priority than 1-5 above)

- Characterizing/tuning the new constructor defaults on real hardware:
  `dud_position_delta_factor` (2.5x), `onset_velocity_fraction` (0.5),
  `v_real_min_window_s` (0.05s), `field_min_samples` (15),
  `t_start_shrinkage_n0` (30). All currently best-guess placeholders per
  their doc comments.
- Velocity-onset detector's constant observer lag is NOT yet compensated
  (only the position detector's velocity-dependent latency is, via the
  `threshold/V` subtraction) — flagged as a TODO in
  `PulseMotorModelEstimator.lf`'s reactor-level comment.
- Once real hardware data comes back in: decide whether the position vs.
  velocity onset detectors agree closely enough to drop one, and whether
  the dud threshold actually kills the joint 3-6 near-zero-V_real cluster
  from the original bug report.
