#include "logger.h"
#include <stdio.h>

// Milli-unit int conversion for printf -- same convention as
// pulse_controller.c's PulseController_PrintDebugInfo (this platform's
// printf doesn't reliably support floats).
//
// Every key for a value built with MILLI() below gets an `_mi` suffix
// (added right in each format string, e.g. "bias_%d_mi=%d") -- a naming
// convention scripts/logger_yaml.py knows about: it strips `_mi` and
// divides by 1000.0 to hand back a real float, so the host-side YAML
// shows actual units instead of milli-scaled ints. Every OTHER key (seq,
// epochs, converged, index, ts, and anything ending in `_n`) is already an
// exact int/count and deliberately carries no `_mi` suffix, so it passes
// through unscaled.
#define MILLI(x) ((int) ((x) * 1000.0f))

// Appends one SampleStats' mean/sample-variance/count as `<name>_mean_mi=
// ... <name>_var_mi=... <name>_n=...` -- shared by
// Logger_FormatPulseMotorModelPooledStatsLine/
// Logger_FormatPulseMotorModelJointStatsLine below so the 6-field dump
// (V_min_cmd/V_real_mean/V_real_variance/T_start/T_stop/minimumPulseWidth)
// isn't repeated by hand in each. Same `written +=`-into-remaining-space
// idiom as stm_comms.c's serialize_state_message_csv.
static int AppendSampleStatsField(char *buffer, size_t size, int written, const char *name, const SampleStats *s) {
  written += snprintf(buffer + written, size - written, "%s_mean_mi=%d %s_var_mi=%d %s_n=%lu ",
      name, MILLI(SampleStats_Mean(s)),
      name, MILLI(SampleStats_SampleVariance(s)),
      name, (unsigned long) SampleStats_Count(s));
  return written;
}

// Shared by Logger_FormatPulseMotorModelPooledStatsLine/
// Logger_FormatPulseMotorModelJointStatsLine -- both dump the same 6
// SampleStats fields off a PulseMotorModelSampleStats, just a different one
// (pooled_stats vs. per_joint_stats[i]).
static int AppendPulseMotorModelSampleStats(char *buffer, size_t size, int written, const PulseMotorModelSampleStats *stats) {
  written = AppendSampleStatsField(buffer, size, written, "vmin", &stats->V_min_cmd);
  written = AppendSampleStatsField(buffer, size, written, "vrealmean", &stats->V_real_mean);
  written = AppendSampleStatsField(buffer, size, written, "vrealvar", &stats->V_real_variance);
  written = AppendSampleStatsField(buffer, size, written, "tstart", &stats->T_start);
  written = AppendSampleStatsField(buffer, size, written, "tstop", &stats->T_stop);
  written = AppendSampleStatsField(buffer, size, written, "minpw", &stats->minimumPulseWidth);
  return written;
}

int Logger_FormatMotorCommandBiasLine(const MotorCommandBiasLogSnapshot *snap, char *buffer, size_t size) {
  int written = 0;
  written += snprintf(buffer + written, size - written, "[LOGGER] source=motor_bias seq=%d ", snap->sequence);
  for (int i = 0; i < 7; i++) {
    written += snprintf(buffer + written, size - written, "bias_%d_mi=%d bias_%d_mean_mi=%d bias_%d_var_mi=%d ",
        i, MILLI(snap->bias[i]), i, MILLI(snap->bias_mean[i]), i, MILLI(snap->bias_variance[i]));
  }
  return written;
}

int Logger_FormatPulseMotorModelLine(const PulseMotorModelLogSnapshot *snap, char *buffer, size_t size) {
  int written = 0;
  written += snprintf(buffer + written, size - written,
      "[LOGGER] source=pulse_model seq=%d vmin_mi=%d vreal_mean_mi=%d vreal_var_mi=%d tstart_mi=%d tstop_mi=%d minpw_mi=%d epochs=%d converged=%d ",
      snap->sequence,
      MILLI(snap->model.V_min_cmd),
      MILLI(snap->model.V_real.mean),
      MILLI(snap->model.V_real.variance),
      MILLI(snap->model.T_start),
      MILLI(snap->model.T_stop),
      MILLI(snap->model.minimumPulseWidth),
      snap->epoch_count,
      snap->converged_epochs);
  return written;
}

int Logger_FormatPulseMotorModelPooledStatsLine(const PulseMotorModelLogSnapshot *snap, char *buffer, size_t size) {
  int written = 0;
  written += snprintf(buffer + written, size - written, "[LOGGER] source=pulse_model_stats scope=pooled seq=%d ", snap->sequence);
  written = AppendPulseMotorModelSampleStats(buffer, size, written, &snap->pooled_stats);
  return written;
}

int Logger_FormatPulseMotorModelJointStatsLine(const PulseMotorModelLogSnapshot *snap, int joint_index, char *buffer, size_t size) {
  int written = 0;
  written += snprintf(buffer + written, size - written, "[LOGGER] source=pulse_model_stats scope=joint%d seq=%d ", joint_index, snap->sequence);
  written = AppendPulseMotorModelSampleStats(buffer, size, written, &snap->per_joint_stats[joint_index]);
  return written;
}

int Logger_FormatEncoderStateLine(const EncoderStateLogSnapshot *snap, char *buffer, size_t size) {
  int written = 0;
  written += snprintf(buffer + written, size - written, "[LOGGER] source=encoder_state seq=%d ", snap->sequence);
  for (int i = 0; i < 7; i++) {
    written += snprintf(buffer + written, size - written, "pos_%d_mi=%d vel_%d_mi=%d dist_%d_mi=%d innov_%d_mi=%d ",
        i, MILLI(snap->position[i]), i, MILLI(snap->velocity[i]), i, MILLI(snap->disturbance[i]), i, MILLI(snap->last_innovation[i]));
  }
  return written;
}
