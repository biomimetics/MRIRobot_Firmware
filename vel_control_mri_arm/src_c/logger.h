#ifndef LOGGER_H
#define LOGGER_H

#include <stddef.h>
#include "pulse_motor_model.h"

// =====================
// Log snapshot structs
// =====================
// Request/response payloads for Logger.lf -- one struct per reactor it
// polls (see Logger.lf's reactor-level comment for the request/response
// pattern). Sized around what's actually useful for spotting misbehaving
// axes on the receiving end, not a dump of every internal field.

// Snapshot of MotorCommandBiasEstimator_Bank.lf's running state.
typedef struct {
  float bias[7];          // per-joint smoothed command bias -- mirrors bias_output
  // Per-joint bias_stats_ running mean/sample-variance -- mirrors
  // reaction(heartbeat)'s stats_mean/stats_var print on
  // MotorCommandBiasEstimator.lf, so a misbehaving joint's raw estimate
  // spread is visible here too, not just the damped bias output above.
  float bias_mean[7];
  float bias_variance[7];
  int sequence;   // bumped every time this snapshot is rebuilt, so a
                  // consumer can tell a fresh snapshot from a stale repeat
} MotorCommandBiasLogSnapshot;

// Snapshot of PulseMotorModelEstimatorMixture_Bank.lf's running state.
typedef struct {
  PulseMotorModel model;  // current authoritative shared model
  // Pooled (all-joints) and per-joint long-horizon observation stats --
  // mirrors reaction(heartbeat)'s print_detailed_stats block, so a
  // persistent per-joint outlier (vs. just noisy single-epoch evidence)
  // is visible here too, same reasoning as that printout's comment.
  PulseMotorModelSampleStats pooled_stats;
  PulseMotorModelSampleStats per_joint_stats[7];
  int epoch_count;
  int converged_epochs;
  int sequence;
} PulseMotorModelLogSnapshot;

// Snapshot of EncoderStateEstimator.lf's running state -- one entry per
// joint straight from that joint's EncoderStateObserver (encoder_state_
// observer.h): the observer's own estimated position/velocity/disturbance
// states, plus last_innovation for judging how hard the innovation gate is
// having to work (see that field's comment on EncoderStateObserver).
typedef struct {
  float position[7];         // rad
  float velocity[7];         // rad/s
  float disturbance[7];      // rad/s^2
  float last_innovation[7];  // rad, PRE-clamp
  int sequence;
} EncoderStateLogSnapshot;

// =====================
// Log line formatting
// =====================
// One reactor's snapshot is printed as one or more logfmt-style lines --
// bare `source=`/`scope=` tokens act as a dictionary header, followed by
// key=value pairs for the actual data, e.g.:
//
//   [LOGGER] source=motor_bias seq=3 bias_0_mi=12 bias_0_mean_mi=10 bias_0_var_mi=2 ...
//
// Every token containing `=` is a plain logfmt key=value pair -- readable
// straight off a serial terminal, greppable by key, and mechanically
// convertible on the host into one YAML mapping per line (source/scope
// become the mapping's own key, the rest become its fields). Floats are
// logged as milli-unit ints, same convention as
// pulse_controller.c's PulseController_PrintDebugInfo (this platform's
// printf doesn't reliably support floats) -- see MILLI() in logger.c for
// the `_mi`-suffix naming convention that tells scripts/logger_yaml.py
// which keys need dividing back by 1000.0 and which (seq/epochs/
// converged/anything ending in `_n`) are already exact ints.
//
// Deliberately one line (one Logger_Format*Line call + one printf) per
// struct/joint rather than one giant assembled buffer: the full per-joint
// PulseMotorModelSampleStats dump alone is dozens of fields, and building
// that into a single buffer would need a buffer sized for the whole round
// at once. Each function below instead builds exactly one line into a
// caller-supplied buffer sized for just that line (LOGGER_LINE_BUFFER_SIZE
// is generous headroom for the widest one) -- see Logger.lf's
// reaction(log_ready) for how these are called in sequence, each with its
// own printf, wrapped in a [LOGGER START]/[LOGGER END] pair so a host-side
// reader knows when one round's lines are all in.

// Bytes needed for the widest single line these functions produce (a
// per-joint PulseMotorModelSampleStats dump, 6 fields x mean/var/n) plus
// headroom. snprintf never overflows the buffer regardless -- this is
// sized so a real round doesn't silently truncate.
#define LOGGER_LINE_BUFFER_SIZE 512

// Builds the motor_bias line (all 7 joints together -- small enough not to
// need splitting further, unlike the PulseMotorModelSampleStats lines
// below). NOT newline-terminated; returns the byte count written (the
// snprintf convention -- see each .c definition).
int Logger_FormatMotorCommandBiasLine(const MotorCommandBiasLogSnapshot *snap, char *buffer, size_t size);

// Builds the pulse_model line: the shared model itself plus epoch counters.
int Logger_FormatPulseMotorModelLine(const PulseMotorModelLogSnapshot *snap, char *buffer, size_t size);

// Builds the pulse_model_stats/scope=pooled line (all-joints long-horizon
// stats).
int Logger_FormatPulseMotorModelPooledStatsLine(const PulseMotorModelLogSnapshot *snap, char *buffer, size_t size);

// Builds the pulse_model_stats/scope=jointN line for joint_index (0..6).
int Logger_FormatPulseMotorModelJointStatsLine(const PulseMotorModelLogSnapshot *snap, int joint_index, char *buffer, size_t size);

// Builds the encoder_state line (all 7 joints together).
int Logger_FormatEncoderStateLine(const EncoderStateLogSnapshot *snap, char *buffer, size_t size);

#endif // LOGGER_H
