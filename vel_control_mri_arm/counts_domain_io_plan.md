# Counts-domain I/O change

## Why

The firmware currently takes command velocities in rad/s and reports encoder
feedback in rad and rad/s. Somewhere between the rad/s command and the motor,
the following conversion chain runs:

    rad/s command
      -> duty = speed / motor_configs[i]->pwm_rad_per_sec_max
      -> pwm_rad_per_sec_max = PWM_RPM_MAX * RPM_TO_RAD_PER_SEC
      -> PWM_RPM_MAX = 250 RPM * CPR_RATIO
      -> CPR_RATIO = (1440 * 4) / (REAL_CPR * INTERP * L2_CAL_FACTOR)

That chain exists to compensate for the Tekceleo drive's hardcoded assumption
about the encoder attached to it. It is not documented by the manufacturer, it
is per-joint (base joints use CPR_RATIO_BASE, elbow/wrist use CPR_RATIO), and
one of its terms (L2_CAL_FACTOR) is an admitted empirical fudge for a
"1.3-1.4x unexplained slowdown" on the 2000 CPR encoders.

We have observed different controller behavior with different external
encoders. This conversion chain is the prime suspect. This change removes it
from the command path so it can be ruled in or out.

## The two count spaces

Keeping these straight is the whole point of the change.

| Space | Counts/rev | Who defines it | Where it appears |
|---|---|---|---|
| **Real encoder counts** | `motor_configs[i]->qdec_cpr` (40000 base, 8000 elbow/wrist) | the physical encoder + FPGA | encoder feedback |
| **Manufacturer counts** | 1440 CPR x 4 interpolation = **5760** | hardcoded in the Tekceleo drive | duty cycle command |

The manufacturer's spec -- 100% duty cycle = 250 RPM -- is stated in the
manufacturer count space:

    250 RPM / 60 = 4.1667 rev/s
    4.1667 rev/s * 5760 counts/rev = 24000 counts/sec at 100% duty

## What changes

**Command direction (host -> firmware):** the host sends counts/sec in the
MANUFACTURER count space, as an int. `USM.lf` divides by a single hardcoded
24000 to get duty cycle. Nothing per-encoder touches the command path.

**Feedback direction (firmware -> host):** the firmware reports raw FPGA
counts and raw counts/sec in the REAL encoder count space, with no conversion
applied at all.

The host knows both CPRs and owns the mapping between the two spaces. This
lets it verify motor response against an exact counts/sec command instead of
trusting the firmware's rad/s -> duty conversion.

A consequence worth noting: because the command space no longer depends on
which encoder is bolted to a given joint, the per-motor `pwm_rad_per_sec_max`
variation (PWM_RAD_PER_SEC_MAX vs PWM_RAD_PER_SEC_MAX_BASE) collapses to one
global constant shared by all 7 motors. That collapse IS the change -- that
field only ever varied to compensate for the real encoder's CPR.

## Constant derivations

All derived from the manufacturer's spec (250 RPM @ 100% duty, 1440 CPR, 4x
interpolation), so all 7 motors share them:

    USM_ASSUMED_COUNTS_PER_REV        = 1440 * 4                = 5760
    PWM_COUNTS_PER_SEC_MAX            = (250/60) * 5760         = 24000
    MOTOR_MAX_SPEED_COUNTS_PER_SEC    = 12.566 rad/s equivalent = 11520  (2 rev/s)
    MOTOR_VELOCITY_MAX_CHANGE_COUNTS_PER_SEC = 0.5236 rad/s eq. = 480    (30 deg/s eq.)

The latter two are exact restatements of today's rad/s limits in the
manufacturer count space, so safety behavior is unchanged.

## File-by-file

1. **`src_c/common.h`**
   - Add the constants above.
   - Retype `EncoderStateMessage`: `int32_t` positions (counts), `float`
     velocities (counts/sec).
   - Mark the `CPR_RATIO` / `PWM_RAD_PER_SEC_MAX` block legacy -- it no longer
     feeds the live command path.

2. **`src/lib/Drivers/Encoder.lf`**
   - Output path emits `usm_counts_accepted[i]` directly as position, and the
     FPGA's Q20.11 estimate scaled only by `ENCODER_VELOCITY_SCALE` (which
     yields counts/sec natively) as velocity. No rad conversion.
   - Applies to BOTH `reaction(trigger)` (blocking path) and
     `reaction(encoder_rx_action)` (DMA path) -- they duplicate this logic.
   - Debug prints switch from mRad to counts.

3. **`src/lib/Drivers/USM.lf`**
   - New `convert_counts_per_sec_to_duty_cycle()` helper: a single divide by
     `PWM_COUNTS_PER_SEC_MAX`, deliberately independent of `qdec_cpr`.
   - `speed_[]` and the `speed` input become `int32_t`.

4. **`src/lib/State_Machine.lf`** -- types/units through the pipeline.

5. **`src_c/stm_comms.h` / `.c`** -- `CommandMessage`/`StateMessage` retype +
   field renames, `PROTOCOL_VERSION` 4 -> 5.

6. **Host tools** (`uart_comms_*.c`) -- keep the `make` targets building.

## Deliberate decisions

**Velocities stay `float` (counts/sec); positions become `int32_t`.** The
command is int as specified. But the FPGA hands us a fractional Q20.11
counts/sec estimate, and truncating it to int discards resolution for no gain
-- it is 4 bytes either way. Positions are genuinely integral, so they are
int32_t.

**The glitch filter keeps its rad conversions internally.**
`filter_channel_update`'s `ENCODER_ANOMALY_THRESHOLD_DEG_PER_SEC` (2 deg/s)
threshold is inherently per-encoder-CPR dependent. `qdec_convert_motor_to_rad_`
/ `qdec_convert_sea_to_rad_` stay alive purely to feed that check, so the
filter behaves bit-identically to today. Changing the I/O units and the glitch
filter's behavior in one commit would confound exactly the experiment this
change exists to run.

## Encoder diagnostics in StateMessage (same protocol version)

The FPGA already reports per-channel health that Encoder.lf parsed and then
threw away. StateMessage now carries it (`EncoderDiagnostics`, defined in
stm_comms.h and embedded in both StateMessage and common.h's
EncoderStateMessage so there is exactly one definition of the layout).

Contents, per channel, for all 7 USM and 7 SEA:

- `*_status` -- bit0 index correction applied, bit1 large discrepancy between
  index-implied and tracked position, bit2 index pulse seen.
- `*_illegal_transition_rejects`, `*_rate_limit_rejects` -- **saturating 4-bit
  counters (0-15)**. 15 means "15 or more", not exactly 15. They are
  qualitative "is this channel unhappy" signals; pinned-at-max is the
  meaningful reading. Do NOT compute rates from them.
- `*_reject_streak` -- FIRMWARE-side, not FPGA: 0 = the counts in this message
  are fresh for that channel, N > 0 = held for N cycles by Encoder.lf's glitch
  filter.
- `encoder_packets_ok` / `encoder_packets_bad` -- frame-level link health since
  boot. Watch the deltas.

**Raw, not filtered, by design.** The diagnostics come from the most recently
PARSED frame (`encoder_meta`), not the `*_meta_accepted` copies.
filter_channel_update discards a rejected sample's metadata, and a rejected
sample is exactly when its diagnostics matter -- the first bank-swap anomaly
correlated with rate_limit_rejects pinned at max on the glitching channel.
Consequence: the counts (filtered) and the diagnostics (raw) in one message can
describe different samples. `reject_streak` is what reconciles them.

Caveat: if a frame fails magic/CRC, `encoder_meta` holds the previous good
parse, so the per-channel fields go STALE rather than zeroing. The
packets_bad delta is how the host detects that.

Folded into PROTOCOL_VERSION 5 rather than minting a 6, since v5 had not been
deployed to either side when this was added -- one host migration, not two.

## Out of scope / follow-ups

- `USM_DAC.lf` is imported by no Main and still uses the rad/s path. Left
  alone; it needs the same treatment if revived.
- `stm_comms_test.c` is already stale (calls v3-era signatures, is not a
  Makefile target). Not revived here.
- The ROS-side copy of `stm_comms.h`
  (`MRIRobot_ROS/src/mri_arm/mri_arm_hardware/include/mri_arm_hardware/`) must
  be updated to match before hardware testing. The PROTOCOL_VERSION bump makes
  a stale host fail loudly rather than silently misread bytes.
- StateMessage grew 104 -> 168 bytes (174 on the wire). That still fits every
  buffer in the chain unchanged -- notably it is exactly UART_BUFFER_SIZE, and
  the LENGTH byte (which must express DATA+2) has room to 253. Anything much
  larger will need that framing revisited, not just the buffers.

## Hardware verification

The point of the change is that a counts/sec command should now be exactly
predictable:

    expected duty = commanded_counts_per_sec / 24000

Sweep a joint with `uart_motor_ramp`, log commanded counts/sec against the
reported real-encoder counts/sec, and convert with the known real CPR on the
host. If measured/commanded still varies by joint after that, the drive's
internal conversion is not the culprit and the search moves elsewhere.
