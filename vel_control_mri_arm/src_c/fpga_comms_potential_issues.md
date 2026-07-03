# FPGA <-> STM32 encoder comms: open issues

Findings from debugging the SEA/USM burst-readout protocol (`mri_encoder_reader.v`
-> `Encoder.lf`). One bug is already fixed upstream; one is open and currently
worked around STM32-side. Written up here so the FPGA side can be fixed properly
later without re-deriving all of this from scratch.

## Fixed: shift_counter14 fire-reset could be silently dropped

`qdec_channel_bank.v`'s burst-readout counter used to gate its register's clock
enable with `en` only:

```verilog
REGISTER_R_CE #(.N(4), .INIT(4'b1111)) counter_reg (
    .q(counter), .d(count_next),
    .rst(reset), .clk(clk), .ce(en)   // <-- was just `en`
);
```

Since `count_next = fire ? 0 : ...` is only ever *captured* on a cycle where
`ce` is high, a `fire` pulse landing on a cycle where `en` (== `out_ready`,
downstream FIFO readiness) happened to be low was silently lost -- the counter
never left its idle sentinel, the channel bank never produced a burst, and the
STM32 saw the read time out with zero bytes ever arriving. This looked like
"the FPGA isn't responding at all," and only ever "worked" when a fire happened
to coincide with `en` already being high, which explained the intermittent
"works once then wedges" behavior we saw early on.

**Status: fixed** -- current source has `.ce(en || fire)`, which unconditionally
captures the reset regardless of `en`'s state that cycle. Confirmed working:
SEA/USM fires now reliably produce full bursts.

## Open: channel_index is consistently rotated by a fixed offset

With framing (COBS decode) and CRC-8 both verified 100% reliable (see below),
each metadata word's embedded `channel_index` self-check byte does not match
the record's position in the burst. The mismatch is not random -- it's a
constant rotation, confirmed against ~230-250 samples each:

- SEA: **232/232** records show `channel_index == (record_position + 4) mod 7`
- USM: **242/243** show the same `+4` offset (one single outlier at `+6`,
  likely an unrelated one-off bit error, not part of the pattern)

### Why this probably means the DATA is rotated too, not just the label

In `qdec_channel_bank.v`:

```verilog
generate
    for (ch = 0; ch < 7; ch = ch + 1) begin : qdec_ch
        qdec_channel #(...) QDEC (
            ...
            .count(count_arr[ch]), ...
        );
    end
endgenerate

generate
    for (ch = 0; ch < 7; ch = ch + 1) begin : metadata_pack
        assign metadata_arr[ch] = {velocity_arr[ch], ..., ch[2:0]};
    end
endgenerate
...
assign count = (cntr_val[0] == 1'b0) ? count_arr[cntr_val[3:1]] :
                                        metadata_arr[cntr_val[3:1]];
```

`count_arr[ch]` and `metadata_arr[ch]` are populated using the *same*
generate-loop `ch`, and read out via the *same* `cntr_val[3:1]` index one
cycle apart (count word, then that channel's metadata word). Whatever
mechanism is rotating the readout, it necessarily rotates a record's count
word and its self-check label together -- they can't desync from each other
independently, since they're indexed identically. That means `channel_index`
should still correctly identify which physical channel a record's *data*
(count + velocity + diagnostics) belongs to, even when its position in the
burst doesn't match.

This can't be visually confirmed yet, though: all channels currently read 0
(encoders at rest), so a rotation of all-zero data across channels is
indistinguishable from no rotation at all. Worth re-checking once encoders are
actually turning and channels have distinguishable values.

### What's been ruled out (static read-through, not simulated)

- **`shift_counter14`'s fire-reset**: unconditionally captures `count_next=0`
  when `fire=1` (now that `ce=en||fire`), so a plain missed/delayed reset
  doesn't explain a *stable* `+4` offset by itself.
- **`Queue.sv`** (the 64-bit FIFO between `qdec_arbiter` and the serializer):
  standard circular-buffer enqueue/dequeue pointer logic, combinational read
  at `deq_ptr_value` -- no reordering mechanism visible.
- **`VectorToSingle.sv`** (64-bit -> 8-bit serializer): already has a
  hand-written comment/fix (`_GEN_2_consumed` gating) for a previously-found
  "last byte of burst silently dropped" bug. Byte order within one 64-bit word
  is LSB-first (`_GEN_0[counter[2:0]]`, counter=0 -> `data[7:0]`), matching
  `read_fpga_example.c` section 4 and the STM32 decode -- doesn't reorder
  across words.
- **`qdec_arbiter.v`**: simple priority mux (`out_ready1 = out_ready &&
  !out_valid0`), passes through whichever bank is active; doesn't interleave
  or reorder within one bank's burst.

None of these individually explain a clean, constant `+4` (== `+8` words)
rotation. Possible remaining leads, best pursued with a simulation/waveform
capture rather than more static reading:

- Exact cycle-by-cycle relationship between the STM32's fire byte arriving,
  `SEA_Block_fire`/`USM_Block_fire` asserting, and `shift_counter14` actually
  resetting -- is `fire` really a clean single-cycle pulse, or does
  `comm_blk.v`'s `STM_recv_valid` hold longer than expected?
  (`STM_data == 8'h04` is a *level* comparison against whatever
  `comm_blk.v` currently reports as the last received byte, not an edge --
  worth checking `uart_receiver.v`'s `data_out_valid` pulse width directly.)
- Queue occupancy (`io_count`) sampled right as a new fire arrives -- is
  there ever leftover unconsumed data from the previous burst still sitting
  in the FIFO when qdec_channel_bank starts enqueuing a fresh one, e.g. if
  packet_framer's DRAIN phase and the FPGA-side "ready for next fill" state
  transition slightly later than when the STM32 believes the previous frame
  is fully consumed (it only waits for the 0x00 delimiter to arrive)?
- Whether the offset is truly pinned at `+4` forever after each FPGA
  power-up/reset, or drifts over a longer session (only ~3-4s of continuous
  capture has been checked so far).

### Current STM32-side mitigation (Encoder.lf)

`parse_encoder_group()` now places each record using its embedded
`channel_index` (bounds-checked against `ENCODER_CHANNELS_PER_GROUP`, with any
out-of-range value logged and dropped) rather than its position in the burst,
and logs (throttled) whenever a record's `channel_index` doesn't match its
position, so the rotation stays visible without spamming. This is a
short-term patch, not a fix -- it depends on the "label and data rotate
together" argument above holding, which is a reasonable inference from the
RTL but not simulation-verified. Once the FPGA-side root cause is
understood/fixed, this should probably revert to trusting word position
(simpler, and matches `read_fpga_example.c`'s documented design of
`channel_index` as self-check-only).

### Update (2026-07-02, after on-robot retest): reversal alone doesn't fix it

Applied `channel_index`-based placement (above) plus a new
`ENCODER_REVERSE_CHANNEL_ORDER` flag in `Encoder.lf` that mirrors the
resulting index (`dest_index = 6 - channel_index`), on the theory that the
whole joint order was simply flipped end-to-end. On-robot retest: **still
wrong** -- joints are not in the correct order even with the mirror applied.

This is important new evidence against the working theory so far. A few
ways to read it:

- The "record's count and its self-check label rotate together, so
  `channel_index` reliably identifies the true physical channel" argument
  (above) may be incomplete or wrong -- e.g. if the actual bug is further
  upstream than `qdec_channel_bank.v` (in the `A_qdec`/`B_qdec`/`I_qdec`
  physical pin slicing per channel, or in the cable harness itself), then
  `channel_index` would faithfully report the *array index* the FPGA thinks
  it's using, but that array index might never have corresponded to the
  physical joint we assumed in the first place -- no software-side
  relabeling of array indices would fix a wiring-level swap.
- The true joint <-> channel mapping may not be a simple uniform rotation
  or mirror at all. Both `ENCODER_REVERSE_CHANNEL_ORDER` and the `+4`
  rotation fix are *global, formula-based* corrections (same permutation
  applied to every channel); if the real cause is e.g. a harness where
  individual connector pairs got swapped pairwise, or channels were wired
  up in an order that doesn't follow any simple arithmetic pattern, no
  single formula will produce the right order for all 7 channels at once --
  fixing one pair can easily still leave others wrong, which would look
  exactly like what was observed ("still some issues," not "completely
  wrong" -- consistent with a partial improvement plus remaining
  mismatches).

**Before trying another formula**: get ground truth. Move one physical joint
at a time (with the others still) and record which reported channel index
(in `sea_counts`/`usm_counts`/`qdec_out`/`sea_out`, with `PRINT_ENCODER` on)
actually changes. Repeat for all 7 SEA and all 7 USM channels. That gives an
explicit joint-index -> reported-index table, which may or may not match any
clean formula -- if it doesn't, the fix on the STM32 side should be an
explicit lookup table (`static const uint8_t sea_channel_map[7] = {...}`)
rather than continuing to guess at rotations/mirrors, and the FPGA/harness
side still needs the real root cause identified separately.

### Update (2026-07-02, motorized ground-truth calibration): motors 0-3 clean, 4-6 inconclusive; reversal flag's effect unclear

Built `MotorCalibration.lf` (temporary reactor, see that file and the
temporarily-repointed wiring in `Main.lf`) to automate the ground-truth
capture above instead of moving joints by hand: pulses each motor 0-6 one at
a time at a small constant velocity for a short burst, diffing
`qdec_out`/`sea_out` immediately before/after each pulse.

First pass (5 deg/s, 0.2s) produced no measurable movement on any motor --
too weak/short to read a clear signal. Second pass (20 deg/s, 0.5s), with
`ENCODER_REVERSE_CHANNEL_ORDER=1` still active from the earlier mirror fix,
gave:

| motor driven | qdec (USM) index that moved | delta [mRad] | confidence |
|---|---|---|---|
| 0 | 6 | -976 | clear |
| 1 | 5 | -1199 | clear |
| 2 | 4 | -1040 | clear |
| 3 | 3 | 55 | clear but small |
| 4 | none | 0 everywhere | **no signal** |
| 5 | none | 0 everywhere | **no signal** |
| 6 | none | 0 everywhere | **no signal** |

SEA deltas were tiny (2-3 mRad) and not clearly attributable to any single
channel for any motor -- likely cross-talk/vibration from the driven motor
bleeding into other joints' SEA sensors rather than genuine per-channel
signal. SEA channel mapping is still unconfirmed.

Motors 0-3 show a clean, unambiguous pattern: **motor N maps to qdec index
`6 - N`** -- i.e. a full mirror. Motors 4-6 produced no measurable USM delta
at all even at 20 deg/s/0.5s; they may have different `pwm_rad_per_sec_max`
scaling than 0-2 (see `motor_config.h`) such that the same commanded rad/s
is a much smaller fraction of their max speed / weaker PWM duty cycle, too
weak to overcome static friction in 0.5s. Need a stronger and/or longer
pulse for 4-6 specifically before their mapping can be read.

**Open question, not yet resolved**: this `6-N` mirror is showing up *with*
`ENCODER_REVERSE_CHANNEL_ORDER=1` already active. That flag was added
specifically to undo a mirror already reported from on-robot testing (motor
0 appearing as joint 6, etc.) -- if it were doing what it was designed to
do, motor 0 driving should now show up at qdec index 0, not 6. Since it's
still landing on 6, either the flag isn't taking effect the way intended, or
the original "0 mapped to 6" observation was made through a different
signal path (e.g. downstream through `State_Machine.lf`/ROS, which may have
its own independent DOF reindexing) rather than through the raw
`qdec_out`/`sea_out` values this calibration reactor measures directly.
Needs to be reconciled before touching the reversal flag again.

### Update (2026-07-02, resolved): mapping is identity; the reversal flag was the bug, not the fix

Re-ran the motorized calibration at a much clearer signal level -- 6% duty
cycle (per-motor, computed as `motor_configs[i]->pwm_rad_per_sec_max * 0.06`
so every motor gets a comparably strong pulse regardless of its own
`pwm_rad_per_sec_max` scaling -- below ~5% duty these motors are
electrically inconsistent) for 1s per motor, 1.5s rest between. Also cut
`Encoder.lf`'s debug print rate from ~66Hz to ~10Hz (`printcnt_lim` 5 -> 33)
so capture logs stayed a manageable size at this pulse length.

**With `ENCODER_REVERSE_CHANNEL_ORDER=1` (as it was)**: motor N cleanly
moved channel `6-N` for all 7 motors (0-3 via `qdec`, 4-6 via `sea` -- their
`qdec`/USM signal apparently doesn't register within 1s at this duty cycle,
but the mirror completes cleanly through `sea` instead). A clean mirror
riding on top of an already-mirrored signal would show up as *identity*, not
another mirror -- so this result means the underlying (unreversed)
`channel_index` mapping was already correct, and the reversal flag was
introducing the backwards mapping, not correcting one.

**Confirmed**: flipped `ENCODER_REVERSE_CHANNEL_ORDER` to `0` and re-ran.
Clean identity mapping for all 7 motors:

| motor | channel that moved | delta [mRad] |
|---|---|---|
| 0 | qdec[0] | -1781 |
| 1 | qdec[1] | -2424 |
| 2 | qdec[2] | -2858 |
| 3 | qdec[3] | 10257 |
| 4 | sea[4] | 191 |
| 5 | sea[5] | 169 |
| 6 | sea[6] | 151 |

This run also directly caught the compliance cross-talk that likely produced
the original "0 mapped to 6" report: motor 0's pulse *also* moved
`sea[4]`/`sea[5]`/`sea[6]` by -194/-175/-154 mRad -- driving the base joint
visibly swings the whole downstream arm including the wrist. Watching the
wrist while the base moved would look exactly like "channel 0 is joint 6."
Likely explanation for the original confusion, not a separate bug.

**Resolution**: `ENCODER_REVERSE_CHANNEL_ORDER = 0` in `Encoder.lf`. The
`channel_index`-based placement fix (correcting the FPGA's `+4` rotation,
see above) was sufficient on its own -- no additional reversal/mirror
needed. `MotorCalibration.lf` and its temporary wiring in `Main.lf` should
be reverted back to normal (`state_machine` -> `usm`) now that the mapping
is confirmed; kept the reactor file around in case the mapping needs
re-verifying after any future FPGA-side fix to the `+4` rotation bug itself.

## Open: single-sample position anomalies (sign flips / large spurious jumps)

Added a position-anomaly check to `Encoder.lf`'s trigger reaction
(`check_position_anomaly()`, 2026-07-02): compares each channel's raw count
to its value one sample ago, and prints an unthrottled alert if the implied
speed exceeds 2 deg/s (well below anything these motors do on purpose over
one ~10ms sample, so with the arm at rest this should only catch genuine
glitches). Deliberately looks at raw position deltas independent of the
FPGA's own velocity_q20_11 estimate, since position and velocity are
computed independently on the FPGA side and don't always glitch together.

First real captures show **two distinct mechanisms**, both confirmed via a
frame that passed CRC-8 (i.e. these are real FPGA-computed values, not
UART/parsing corruption):

1. **Rate-limiter saturation.** The glitching channel's `rate_limit_rejects`
   metadata field was pinned at 255 (max) in the same burst as a huge
   spurious count jump (e.g. USM channel 1 jumped by -56467 counts in one
   ~10ms sample, channel 2 by -34551, both with `rate_reject=255`; every
   other channel in the same burst read 0 or 1). That counter increments
   when the FPGA's rate limiter rejects a quadrature step for arriving
   faster than `QDEC_MIN_STEP_INTERVAL_CYCLES` allows -- consistent with a
   burst of implausibly-fast, likely electrically-noisy transitions on that
   channel's A/B lines (not real motion; 56467 counts in 10ms would be
   ~5.6M counts/sec).
2. **Index-correction jump.** A separate capture showed channels 3 and 4
   jump with `rate_reject=0` but `index_status=6` (bit1 "large discrepancy
   detected" + bit2 "index pulse seen" both set) -- i.e. the FPGA's own
   index-correction logic firing at an index-pulse event and applying a
   large correction. Whether that correction is itself buggy (wrong
   expected position, bad discrepancy threshold) or correctly reporting a
   real large discrepancy is not yet known.

The anomaly alert now includes `rate_reject`/`illegal`/`index_status`/
`velocity_q20_11` directly (previously required a lucky coincidental
raw-burst dump on the same cycle to correlate) so future occurrences are
self-contained in the log.

### Update (2026-07-02, dashboard cross-check): the "spikes" are drops to zero, not sign flips

The user re-checked behavior after a fresh flash using a separate
higher-level monitoring dashboard (position/velocity/SEA-deflection plots
over time, sourced downstream of `Encoder.lf`). What originally looked from
the raw `Encoder.lf` prints like a same-magnitude sign flip (e.g. -X jumping
to +X or a large unrelated value) is, per that plot, actually the position
value being **erroneously set to zero** for a single sample and then
recovering back to its real value -- a single-sample dropout-to-zero glitch,
not a sign-bit corruption or an arbitrary large excursion. Visually this
shows up as a sharp spike toward zero on an otherwise-flat or slowly-varying
trace (M1's position plot in particular: a series of narrow spikes jumping
from a steady ~-380 deg baseline up toward 0/positive values and back).
This is a meaningfully different failure signature than "sign flip" implies
-- worth keeping in mind if/when tracing this in simulation: look for
whatever could force a position register (or a word in the burst pipeline)
to 0 for exactly one readout, rather than for a bit-flip in a sign bit.

### Update (2026-07-02, pose-dependent reproduction): the whole USM burst occasionally becomes an exact copy of SEA's data

The user moved the robot into a specific configuration where the anomaly
became frequent (168 alerts in a 10s capture, vs. 0 in an earlier 15s
capture at a different pose) -- reproducible-by-pose, not random. That
volume of data revealed a much more specific pattern than "per-channel
noise": across many consecutive triggers, `usm_counts[]` doesn't drift or
spike independently per channel -- **all 7 channels flip together, in
lockstep, to values that exactly match SEA's own current reading**, then
revert to USM's real values the next cycle. Confirmed directly: the
separately-throttled `SEA counts:`/`USM counts:` summary prints stayed
completely constant for the whole capture (robot at rest in this pose) at
`SEA counts: -98 -52 161 -230 4790 -14784 -1611` and
`USM counts: -38416 68736 81212 14840 0 13 -18` -- and the anomaly alerts'
"new" values for a "glitching" cycle are exactly SEA's vector, while the
immediately following cycle's alert shows USM's real vector coming back.
This means what looked like independent per-channel corruption is actually
whole-burst-scale: **for one cycle, the response to a USM fire is an exact,
valid (CRC-8-passing) copy of SEA's data**, not noise on individual
channels.

I initially guessed this might be command-byte corruption on the STM32->FPGA
link (fire bytes are `0x04`/`0x05`, one bit apart, and that direction has no
CRC unlike the FPGA->STM32 burst response) -- **the user pushed back on
this and I don't have real evidence for it**: no history of single-bit
corruption between these boards, and this was reproduced in a low-EMI test
environment. Retracting that as the leading theory. The pose-dependent
reproducibility (a specific robot configuration makes it frequent, not
random timing) points more toward something FPGA-internal and
data/state-dependent -- e.g. a race in `qdec_arbiter`'s bank-priority
handshake, or something in `packet_framer`'s single-buffered FILL/DRAIN
state machine that can occasionally re-present the previous (still-buffered)
payload instead of accepting a fresh one -- but neither is confirmed. Not
all glitching channels in this capture showed `index_status=6` (only 2 of 7
did, e.g. channels 4/5 in one sample), so the earlier "index-correction
jump" theory may be a real but separate/coincidental phenomenon rather than
the cause of this whole-burst swap.

**Still needed**: FPGA-side simulation/waveform tracing of `qdec_arbiter`'s
`out_valid0`/`out_ready0`/`out_ready1` and `packet_framer`'s state around a
USM fire that immediately follows a SEA fire, to find the actual mechanism
by which one bank's data can end up substituting for the other's for a
single burst.

### Worked example: exact behavior of the whole-burst bank swap

This section is meant to stand alone -- everything needed to understand the
bug is below, without needing the rest of this file.

**Background.** The STM32 talks to an FPGA over a single UART. Each ~10ms
cycle, `Encoder.lf`'s trigger reaction does, in order: (1) send a single
byte `0x04` ("fire SEA"), then blocking-read the FPGA's response -- a
COBS-framed, CRC-8-protected packet containing 7 channels' worth of SEA
(series elastic actuator, one per joint) position data; (2) send a single
byte `0x05` ("fire USM"), then blocking-read the same kind of framed
response, this time containing 7 channels' worth of USM (motor shaft
encoder) position data. SEA and USM are two independent counter banks in
the FPGA (`SEA_Block` / `USM_Block` in `mri_encoder_reader.v`), each
covering a disjoint set of physical encoder pins -- there is no shared
hardware between them below the arbiter/serializer level (confirmed via
pin-constraint diffing, see below). Every parsed count is logged after each
successful read as `Encoder.lf: SEA counts: ...` / `Encoder.lf: USM
counts: ...` (throttled to ~10Hz), and a separate, unthrottled check
compares each channel's *this-cycle* count to its *previous-cycle* count and
prints an alert (`!!! POSITION ANOMALY !!!`) if the implied speed between
the two samples exceeds 2 deg/s.

**What was captured.** With the robot held stationary in a specific pose,
the following was logged over several seconds (real excerpt, reformatted
for clarity -- the actual log lines are single lines each):

```
Encoder.lf: SEA counts: -98 -52 161 -230 4790 -14784 -1611
Encoder.lf: USM counts: -38416 68736 81212 14840 0 13 -18
```

These two lines repeat, byte-for-byte identical, for the entire capture --
the robot is not moving, so both banks' real values are constant. Call
these SEA_BASELINE and USM_BASELINE.

In between those throttled summary prints, the per-cycle anomaly check
fired on *every one of the 7 USM channels simultaneously*, twice in a row
(a "glitch" cycle followed by a "revert" cycle), like this:

```
--- cycle N (glitch) ---
POSITION ANOMALY USM ch0: prev=-38416 new=-98    delta=38318  ...
POSITION ANOMALY USM ch1: prev=68736  new=-52    delta=-68788 ...
POSITION ANOMALY USM ch2: prev=81212  new=161    delta=-81051 ...
POSITION ANOMALY USM ch3: prev=14840  new=-230   delta=-15070 ...
POSITION ANOMALY USM ch4: prev=0      new=4790   delta=4790   ...
POSITION ANOMALY USM ch5: prev=13     new=-14784 delta=-14797 ...
POSITION ANOMALY USM ch6: prev=-18    new=-1611  delta=-1593  ...

--- cycle N+1 (revert) ---
POSITION ANOMALY USM ch0: prev=-98    new=-38416 delta=-38318 ...
POSITION ANOMALY USM ch1: prev=-52    new=68736  delta=68788  ...
POSITION ANOMALY USM ch2: prev=161    new=81212  delta=81051  ...
POSITION ANOMALY USM ch3: prev=-230   new=14840  delta=15070  ...
POSITION ANOMALY USM ch4: prev=4790   new=0      delta=-4790  ...
POSITION ANOMALY USM ch5: prev=-14784 new=13     delta=14797  ...
POSITION ANOMALY USM ch6: prev=-1611  new=-18    delta=1593   ...
```

**How to read this.** Look at cycle N's 7 "new" values as a vector:
`(-98, -52, 161, -230, 4790, -14784, -1611)`. That is *exactly*
SEA_BASELINE, element for element. Cycle N+1's "new" values are
`(-38416, 68736, 81212, 14840, 0, 13, -18)` -- exactly USM_BASELINE. So for
one single cycle (N), whatever the STM32 fired `0x05` (USM) and received
back was, byte-for-byte, SEA's data -- not corrupted, not random, not
"close to" SEA's data, but an *exact* copy of it. The very next cycle, USM's
real data is back. This is not visible in the throttled `USM counts:`
summary line at all, because that line is only printed roughly every 10th
cycle and essentially never happens to land on the 1-cycle-wide glitch --
it always shows the settled/correct value. The per-cycle anomaly check
(which runs on literally every cycle) is what catches it.

**What this rules in/out:**
- Not random noise or a bit error: the "wrong" value isn't garbage, it's
  precisely another valid, currently-true reading from the other bank.
- Not a UART/parsing bug on the STM32 side: the frame that produced the
  "glitch" cycle passed its CRC-8 check (see `encoder_decode_and_verify()`
  in `Encoder.lf`) -- whatever the FPGA sent was self-consistent and
  received correctly.
- Not a physical wiring/pin-mapping issue: SEA and USM channels are wired
  to disjoint bits of the same 14-bit bus (`A_qdec[6:0]` for SEA,
  `A_qdec[13:7]` for USM in `mri_encoder_reader.v`), confirmed via a diff of
  the current project's pin-constraint file (`Arty-A7-35-Master.xdc`)
  against the last-known-working version -- every `IO_A`/`IO_B`/`IO_I`
  per-pin line is byte-for-byte identical between the two, and SEA/USM
  channel 4 land on physically distinct FPGA pins.
- Pose-dependent, not purely random-timing: this pattern was rare in one
  robot configuration and happened 168 times in a 10-second window in
  another. That points at something FPGA-internal that depends on encoder
  state/timing (a specific bank being fired again while its OWN previous
  reading is still somehow "live"/selected by the arbiter, for example --
  see `qdec_arbiter.v`'s `out_ready1 = out_ready && !out_valid0` priority
  logic and `packet_framer.v`'s single-buffered FILL/DRAIN state machine as
  starting points), not something that should be equally likely regardless
  of what the robot is doing.

**What's needed to actually fix this**: simulation or waveform capture of
the FPGA internals (`qdec_arbiter`'s `out_valid0`/`out_ready0`/`out_ready1`,
and `packet_framer`'s state machine) around the transition from a SEA fire
to an immediately-following USM fire, specifically in a pose known to
reproduce this frequently, to find the actual mechanism that lets one
bank's already-read data get delivered again in response to a fire for the
*other* bank.

### Update (2026-07-02, STM32-side mitigation added): reject-and-hold filter with an auto-recovery streak limit

Added `filter_channel_update()` to `Encoder.lf`, replacing the previous
alert-only `check_position_anomaly()`. Per channel, per group (SEA/USM
independently), it compares each freshly-parsed reading against the last
*accepted* value (not just the last-received one) using the same >2 deg/s
implied-speed threshold as before. Two new arrays,
`sea_counts_accepted`/`usm_counts_accepted` (+ matching `..._meta_accepted`
structs), hold what's actually fed to `qdec_out`/`sea_out`/`qdec_vel_out`/
`sea_vel_out` downstream -- `sea_counts`/`usm_counts` (raw, whatever was
just parsed) are left alone as before.

- **Suspicious reading**: `*_counts_accepted`/`*_meta_accepted` are left
  untouched (downstream keeps seeing the last good value for that one
  cycle) and a per-channel `reject_streak` counter increments.
- **10 suspicious readings in a row** (`ENCODER_REJECT_STREAK_LIMIT`): the
  channel gives up rejecting and accepts the new value as a real change
  (e.g. an actual FPGA reset, or the robot genuinely being moved faster
  than the threshold) rather than continuing to hold a stale value forever.
  The observed whole-burst bank-swap bug has only ever lasted exactly one
  cycle before reverting on its own, so 10 in a row is well clear of that
  while still recovering within ~100ms of a genuine change.
- A non-suspicious reading always accepts immediately and resets the streak
  to 0.

The `POSITION ANOMALY` alert now also prints the current `reject_streak`
and whether this specific occurrence is being rejected or (having exceeded
the limit) accepted, so the accept/reject decision is visible in the log
without needing to infer it.

**Status: mitigated STM32-side, confirmed working on-robot (2026-07-02)**
(this is a filter on top of the swap fix above --
`ENCODER_DEBUG_SWAP_SEA_USM_LAST_THREE`'s relabeling now reads from the
`_accepted` arrays, i.e. operates on already-glitch-filtered data). User
tested on the physical robot after flashing and confirmed the filter works
as expected. Root cause is still the unidentified FPGA-internal issue
described above; this only prevents single-cycle glitches from reaching the
rest of the firmware, it doesn't fix why they happen -- keep this filter in
place until that's found and fixed.

## Confirmed + mitigated STM32-side: SEA channels 4-6 read large values while corresponding USM channels stay near zero

Live monitoring (2026-07-02, robot at rest) showed `SEA counts: -42 -4 28 17
-3588 -3220 2456` alongside `USM counts: 0 -23287 72448 -100 0 82 1` --
channels 4, 5, 6 (last elbow motor + both wrist motors) have SEA values in
the thousands while their USM counterparts sit near zero. This matches
`MotorCalibration.lf`'s earlier per-motor pulse test exactly: driving motor
4, 5, or 6 produced zero measurable USM delta but a clear SEA delta on the
same index, for every one of those three motors.

Initially suspected a physical wiring/pin-mapping issue (SEA and USM share
a 14-bit `A_qdec`/`B_qdec`/`I_qdec` bus, sliced as `[6:0]` for SEA and
`[13:7]` for USM in `mri_encoder_reader.v`) -- **ruled out**: confirmed with
the user that physical wiring hasn't changed, and diffed the current
project's `Arty-A7-35-Master.xdc` against the last-known-working one
(`/home/johnatkins/MRIRobot_FPGA/EECS106B_QDEC/...`); every `IO_A`/`IO_B`/
`IO_I` per-pin constraint line is byte-for-byte identical between the two
(only unrelated clock-definition/bitstream-config lines differ). Spot-check
also confirms SEA ch4 (`IO_A[4]`, pin P3) and USM ch4 (`IO_A[11]`, pin W4)
land on genuinely distinct physical pins, matching the RTL's non-overlapping
bit-slice design. So this isn't a pin-routing bug.

Two hypotheses were considered:

- **Mundane** (my initial guess, now considered less likely): SEA measures
  post-gear-reduction output displacement, USM measures raw motor-shaft
  ticks, and motors 3-6's much lower `qdec_cpr` (2000 vs. 10000 for the base
  joints) plus different `gear_ratio`s per joint could in principle make the
  *same* real motion produce a huge SEA count and a tiny USM count with no
  cross-talk involved.
- **Active FPGA-internal bug**: some routing issue (not pin-level -- see
  above) mixes USM channel 4-6 data into the SEA_Block's counters, likely
  specific to the higher-indexed / `COUNTS_PER_REV_USM_B` channels
  (`SPLIT_INDEX=3` in `mri_encoder_reader.v`'s USM_Block instantiation).

**User's assessment (2026-07-02), favoring the active-bug explanation**: CPR
differences wouldn't produce this pattern, and the SEA[4,5,6] signal shape
tracks almost exactly what's expected for the *motor* (USM) signal given the
same commands -- not just "large instead of small," but shaped like the USM
response itself. That's a stronger signal than count-magnitude scaling would
produce, so the working theory now is a genuine FPGA-side routing/reroute
bug specific to channels 4-6, not a benign CPR/gearing artifact. Root cause
within the FPGA (post-pin, pre- or within `qdec_channel_bank`) still
unidentified -- would need simulation/waveform tracing of `USM_Block`'s
`count`/`count_arr` for channels 4-6 against `SEA_Block`'s to pin down where
they merge.

Separately, the user mentions having possibly just fixed an unrelated CPR
bug on the FPGA side during this same session -- noted as unrelated to the
channel-4-6 swap; not otherwise documented here since it's outside what's
been directly observed/verified from the STM32 side yet.

### Update (2026-07-02, dashboard cross-check): visually confirmed after a fresh flash

Re-checked after reflashing using the same higher-level monitoring
dashboard mentioned above. Confirmed directly: the "Motor Positions" plot
(USM-position-sourced) shows M2-M6 with no real data -- greyed out/disabled
in the legend, consistent with those channels reading near-zero/flat -- while
M1 is the only one showing real (if glitchy, see above) motion. Meanwhile
the "Motor SEA Deflection" plot shows clearly distinct nonzero, non-flat
constant offsets on multiple channels (e.g. one sitting around +20 deg,
another around -220 deg), i.e. real position information landing on SEA
where USM should have (or also) shown it, for the same last three motors.
Consistent with everything above; doesn't yet distinguish the two remaining
hypotheses on its own, but is a second, independent (dashboard vs. raw
`Encoder.lf` prints) confirmation of the same channel-4-6 pattern.

### Update (2026-07-02, STM32-side swap confirmed as a working fix): active-bug theory confirmed, not the CPR explanation

Added a debug flag to `Encoder.lf` (`ENCODER_DEBUG_SWAP_SEA_USM_LAST_THREE`)
that, for motors 4-6 only, swaps which raw counts/metadata feed the SEA vs.
USM (motor) *outputs* while keeping each output's own normal
physical-unit conversion factor -- i.e. tests whether the data itself is
simply mislabeled/cross-routed between groups, as distinct from a
CPR/gearing scaling difference. Confirmed on-robot: after the swap,
`qdec_out`/`qdec_vel_out` for motors 4-6 show sane motor position/velocity
(previously near-zero) and `sea_out` for those channels drops back to
near-zero (previously showing the large motor-like values). This directly
confirms the **active FPGA-internal routing bug** hypothesis over the
mundane CPR/gearing explanation -- a scaling-only artifact would not have
been fixable by swapping data sources like this.

**Status: mitigated STM32-side, root cause still unfixed.** The swap is
being kept in `Encoder.lf` as a marked temporary fix (search for
`ENCODER_DEBUG_SWAP_SEA_USM_LAST_THREE`) until the actual FPGA-side
cross-routing bug is identified and fixed -- do not remove the STM32-side
swap until that's confirmed to actually resolve it, or the outputs will
silently go back to being swapped. Root cause within the FPGA (still
unidentified -- see the two hypotheses above; "active FPGA-internal bug" is
now the confirmed direction, just not yet localized to a specific line)
would need simulation/waveform tracing of `USM_Block`'s `count`/`count_arr`
for channels 4-6 against `SEA_Block`'s to pin down exactly where they merge.

## Open: some SEA channels also appear swapped with each other (newly noticed 2026-07-02)

Separate from the SEA<->USM cross-group swap above (channels 4-6's real USM
data landing on the SEA readout, now mitigated via
`ENCODER_DEBUG_SWAP_SEA_USM_LAST_THREE`), the user has now also noticed that
some SEA channels appear to be swapped **with each other** -- i.e. within
the SEA group itself, not just across the SEA/USM boundary. This wasn't
caught by the earlier `MotorCalibration.lf` per-motor pulse testing or the
channel-4-6 investigation above, so it's a distinct, previously-undetected
issue.

Not yet root-caused or even fully characterized -- no specific channel
pairs, reproduction steps, or supporting log data have been captured for
this one yet (unlike the channel-4-6 issue and the whole-burst bank swap
above, both of which have concrete worked examples in this file). Given how
the earlier issues were pinned down, the same approach likely applies here
too: re-run (or re-purpose) `MotorCalibration.lf`'s per-motor pulse test and
check which `sea_out`/`sea_counts` index actually responds to each motor's
pulse, specifically looking for two SEA indices whose responses are swapped
relative to each other (as opposed to one of them just being silent, which
was the channel-4-6 symptom).

Whether this is the same underlying FPGA-internal mechanism as the other
two issues above (e.g. `qdec_arbiter`/`packet_framer` state confusion
extending to within-group channel ordering too) or a separate bug entirely
is unknown. Worth checking once reproduced whether it's connected to the
already-documented `channel_index` `+4` rotation (see "Open: channel_index
is consistently rotated by a fixed offset" above) -- that fix already
reorders channels based on the embedded self-check byte rather than word
position, so if this new swap is happening at a layer the `channel_index`
fix doesn't cover (e.g. actually swapped in the FPGA's own `count_arr`
before the metadata `ch` label is even attached), the existing fix would
not catch it and could even make it harder to notice.

## Confirmed working correctly

- **COBS framing** (`packet_framer.v` <-> `cobs_decode()`/`crc8_compute()` in
  `Encoder.lf`/`read_fpga_example.c`): 0 malformed frames across the whole
  capture (~230+ samples per group) using the current STM32-side blocking,
  byte-at-a-time read (see `Encoder.lf`; `Encoder_DMA.lf` keeps the earlier
  idle-DMA version, which had unrelated DMA-specific framing issues --
  isolating those from the packet protocol itself was the point of the
  blocking rewrite).
- **CRC-8** (poly `0x07`, init `0x00`): 0 mismatches across the same capture.
- **Byte order**: every 64-bit word (count or metadata) is little-endian
  (LSB byte first) on the wire, per `VectorToSingle.sv`'s
  `_GEN_0[counter[2:0]]` indexing and confirmed empirically. This was wrong
  in an earlier version of both `read_fpga_example.c` and `Encoder.lf`
  (assumed big-endian) -- now fixed in both.
