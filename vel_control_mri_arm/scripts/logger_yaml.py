#!/usr/bin/env python3
"""Parses Logger.lf's on-console logfmt output (see src_c/logger.c) back
into structured data and dumps it as YAML -- the receiving-computer half of
the request/response logging pattern in Logger.lf/logger.h/logger.c.

Every data line Logger.lf prints is strict logfmt: whitespace-separated
key=value tokens, no bare tokens (`source=...`/`scope=...` are themselves
key=value pairs, not headerless markers) -- see logger.c's
Logger_Format*Line functions for exactly what gets emitted. A full round
looks like:

    [LOGGER START] index=12 ts=48213
    [LOGGER] source=motor_bias seq=12 bias_0_mi=15 bias_0_mean_mi=14 ...
    [LOGGER] source=pulse_model seq=12 vmin_mi=400 vreal_mean_mi=372 ...
    [LOGGER] source=pulse_model_stats scope=pooled seq=12 vmin_mean_mi=... ...
    [LOGGER] source=pulse_model_stats scope=joint0 seq=12 ...
    ...
    [LOGGER] source=encoder_state seq=12 pos_0_mi=100 vel_0_mi=5 ...
    [LOGGER END]

This module turns one such round into a nested dict (grouped by each
line's `source`, then `scope` if present) and dumps a stream of rounds as a
multi-document YAML stream (one `---`-separated document per round, so the
output stays appendable in real time rather than one giant list that has
to be rewritten whole on every round).

NAMING CONVENTION: every numeric value in these lines is a plain int (this
platform's printf doesn't reliably support floats -- see logger.c's
MILLI() macro). A key ending in `_mi` ("milli-int") is one of those
milli-scaled values -- this module strips the suffix and divides by 1000.0
to hand back a real float under the plain key name. Every other key (seq,
epochs, converged, index, ts, and anything ending in `_n`) is already an
exact int/count and passes through unscaled. This is a firmware/host
contract: if logger.c ever adds a milli-scaled field, it MUST get an `_mi`
suffix there for this module to rescale it correctly.

---- Quickstart ----
    python3 scripts/logger_yaml.py path/to/serial_log.txt > snapshot.yaml
    # or, reading live off a file something else is appending serial output to:
    tail -f /path/to/serial.log | python3 scripts/logger_yaml.py -
"""

import sys

import yaml


_MILLI_SUFFIX = "_mi"


def parse_logfmt_line(line: str) -> dict:
    """Splits one logfmt line into a flat {key: value} dict, rescaling any
    `_mi`-suffixed key back to a real float (see the module docstring's
    naming convention). Tokens without '=' (there shouldn't be any in
    Logger.lf's own output, but a stray console print interleaved on the
    same serial link is always possible) are silently skipped rather than
    raising, so a line from something else sharing the UART doesn't take
    the whole round down.
    """
    fields = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, _, raw_value = token.partition("=")
        value = _coerce(raw_value)

        if key.endswith(_MILLI_SUFFIX) and isinstance(value, int):
            key = key[: -len(_MILLI_SUFFIX)]
            value = value / 1000.0

        fields[key] = value
    return fields


def _coerce(value: str):
    """logfmt values here are always plain ints (see module docstring) --
    falls back to the raw string for anything that isn't, rather than
    raising, so one unexpected field doesn't break parsing of the rest of
    the round."""
    try:
        return int(value)
    except ValueError:
        return value


def parse_log_round(lines) -> dict:
    """Turns one round's lines into a single nested dict: the START
    line's index/ts fold straight into the top level (it has no
    `source`), every other line's fields land under round_dict[source]
    (or round_dict[source][scope] for the pulse_model_stats lines, the
    only ones with a `scope`)."""
    round_dict = {}
    for line in lines:
        fields = parse_logfmt_line(line)
        if not fields:
            continue  # blank line, or a bracket-only marker like [LOGGER END]

        if "source" not in fields:
            round_dict.update(fields)
            continue

        source = fields.pop("source")
        scope = fields.pop("scope", None)
        if scope is None:
            round_dict[source] = fields
        else:
            round_dict.setdefault(source, {})[scope] = fields

    return round_dict


def read_log_rounds(lines):
    """Generator over an iterable of lines (an open file, stdin, a live
    tail -f, ...): yields one parsed round dict per [LOGGER START]/
    [LOGGER END] pair. Lines outside a START/END pair (other console
    output sharing the same link) are ignored; a round missing its END
    (e.g. the log was cut off mid-round) is dropped rather than yielded
    half-built.
    """
    current = None
    for raw_line in lines:
        line = raw_line.strip()
        if line.startswith("[LOGGER START]"):
            current = [line]
        elif line.startswith("[LOGGER END]"):
            if current is not None:
                yield parse_log_round(current)
            current = None
        elif current is not None:
            current.append(line)


def dump_yaml_stream(rounds, stream=sys.stdout) -> None:
    """Writes each round as its own `---`-separated YAML document."""
    for round_dict in rounds:
        stream.write("---\n")
        yaml.safe_dump(round_dict, stream, sort_keys=False)


def main(argv) -> int:
    if len(argv) != 2:
        print(f"usage: {argv[0]} <path/to/log.txt | ->", file=sys.stderr)
        return 1

    source = sys.stdin if argv[1] == "-" else open(argv[1], "r")
    try:
        dump_yaml_stream(read_log_rounds(source))
    finally:
        if source is not sys.stdin:
            source.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
