# Including external C files in a `.lf` reactor

Working pattern for pulling a plain `.c`/`.h` pair into an LF reactor file,
worked out while building `Small_Velocity_Controller.lf` /
`pulse_mpc.h`/`pulse_mpc.c`. Use this instead of writing non-trivial C logic
directly inline in a `preamble {= ... =}` block.

## 1. Target properties: `files:` + `cmake-include:`

```lf
target C {
  ...
  files: ["../../../src_c/pulse_mpc.h", "../../../src_c/pulse_mpc.c"],
  cmake-include: ["../../../src_c/pulse_mpc_cmake.cmake"]
}
```

- `files:` only **copies** the listed files into `src-gen/Main/`. It does
  **not** add them to the compiled/linked sources.
- `cmake-include:` copies a `.cmake` snippet in the same way, and it must be
  the one to actually wire the `.c` file into the build:

  ```cmake
  # src_c/pulse_mpc_cmake.cmake
  target_sources(${LF_MAIN_TARGET} PRIVATE pulse_mpc.c pulse_mpc.h)
  ```

  Without this, `pulse_mpc.c` gets copied but never compiled.

## 2. Path resolution

Both `files:` and `cmake-include:` entries resolve the same way (same LF
`findInPackage` code path): tried relative to the **main file's `src/`
directory**, then relative to the **project root** (the parent of `src/`).
In practice, the reliable choice for a `.lf` file nested inside
`src/lib/...` is a plain relative path from that file's own location up to
wherever the shared C sources live, e.g. from
`src/lib/SmallVelocityControl/Small_Velocity_Controller.lf` up to
`src_c/pulse_mpc.h`:

```
../../../src_c/pulse_mpc.h
```

(three levels: `SmallVelocityControl/` → `lib/` → `src/` → project root, then
into `src_c/`). Don't assume a leading `/` or a bare `src_c/...` path will
resolve correctly from a nested reactor file — verify by actually running
`make build_dev` and checking for "Unable to copy ... from the class path"
errors, which mean the path didn't resolve.

## 3. Getting the type into the reactor's `state` fields

If a reactor needs a `state` field typed with a struct from the external
header (e.g. `state controller_: PulseMPC`), the `#include` **must** go in a
**top-level preamble** — written before the `reactor` keyword, not nested
inside the reactor body:

```lf
preamble {=
  #include "pulse_mpc.h"
=}

reactor Small_Velocity_Controller(...) {
  state controller_: PulseMPC
  ...
}
```

Two things to know here, both learned the hard way:

- **A reactor-scoped preamble (nested inside `reactor { ... }`) only lands in
  the generated `.c` file, never the self-struct header.** If the `state`
  field's type isn't visible yet when the header is generated, you get
  `error: unknown type name 'PulseMPC'`. Moving the same `#include` to a
  top-level preamble (outside the reactor) fixes this — top-level preambles
  do get placed ahead of the self-struct in the generated header.
- **Only put a bare `#include` in a top-level preamble, not inline
  typedefs/structs/functions.** LF copies a top-level preamble's contents
  into the generated header of every file that transitively imports that
  reactor (e.g. both `Small_Velocity_Controller.h` and
  `Small_Velocity_Controller_Bank.h` in this case), each under a
  differently-numbered include guard. If the preamble body is real inline
  code, both copies get compiled into the same translation unit and you get
  `redeclaration of enumerator`/`conflicting types` errors. An `#include` of
  a header with its own include guard is immune to this, since the second
  copy just no-ops — that's why the header-file approach is what makes this
  work at all, not a stylistic preference.

## Summary

- Real C logic (types + functions) → a real `.h`/`.c` pair in `src_c/`,
  wired in via `files:` + `cmake-include:`.
- Reactor file → a **top-level** `preamble {= #include "your_header.h" =}`
  only, then a thin reactor body (I/O, timing, `state` fields, reactions
  that call the external functions).
- Verify with `make build_dev` (codegen + compile) and `make build_test`
  (from-scratch cmake build) — both must succeed, and check the generated
  header for the file in question to confirm the type actually shows up in
  the self-struct if using it as a `state` field type.
