# CLAUDE.md — libcalib

## What this repo is

libcalib is an embedded-portable C++ calibration library for IMU sensors (accelerometer,
gyroscope, magnetometer). It is a dependency of both SensorCal (macOS/Win/Linux desktop app)
and kitelite (ESP32 Feather firmware). It must run on both without modification.

libcalib was extracted from SensorCal as the calibration logic was separated from the UI.
It has no upstream fork — it is original code. The long-term goal is for Adafruit to take
ownership of libcalib and SensorCal as first-party tools in their ecosystem. Code quality,
API clarity, and documentation standards must reflect that ambition.

## How to work in this repo

- **Never `git add`, `git commit`, or `git push` anything**, ever, under any circumstances.
  Leave all changes as working tree modifications only.
- **Never create or switch branches** without explicit instruction from the user.
- When a task is complete, summarize what files changed and why. The user will review
  every diff, stage what they want, and commit on their own schedule.
- Before making changes that touch more than one file, propose a plan and wait for approval.
- Prefer small, focused changes over large rewrites in a single pass.
- Write code as if the user will need to read, understand, and debug it without your help.
  Clarity over cleverness, always.

## Hard constraints (embedded portability)

- No heap allocation (`new`, `delete`, `malloc` are forbidden)
- No STL (`std::vector`, `std::string`, `std::map`, etc. are forbidden)
- No exceptions, no RTTI
- No standard library headers unavailable in the Arduino/ESP32 toolchain
- C++17 maximum; conservative feature use — namespaces and basic templates are fine,
  heavy metaprogramming is not
- Use ETL (Embedded Template Library) if container types are needed
- All sizing via compile-time constants or fixed-size arrays

## AHRS

libcalib supports three AHRS backends:
- **Fusion** (xioTechnologies, Sebastian Madgwick) — current preferred backend
- **NXP** — older Kalman-based fusion, bundled with original MotionCal
- **Mahony** — also bundled with original MotionCal

Fusion is vendored as a git submodule at `vendor/Fusion`. Whether to expose backend
selection as a runtime or compile-time option is TBD — it may be useful for less capable
devices that can't afford Fusion's computational cost.

Key architectural decision: the AHRS backend is a dependency *of libcalib*, not wired
separately by consumers. The choice between `FusionAhrsUpdate` (with magnetometer) vs
`FusionAhrsUpdateNoMagnetometer` depends on `m_fitter.FHasSolution()`, which is private to
`Calibrator`. Consumers must not bypass this.

## Calibration state machine

libcalib currently owns the magnetometer calibration state machine:
- **Magnetometer**: sphere-fitting, hard/soft iron correction matrix, coverage quality metrics

Accelerometer and gyroscope calibration state machines are planned but do not yet exist.
They are a goal for future development — do not invent classes or APIs for them.

The state machine is driven by a host (SensorCal on desktop, or kitelite on device).
The host provides raw sensor samples; libcalib tracks state, determines completion, and
produces calibration output. All display/feedback logic lives in the host, not here.

## Key classes

- `libcalib::Calibrator` — singleton, owns all calibration state, drives the AHRS
- `libcalib::Sphere::CFitter` — magnetometer sphere fitting and quality metrics;
  `FHasSolution()` indicates whether a valid calibration has been computed

## Coding style

See `CLAUDE-coding.md` in the repo root for full reference. Key rules:

- Tabs (not spaces), 4-wide
- `m_` prefix on all class/struct members
- `g_` prefix on file-static globals; `s_` on function-static variables
- No `enum class` — plain enums, integer-convertible
- `ASSERT` / `CASSERT` / `VERIFY` for correctness checks
- `nullptr` not `NULL`
- C++ casts only (`static_cast`, `reinterpret_cast`) — never C-style casts
- `const` on read-only methods and pointed-to data; not on primitive locals
- `override` on all overridden virtual functions
- `(void)FTrySomething();` to explicitly discard return values
- `//` comments only, never `/* */`
- `BB(username)` for known improvement areas; `NOTE(username)` for non-obvious decisions
- Functions named `[ReturnTag]VerbNoun`: `FIsValid()`, `AddRawSample()`, `GFieldStrength()`

## Vendor submodules

All vendored dependencies live under `vendor/`:
- `vendor/Fusion` — xioTechnologies/Fusion AHRS (pure C, no heap, embedded-safe)

When updating submodules: commit and push the submodule first, then update the SHA reference
in the parent repo. Never leave dangling submodule references.

## Things to be careful about

- `m_fitter.FHasSolution()` is the correct way to test whether a valid magnetometer
  calibration exists — the AHRS mag/no-mag branch must stay inside `Calibrator` and
  must not leak to consumers
- Soft iron matrix uses enum indexing — keep the enum and matrix access in sync
- `Calibrator` is a singleton (`Calibrator::Ensure()`) — be careful with lifetime
- All fixed-size buffers must have accompanying `static_assert` size guards