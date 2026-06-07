# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- Restructured the repository into a standard `src/` layout
  (`src/autopilot/`), with runnable demos under `examples/` and the test
  suite at the top level.
- Split dependencies: the core library now depends only on the numerical /
  optimization stack; `matplotlib` and `mujoco` moved to the `demos` and
  `dev` optional extras.
- Scenario configuration (`simulation.yaml`) now lives alongside the example
  demos instead of being bundled inside the installed package.

### Removed
- Removed the historical Jupyter derivation notebooks; the repository now
  ships only the library, demos, and tests.

### Added
- Expanded the test suite to cover path arc-length bookkeeping, obstacle
  advancement, and the controller's emergency-braking fallback.

## [0.2.0]

### Added
- Installable package with a correct build backend and console-friendly
  layout.
- Test suite, GitHub Actions CI (lint, format, test matrix, wheel build),
  and `ruff` / `black` configuration.
- Structured logging throughout the library and demos.
