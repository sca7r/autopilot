# Contributing

Thanks for your interest in improving autopilot.

## Development setup

```bash
git clone <your-fork-url> autopilot
cd autopilot
pip install -e ".[dev]"
```

This installs the package in editable mode along with the linting, formatting,
and test tooling.

## Before opening a pull request

Run the full local check that CI will run:

```bash
ruff check .        # lint
black --check .     # formatting
pytest              # tests
```

Auto-format and auto-fix where possible:

```bash
black .
ruff check --fix .
```

## Guidelines

- Keep the core library (`src/autopilot/`) free of demo-only dependencies
  such as `matplotlib` and `mujoco`. Those belong with the examples.
- Add or update tests for any behavior change.
- Update `CHANGELOG.md` under the `[Unreleased]` section.
- Follow the existing style: type hints, concise docstrings, and named
  loggers rather than `print`.

## Project layout

See the "Project layout" section of the [README](README.md) for where things
live.
