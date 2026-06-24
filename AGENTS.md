# Repository Guidelines

## Project Structure & Module Organization

This repository is currently an initial turbine workspace with no committed source tree. Keep project files organized as they are added:

- `src/` for source code, CAD/model generation scripts, and reusable modules.
- `tests/` for automated tests and small regression fixtures.
- `assets/` or `references/` for source images, measurements, diagrams, and other inputs.
- `output/`, `dist/`, or `build/` for generated meshes, renders, exports, and other reproducible artifacts.

Do not commit large generated files unless they are required deliverables. Prefer committing the script or parameter file that regenerates them.

## Build, Test, and Development Commands

No build system, package manifest, or test runner is checked in yet. When adding tooling, document the exact command in `README.md` and keep it runnable from the repository root.

Useful baseline commands:

- `rg --files` lists tracked project files quickly.
- `python -m pytest` should run the test suite if Python tests are added.
- `python src/<script>.py` is the preferred pattern for runnable Python scripts.

If a future workflow uses `make`, `npm`, or another tool, add a root-level config file and keep commands non-interactive.

## Coding Style & Naming Conventions

Use clear, descriptive names tied to turbine geometry and workflow concepts. For Python, use 4-space indentation, `snake_case` for functions and variables, `PascalCase` for classes, and small modules with focused responsibilities. Use lowercase, hyphenated names for generated files where practical, for example `rotor-test-fit.step`.

Keep parameters explicit near the top of scripts or in structured config files. Avoid hard-coded absolute paths; prefer paths relative to the repository root.

## Testing Guidelines

Place tests under `tests/` and name them `test_<behavior>.py` for Python projects. Add regression tests for geometry calculations, export settings, and file-generation behavior. Generated-output tests should compare stable metadata or dimensions rather than brittle binary file contents.

## Commit & Pull Request Guidelines

This directory has no Git history yet, so use concise imperative commit messages such as `Add rotor profile generator` or `Document mesh export workflow`.

Pull requests should include a short summary, verification steps, linked issue or task when available, and screenshots or rendered previews for visual/CAD changes. Note any generated artifacts and how to reproduce them.
