# Turbine CAD Model

Parametric CAD workspace for a 3D-printable powered cutaway jet-plane turbine model.

The approved concept is documented in `docs/superpowers/specs/2026-06-25-powered-cutaway-turbine-design.md`.

## Current Status

- Contributor guide: `AGENTS.md`
- Design spec: `docs/superpowers/specs/2026-06-25-powered-cutaway-turbine-design.md`
- CAD source: `src/turbine_assembly.py`
- Primary assembly STEP: `models/turbine_assembly.step`
- Printable part STEP files: `models/parts/`

## Planned Model

- Half-cutaway nacelle
- Removable rotor cartridge on an 8 mm shaft
- 608 bearing supports
- Hidden GT2 belt-drive display base
- Low-voltage DC gearmotor envelope

This is an educational/display model, not a functional jet engine or thrust-producing turbine.

## CAD Workflow

Create a local Python environment and install the modeling dependency:

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\python -m pip install -r requirements.txt
```

Generate the primary assembly, printable STEP parts, and STL sidecars:

```powershell
.\.venv\Scripts\python src\turbine_assembly.py
```

Generated files:

- `models/turbine_assembly.step`
- `models/parts/*.step`
- `models/stl/*.stl`

STL files are ignored because they are derived mesh outputs. Regenerate them from `src/turbine_assembly.py` when needed.

## Validation Workflow

If the local Codex CAD skill is installed, set its path and run:

```powershell
$env:CAD_SKILL="$HOME\.agents\skills\cad"
.\.venv\Scripts\python "$env:CAD_SKILL\scripts\step" src/turbine_assembly.py -o models/turbine_assembly.step --force
.\.venv\Scripts\python "$env:CAD_SKILL\scripts\inspect" refs models/turbine_assembly.step --facts --planes --positioning
```

Snapshot review packet:

```powershell
@'
{
  "input": "models/turbine_assembly.step",
  "mode": "view",
  "outputs": [
    { "path": "snapshots/turbine_iso.png", "camera": "iso" },
    { "path": "snapshots/turbine_iso_opposite.png", "camera": { "direction": [-1, 1, -0.8] } },
    { "path": "snapshots/turbine_top.png", "camera": "top" },
    { "path": "snapshots/turbine_front.png", "camera": "front" }
  ],
  "render": { "viewLabels": true, "padding": 0.12, "sizeProfile": "assembly" }
}
'@ | .\.venv\Scripts\python "$env:CAD_SKILL\scripts\snapshot" --job -
```
