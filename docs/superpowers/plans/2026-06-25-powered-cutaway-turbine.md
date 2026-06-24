# Powered Cutaway Turbine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a parametric STEP-first CAD assembly for a 3D-printable, low-speed powered half-cutaway jet turbine display model.

**Architecture:** A single build123d generator owns the geometry and exports a labeled full assembly plus individual printable part STEP files. The model is split into nacelle shell sections, rotor cartridge disks, fixed stator rings, and a hidden belt-drive base sized for an 8 mm shaft, 608 bearings, GT2 belt/pulley envelopes, and an adjustable DC gearmotor pocket.

**Tech Stack:** Python 3.13, build123d, CAD skill `scripts/step`, CAD skill `scripts/inspect`, CAD skill `scripts/snapshot`.

---

## File Structure

- Create `src/turbine_assembly.py`: parametric build123d source with `gen_step()` and a CLI export path for individual printable parts.
- Create `models/`: generated STEP assembly and derived hidden CAD viewer artifacts.
- Create `models/parts/`: generated STEP files for individual printable groups.
- Create `snapshots/`: generated PNG review packet.
- Modify `.gitignore`: keep local `.venv/`, generated mesh/build output, and `.superpowers/` out of Git.
- Modify `README.md`: document local CAD dependency setup and generation commands.

### Task 1: Environment And Plan Baseline

**Files:**
- Modify: `.gitignore`
- Create: `docs/superpowers/plans/2026-06-25-powered-cutaway-turbine.md`

- [ ] **Step 1: Use a feature branch**

Run: `git switch -c cad/powered-cutaway-turbine`
Expected: branch `cad/powered-cutaway-turbine` is active.

- [ ] **Step 2: Install CAD dependencies locally**

Run:

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\python -m pip install -e 'C:\Users\javeliner\.agents\skills\cad\scripts\packages\cadpy' playwright
```

Expected: imports for `build123d` and `cadpy` succeed from `.venv`.

- [ ] **Step 3: Confirm CAD CLI help works**

Run:

```powershell
.\.venv\Scripts\python 'C:\Users\javeliner\.agents\skills\cad\scripts\step' --help
```

Expected: command prints `scripts/step` usage without `ModuleNotFoundError`.

### Task 2: Parametric Generator

**Files:**
- Create: `src/turbine_assembly.py`

- [ ] **Step 1: Define parameters**

Create named constants for `printer_bed`, `nacelle_od`, `nacelle_length`, `base_length`, `base_width`, `shaft_diameter`, `bearing_od`, `bearing_width`, `m3_clearance`, `gt2_pulley_envelope`, and `clearance`.

- [ ] **Step 2: Add part builders**

Implement functions named `make_base()`, `make_lower_nacelle()`, `make_cutaway_upper_shell()`, `make_rotor_stack()`, `make_stator_ring(name, x_position)`, `make_service_cover()`, and `make_reference_hardware()`.

- [ ] **Step 3: Add assembly builder**

Implement `build_printable_parts()` returning a dictionary of named solids and `gen_step()` returning a labeled `Compound` with all parts placed in assembled positions.

- [ ] **Step 4: Add direct export CLI**

Implement `export_printable_parts(output_dir='models/parts')` and a `__main__` block that writes individual STEP files plus `models/turbine_assembly.step`.

### Task 3: STEP Generation

**Files:**
- Generate: `models/turbine_assembly.step`
- Generate: `models/parts/*.step`

- [ ] **Step 1: Run syntax/import check**

Run: `.\.venv\Scripts\python -m py_compile src\turbine_assembly.py`
Expected: exit code `0`.

- [ ] **Step 2: Generate the primary assembly through CAD CLI**

Run:

```powershell
.\.venv\Scripts\python 'C:\Users\javeliner\.agents\skills\cad\scripts\step' src/turbine_assembly.py -o models/turbine_assembly.step --force
```

Expected: `models/turbine_assembly.step` exists and CAD viewer sidecars are generated.

- [ ] **Step 3: Generate individual printable parts**

Run: `.\.venv\Scripts\python src\turbine_assembly.py`
Expected: STEP files appear under `models/parts/`.

### Task 4: Validation

**Files:**
- Read: `models/turbine_assembly.step`
- Create: `snapshots/*.png`

- [ ] **Step 1: Baseline inspect assembly**

Run:

```powershell
.\.venv\Scripts\python 'C:\Users\javeliner\.agents\skills\cad\scripts\inspect' refs models/turbine_assembly.step --facts --planes --positioning
```

Expected: inspection returns valid geometry facts and no crash.

- [ ] **Step 2: Validate printable part bounds**

Run:

```powershell
@'
from src.turbine_assembly import build_printable_parts, PRINTER_BED
for name, part in build_printable_parts().items():
    box = part.bounding_box()
    dims = (box.size.X, box.size.Y, box.size.Z)
    print(name, tuple(round(v, 2) for v in dims))
    assert dims[0] <= PRINTER_BED[0]
    assert dims[1] <= PRINTER_BED[1]
    assert dims[2] <= PRINTER_BED[2]
'@ | .\.venv\Scripts\python -
```

Expected: every printed tuple is within `(325, 325, 350)`.

- [ ] **Step 3: Generate snapshot packet**

Run:

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
'@ | .\.venv\Scripts\python 'C:\Users\javeliner\.agents\skills\cad\scripts\snapshot' --job -
```

Expected: PNG files are written under `snapshots/`.

### Task 5: Docs, Commit, And Handoff

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Document generation commands**

Add a `CAD Workflow` section to `README.md` containing the exact `.venv` setup, assembly STEP generation, individual part export, inspection, and snapshot commands from Tasks 1, 3, and 4.

- [ ] **Step 2: Commit implementation**

Run:

```powershell
git status --short
git add .gitignore README.md src docs models snapshots
git commit -m "Add powered cutaway turbine CAD model"
```

Expected: commit succeeds on `cad/powered-cutaway-turbine`.

- [ ] **Step 3: Push branch**

Run: `git push -u origin cad/powered-cutaway-turbine`
Expected: branch is available on GitHub.
