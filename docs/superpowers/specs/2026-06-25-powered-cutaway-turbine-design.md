# Powered Cutaway Turbine CAD Design

## Goal

Create a parametric CAD model of a jet-plane-style turbine that is suitable for desktop 3D printing, assembly, and low-speed powered display rotation. The model is educational/display hardware only: it is not a functional jet engine, thrust device, compressor, or high-speed turbine.

## Printer And Scale Constraints

- Printer volume: `325 x 325 x 350 mm`.
- Every individual printable part must fit within the printer volume with practical margin for skirts, brim, supports, and handling.
- Target maximum part envelope: about `300 x 300 x 330 mm` or smaller.
- Units: millimeters.

## Approved Concept

The selected design is a modular half-cutaway desktop assembly with a hidden base belt drive.

The turbine body will look like a jet engine nacelle while leaving one side open to reveal the internal fan, compressor-style stages, turbine-style stages, shaft, and stator rings. The motor will not sit in line behind the turbine; it will be hidden in the display base and drive the rotor shaft through a GT2 belt.

Baseline model dimensions:

- Turbine nacelle outside diameter: about `160 mm`.
- Turbine body length: about `300 mm`.
- Display base footprint: about `300 x 140 mm`.
- Turbine shaft centerline above base: about `125 mm`.

## Hardware Assumptions

Use common off-the-shelf hardware envelopes:

- `8 mm` steel shaft.
- `608` bearings for shaft support.
- GT2 belt and pulleys.
- Low-voltage DC gearmotor mounted inside the base.
- M3 screws for shell and base assembly, using normal printed clearance holes around `3.4 mm`.

Exact motor face holes, pulley bore details, and belt length are treated as adjustable parameters because purchased parts vary.

## Part Breakdown

The CAD source should generate these printed groups:

- Split nacelle shells with inlet lip, rear nozzle, cutaway edge, screw bosses, and alignment pins.
- Removable rotor cartridge with front fan disk, compressor visual disks, turbine visual disks, hub spacers, and `8 mm` shaft bore.
- Fixed stator rings that mount to the nacelle and remain clear of rotating disks.
- Display drive base with motor pocket, bearing supports, GT2 belt channel, pulley clearance, wiring/switch cavity, and removable service cover.

## Modeling Scope

Included geometry:

- Thick printable fan blades and stage blades.
- Bearing pockets and shaft path.
- Belt-drive passage between base and turbine shaft.
- Clearances between rotating and fixed geometry.
- Printable mating faces, bosses, alignment pins, and covers.
- Visual turbine details that communicate compressor/turbine stages.

Intentionally simplified:

- No combustion chamber, fuel system, hot-section function, or airflow/thrust goal.
- No aerodynamic blade optimization.
- No high-speed rotor balancing claim.
- No electrical safety certification.

## Source And Outputs

Primary source should be a parametric CAD generator, expected path:

- `src/turbine_assembly.py`

Expected outputs:

- Primary assembled STEP model.
- Separate STEP files for printable groups or individual parts.
- Secondary STL/mesh exports if supported by the local CAD toolchain.
- Snapshot images for visual inspection.

STEP is the primary validated CAD artifact. Meshes are derived outputs for printing workflows.

## Validation Plan

Implementation must verify:

- Each individual printable part fits within `325 x 325 x 350 mm`.
- Shaft bore, bearing pocket, belt path, and pulley envelope dimensions are coherent.
- Rotating disks have clearance from stators and shell geometry.
- Base has accessible motor and service-cover geometry.
- The final STEP assembly is visually inspected through snapshots.

Validation should report actual generated paths and checks that were run.
