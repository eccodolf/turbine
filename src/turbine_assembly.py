from __future__ import annotations

from math import cos, pi, radians, sin
from pathlib import Path

from build123d import (
    Box,
    BuildLine,
    BuildSketch,
    Compound,
    Cone,
    Cylinder,
    Line,
    Location,
    Plane,
    Rot,
    Spline,
    Torus,
    export_step,
    export_stl,
    extrude,
    make_face,
)

# Coordinate convention:
# X is the turbine shaft axis, Y is lateral width, Z is vertical/up.
# Assembly origin sits at the center of the display base footprint.

PRINTER_BED = (325.0, 325.0, 350.0)

NACELLE_OD = 160.0
NACELLE_LENGTH = 270.0
NACELLE_WALL = 4.0
NACELLE_RADIUS = NACELLE_OD / 2.0
SHAFT_CENTER_Z = 125.0
FAN_RADIUS = 73.0
CORE_CASE_RADIUS = 36.0
REAR_TURBINE_RADIUS = 59.0

BASE_LENGTH = 300.0
BASE_WIDTH = 140.0
BASE_HEIGHT = 42.0
BASE_Z = BASE_HEIGHT / 2.0

SHAFT_DIAMETER = 8.0
SHAFT_RADIUS = SHAFT_DIAMETER / 2.0
SHAFT_LENGTH = 308.0
BEARING_OD = 22.0
BEARING_WIDTH = 7.0
BEARING_CLEARANCE = 0.35
FRONT_BEARING_X = -88.0
REAR_BEARING_X = 132.0
M3_CLEARANCE = 3.4

GT2_PULLEY_DIAMETER = 20.0
GT2_PULLEY_WIDTH = 12.0
GT2_BELT_WIDTH = 6.0
CLEARANCE = 1.5

MOTOR_LENGTH = 58.0
MOTOR_WIDTH = 38.0
MOTOR_HEIGHT = 32.0


def _label(shape, label: str):
    shape.label = label
    return shape


def _x_cylinder(radius: float, length: float, label: str | None = None):
    shape = Rot(0, 90, 0) * Cylinder(radius=radius, height=length)
    return _label(shape, label) if label else shape


def _axis_cylinder(axis: str, radius: float, length: float, label: str | None = None):
    if axis == "x":
        shape = Rot(0, 90, 0) * Cylinder(radius=radius, height=length)
    elif axis == "y":
        shape = Rot(90, 0, 0) * Cylinder(radius=radius, height=length)
    elif axis == "z":
        shape = Cylinder(radius=radius, height=length)
    else:
        raise ValueError(f"Unsupported cylinder axis: {axis}")
    return _label(shape, label) if label else shape


def _x_torus(major_radius: float, minor_radius: float, label: str | None = None):
    shape = Rot(0, 90, 0) * Torus(major_radius=major_radius, minor_radius=minor_radius)
    return _label(shape, label) if label else shape


def _placed(shape, x: float = 0.0, y: float = 0.0, z: float = 0.0, label: str | None = None):
    placed = Location((x, y, z)) * shape
    return _label(placed, label) if label else placed


def _duct_ring(x: float, radius: float, length: float, wall: float, label: str):
    outer = _placed(_x_cylinder(radius, length), x=x, z=SHAFT_CENTER_Z)
    inner = _placed(_x_cylinder(radius - wall, length + 2.0), x=x, z=SHAFT_CENTER_Z)
    return _label(outer - inner, label)


def _arc_blocks(
    x: float,
    inner_radius: float,
    outer_radius: float,
    thickness_x: float,
    start_deg: float,
    end_deg: float,
    block_count: int,
    label: str,
):
    center_radius = (inner_radius + outer_radius) / 2.0
    radial_width = outer_radius - inner_radius
    span = end_deg - start_deg
    step = span / block_count
    children = []
    for index in range(block_count):
        angle = start_deg + (index + 0.5) * step
        arc_len = max(3.5, 2.0 * pi * center_radius * abs(step) / 360.0 * 0.78)
        block = Rot(angle - 90.0, 0, 0) * Box(thickness_x, arc_len, radial_width)
        y = center_radius * cos(radians(angle))
        z = SHAFT_CENTER_Z + center_radius * sin(radians(angle))
        children.append(_placed(block, x=x, y=y, z=z, label=f"{label}_{index + 1:02d}"))
    return _label(Compound(children=children), label)


def _open_case_ring(x: float, radius: float, length: float, wall: float, label: str):
    start_deg = -145.0
    end_deg = 145.0
    rim = _arc_blocks(x, radius - wall, radius, length, start_deg, end_deg, 34, f"{label}_rim")
    end_caps = [
        _radial_blade(x, radius - wall, radius, wall + 1.5, length, start_deg, 0.0, f"{label}_cut_edge_a"),
        _radial_blade(x, radius - wall, radius, wall + 1.5, length, end_deg, 0.0, f"{label}_cut_edge_b"),
    ]
    return _label(Compound(children=[rim, *end_caps]), label)


def _radial_blade(
    x: float,
    inner_radius: float,
    outer_radius: float,
    chord: float,
    thickness_x: float,
    angle_deg: float,
    sweep_deg: float,
    label: str,
):
    radial_length = outer_radius - inner_radius
    center_radius = inner_radius + radial_length / 2.0
    blade = Box(thickness_x, chord, radial_length)
    blade = Rot(angle_deg - 90.0 + sweep_deg, 0, 0) * blade
    y = center_radius * cos(radians(angle_deg))
    z = SHAFT_CENTER_Z + center_radius * sin(radians(angle_deg))
    return _placed(blade, x=x, y=y, z=z, label=label)


def _polar_point(radius: float, angle_deg: float):
    return (radius * cos(radians(angle_deg)), SHAFT_CENTER_Z + radius * sin(radians(angle_deg)))


def _curved_blade(
    x: float,
    inner_radius: float,
    outer_radius: float,
    root_chord: float,
    tip_chord: float,
    thickness_x: float,
    angle_deg: float,
    sweep_deg: float,
    label: str,
):
    inner_half = (root_chord / max(inner_radius, 1.0)) * 28.6478898
    outer_half = (tip_chord / max(outer_radius, 1.0)) * 28.6478898
    root_a = angle_deg - inner_half
    root_b = angle_deg + inner_half
    tip_a = angle_deg + sweep_deg - outer_half
    tip_b = angle_deg + sweep_deg + outer_half
    root_le = _polar_point(inner_radius, root_a)
    root_te = _polar_point(inner_radius, root_b)
    tip_le = _polar_point(outer_radius, tip_a)
    tip_te = _polar_point(outer_radius, tip_b)
    mid_le = _polar_point((inner_radius + outer_radius) / 2.0, (root_a + tip_a) / 2.0 - 3.0)
    mid_te = _polar_point((inner_radius + outer_radius) / 2.0, (root_b + tip_b) / 2.0 + 3.0)
    try:
        with BuildSketch(Plane.YZ) as blade_sketch:
            with BuildLine():
                Spline(root_le, mid_le, tip_le)
                Line(tip_le, tip_te)
                Spline(tip_te, mid_te, root_te)
                Line(root_te, root_le)
            make_face()
        blade = extrude(blade_sketch.sketch, amount=thickness_x)
        return _placed(blade, x=x - thickness_x / 2.0, label=label)
    except Exception:
        return _radial_blade(x, inner_radius, outer_radius, (root_chord + tip_chord) / 2.0, thickness_x, angle_deg, sweep_deg, label)


def _swept_fan_blade(
    x: float,
    inner_radius: float,
    outer_radius: float,
    thickness_x: float,
    angle_deg: float,
    label: str,
):
    blade = _curved_blade(
        x=x,
        inner_radius=inner_radius,
        outer_radius=outer_radius,
        root_chord=12.0,
        tip_chord=23.0,
        thickness_x=thickness_x,
        angle_deg=angle_deg,
        sweep_deg=38.0,
        label=f"{label}_airfoil",
    )
    raised_rib = _curved_blade(
        x=x + thickness_x / 2.0 + 0.9,
        inner_radius=inner_radius + 4.0,
        outer_radius=outer_radius - 5.0,
        root_chord=2.2,
        tip_chord=3.0,
        thickness_x=1.8,
        angle_deg=angle_deg,
        sweep_deg=38.0,
        label=f"{label}_raised_midrib",
    )
    return _label(Compound(children=[blade, raised_rib]), label)


def _fan_stage(x: float, outer_radius: float, thickness: float, blade_count: int, label: str):
    hub = _placed(_x_cylinder(radius=24.0, length=thickness + 8.0), x=x, z=SHAFT_CENTER_Z, label=f"{label}_hub")
    blade_root = _placed(_x_cylinder(radius=31.0, length=thickness), x=x, z=SHAFT_CENTER_Z, label=f"{label}_root_ring")
    bolt_heads = []
    for index in range(12):
        angle = index * 360.0 / 12.0
        y = 19.0 * cos(radians(angle))
        z = SHAFT_CENTER_Z + 19.0 * sin(radians(angle))
        bolt_heads.append(_placed(_x_cylinder(1.6, 2.0), x=x - thickness / 2.0 - 1.0, y=y, z=z, label=f"{label}_hub_bolt_{index + 1:02d}"))
    blades = []
    for index in range(blade_count):
        angle = index * 360.0 / blade_count
        blades.append(_swept_fan_blade(x, 29.0, outer_radius, thickness, angle, f"{label}_blade_{index + 1:02d}"))
    return _label(Compound(children=[hub, blade_root, *bolt_heads, *blades]), label)


def _stage_disk(
    x: float,
    outer_radius: float,
    thickness: float,
    blade_count: int,
    label: str,
    hub_radius: float = 14.0,
    blade_sweep: float = 16.0,
):
    hub = _placed(_x_cylinder(radius=hub_radius, length=thickness + 4.0), x=x, z=SHAFT_CENTER_Z, label=f"{label}_hub")
    blade_root = _placed(
        _x_cylinder(radius=hub_radius + 4.0, length=thickness * 0.85),
        x=x,
        z=SHAFT_CENTER_Z,
        label=f"{label}_blade_root_ring",
    )
    blades = []
    for index in range(blade_count):
        angle = index * 360.0 / blade_count
        blade = _curved_blade(
            x=x,
            inner_radius=hub_radius + 5.0,
            outer_radius=outer_radius,
            root_chord=max(5.5, outer_radius * 0.12),
            tip_chord=max(7.0, outer_radius * 0.18),
            thickness_x=thickness,
            angle_deg=angle,
            sweep_deg=blade_sweep,
            label=f"{label}_blade_{index + 1:02d}",
        )
        blades.append(blade)
    return _label(Compound(children=[hub, blade_root, *blades]), label)


def _open_vaned_ring(
    name: str,
    x_position: float,
    inner_radius: float,
    outer_radius: float,
    thickness_x: float,
    vane_count: int,
):
    start_deg = -145.0
    end_deg = 145.0
    outer_rim = _arc_blocks(x_position, outer_radius - 4.0, outer_radius, thickness_x, start_deg, end_deg, 36, f"{name}_outer_rim")
    inner_rim = _arc_blocks(x_position, inner_radius, inner_radius + 3.2, thickness_x, start_deg, end_deg, 30, f"{name}_inner_rim")
    vanes = []
    for index in range(vane_count):
        angle = start_deg + (index + 0.5) * (end_deg - start_deg) / vane_count
        vanes.append(
            _curved_blade(
                x=x_position,
                inner_radius=inner_radius + 4.0,
                outer_radius=outer_radius - 5.0,
                root_chord=3.6,
                tip_chord=5.0,
                thickness_x=thickness_x,
                angle_deg=angle,
                sweep_deg=-10.0,
                label=f"{name}_vane_{index + 1:02d}",
            )
        )
    end_caps = [
        _radial_blade(x_position, inner_radius, outer_radius, 5.0, thickness_x, start_deg, 0.0, f"{name}_cut_edge_a"),
        _radial_blade(x_position, inner_radius, outer_radius, 5.0, thickness_x, end_deg, 0.0, f"{name}_cut_edge_b"),
    ]
    return _label(Compound(children=[outer_rim, inner_rim, *vanes, *end_caps]), name)


def _gear_wheel(x: float, y: float, radius: float, thickness: float, tooth_count: int, label: str):
    body = _placed(_x_cylinder(radius=radius * 0.78, length=thickness), x=x, y=y, z=SHAFT_CENTER_Z, label=f"{label}_body")
    hub = _placed(_x_cylinder(radius=radius * 0.35, length=thickness + 5.0), x=x, y=y, z=SHAFT_CENTER_Z, label=f"{label}_hub")
    teeth = []
    for index in range(tooth_count):
        angle = index * 360.0 / tooth_count
        tooth = Box(thickness, 6.0, 8.0)
        tooth = Rot(angle - 90.0, 0, 0) * tooth
        tooth_y = y + radius * cos(radians(angle))
        tooth_z = SHAFT_CENTER_Z + radius * sin(radians(angle))
        teeth.append(_placed(tooth, x=x, y=tooth_y, z=tooth_z, label=f"{label}_tooth_{index + 1:02d}"))
    return _label(Compound(children=[body, hub, *teeth]), label)


def _pipe_run(points: list[tuple[float, float, float]], thickness: float, label: str):
    segments = []
    for index, (start, end) in enumerate(zip(points, points[1:]), start=1):
        x1, y1, z1 = start
        x2, y2, z2 = end
        if abs(x2 - x1) >= abs(y2 - y1) and abs(x2 - x1) >= abs(z2 - z1):
            length = abs(x2 - x1) or thickness
            seg = _axis_cylinder("x", thickness / 2.0, length)
        elif abs(y2 - y1) >= abs(z2 - z1):
            length = abs(y2 - y1) or thickness
            seg = _axis_cylinder("y", thickness / 2.0, length)
        else:
            length = abs(z2 - z1) or thickness
            seg = _axis_cylinder("z", thickness / 2.0, length)
        segments.append(
            _placed(
                seg,
                x=(x1 + x2) / 2.0,
                y=(y1 + y2) / 2.0,
                z=(z1 + z2) / 2.0,
                label=f"{label}_segment_{index}",
            )
        )
    return _label(Compound(children=segments), label)


def _bolt_circle(x: float, radius: float, count: int, bolt_radius: float, bolt_length: float, label: str):
    bolts = []
    for index in range(count):
        angle = index * 360.0 / count
        y, z = _polar_point(radius, angle)
        bolts.append(_placed(_x_cylinder(bolt_radius, bolt_length), x=x, y=y, z=z, label=f"{label}_{index + 1:02d}"))
    return _label(Compound(children=bolts), label)


def make_base():
    base = Box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
    belt_slot = _placed(Box(38.0, 34.0, BASE_HEIGHT + 4.0), x=-8.0, z=BASE_Z + 5.0)
    motor_pocket = _placed(Box(MOTOR_LENGTH, MOTOR_WIDTH, MOTOR_HEIGHT), x=-72.0, z=BASE_Z + 8.0)
    wiring_channel = _placed(Box(92.0, 16.0, 16.0), x=-106.0, y=43.0, z=BASE_Z + 7.0)
    service_recess = _placed(Box(130.0, 96.0, 7.0), x=-64.0, z=BASE_HEIGHT - 1.0)
    base = base - belt_slot - motor_pocket - wiring_channel - service_recess

    tower_left = _placed(Box(20.0, 24.0, SHAFT_CENTER_Z - BASE_HEIGHT), x=FRONT_BEARING_X, z=(BASE_HEIGHT + SHAFT_CENTER_Z) / 2.0)
    tower_right = _placed(Box(20.0, 24.0, SHAFT_CENTER_Z - BASE_HEIGHT), x=REAR_BEARING_X, z=(BASE_HEIGHT + SHAFT_CENTER_Z) / 2.0)
    front_bearing = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + 3.5, length=14.0),
        x=FRONT_BEARING_X,
        z=SHAFT_CENTER_Z,
        label="front_bearing_carrier",
    )
    rear_bearing = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + 3.5, length=14.0),
        x=REAR_BEARING_X,
        z=SHAFT_CENTER_Z,
        label="rear_bearing_carrier",
    )
    shaft_clearance = _placed(_x_cylinder(radius=SHAFT_RADIUS + 1.2, length=BASE_LENGTH + 4.0), z=SHAFT_CENTER_Z)
    bearing_cut_front = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + BEARING_CLEARANCE, length=BEARING_WIDTH + 2.0),
        x=FRONT_BEARING_X,
        z=SHAFT_CENTER_Z,
    )
    bearing_cut_rear = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + BEARING_CLEARANCE, length=BEARING_WIDTH + 2.0),
        x=REAR_BEARING_X,
        z=SHAFT_CENTER_Z,
    )
    base_group = Compound(children=[_label(base, "drive_base_shell"), tower_left, tower_right, front_bearing, rear_bearing])
    base_group = base_group - shaft_clearance - bearing_cut_front - bearing_cut_rear
    return _label(base_group, "drive_base")


def make_service_cover():
    cover = Box(128.0, 94.0, 4.0)
    boss_positions = [(-48.0, -34.0), (48.0, -34.0), (-48.0, 34.0), (48.0, 34.0)]
    screw_cuts = [
        _placed(Cylinder(radius=M3_CLEARANCE / 2.0, height=8.0), x=x, y=y)
        for x, y in boss_positions
    ]
    for cut in screw_cuts:
        cover = cover - cut
    return _placed(_label(cover, "service_cover"), x=-64.0, z=BASE_HEIGHT + 2.5)


def make_lower_nacelle():
    inlet_ring = _open_case_ring(-136.0, NACELLE_RADIUS + 8.0, 18.0, 6.0, "large_inlet_lip")
    inlet_round_lip = _placed(_x_torus(major_radius=NACELLE_RADIUS + 3.5, minor_radius=2.5), x=-145.0, z=SHAFT_CENTER_Z, label="rounded_inlet_lip")
    front_bypass_stators = _open_vaned_ring("front_bypass_stator_cascade", -112.0, 55.0, 82.0, 8.0, 38)
    fan_case_ring = _open_case_ring(-96.0, NACELLE_RADIUS + 1.0, 8.0, 4.0, "front_fan_case_ring")
    ipc_case_ring = _open_case_ring(-42.0, 55.0, 8.0, 4.0, "intermediate_compressor_case_ring")
    hpc_case_ring = _open_case_ring(18.0, 45.0, 8.0, 4.0, "high_pressure_compressor_case_ring")
    turbine_case_ring = _open_case_ring(92.0, 59.0, 8.0, 4.0, "rear_turbine_case_ring")
    nozzle_ring = _open_case_ring(142.0, 64.0, 14.0, 4.0, "rear_nozzle_ring")
    nozzle_round_lip = _placed(_x_torus(major_radius=61.0, minor_radius=2.0), x=153.0, z=SHAFT_CENTER_Z, label="rounded_nozzle_lip")
    flange_bolts = [
        _bolt_circle(-136.0, 83.5, 36, 1.3, 2.2, "inlet_flange_bolt"),
        _bolt_circle(92.0, 56.5, 28, 1.2, 2.0, "turbine_case_bolt"),
        _bolt_circle(142.0, 62.0, 24, 1.2, 2.0, "nozzle_flange_bolt"),
    ]
    lower_keel = _placed(Box(288.0, 10.0, 7.0), x=2.0, z=SHAFT_CENTER_Z - 73.0, label="lower_cutaway_keel")
    left_cut_edge = _placed(Box(278.0, 5.0, 6.0), x=2.0, y=-NACELLE_RADIUS + 6.0, z=SHAFT_CENTER_Z + 2.0, label="left_section_edge")
    right_cut_edge = _placed(Box(278.0, 5.0, 6.0), x=2.0, y=NACELLE_RADIUS - 6.0, z=SHAFT_CENTER_Z + 2.0, label="right_section_edge")
    bypass_floor_left = _placed(Box(255.0, 5.0, 5.0), x=8.0, y=-42.0, z=SHAFT_CENTER_Z - 54.0, label="left_bypass_floor_rail")
    bypass_floor_right = _placed(Box(255.0, 5.0, 5.0), x=8.0, y=42.0, z=SHAFT_CENTER_Z - 54.0, label="right_bypass_floor_rail")
    mount_saddle = _placed(Box(176.0, 18.0, 8.0), x=-12.0, z=BASE_HEIGHT + 8.0, label="engine_mount_saddle")
    return _label(
        Compound(
            children=[
                inlet_ring,
                inlet_round_lip,
                front_bypass_stators,
                fan_case_ring,
                ipc_case_ring,
                hpc_case_ring,
                turbine_case_ring,
                nozzle_ring,
                nozzle_round_lip,
                *flange_bolts,
                lower_keel,
                left_cut_edge,
                right_cut_edge,
                bypass_floor_left,
                bypass_floor_right,
                mount_saddle,
            ]
        ),
        "lower_nacelle_frame",
    )


def make_cutaway_upper_shell():
    cutaway_y = -72.0
    top_spine = _placed(Box(266.0, 9.0, 7.0), x=0.0, y=cutaway_y, z=SHAFT_CENTER_Z + 78.0, label="upper_nacelle_spine")
    inlet_top = _placed(Box(74.0, 9.0, 7.0), x=-123.0, y=cutaway_y, z=SHAFT_CENTER_Z + 87.0, label="squared_inlet_upper_lip")
    fan_case_top = _placed(Box(36.0, 9.0, 8.0), x=-103.0, y=cutaway_y, z=SHAFT_CENTER_Z + 71.0, label="front_fan_case_upper_rail")
    core_top = _placed(Box(128.0, 9.0, 7.0), x=20.0, y=cutaway_y, z=SHAFT_CENTER_Z + 47.0, label="core_case_upper_rail")
    rear_flare_top = _placed(Box(96.0, 9.0, 7.0), x=107.0, y=cutaway_y, z=SHAFT_CENTER_Z + 64.0, label="rear_nozzle_upper_rail")
    front_bulkhead = _placed(Box(7.0, 9.0, 126.0), x=-145.0, y=cutaway_y, z=SHAFT_CENTER_Z + 1.0, label="front_cutaway_bulkhead")
    rear_bulkhead = _placed(Box(7.0, 9.0, 108.0), x=151.0, y=cutaway_y, z=SHAFT_CENTER_Z, label="rear_cutaway_bulkhead")
    pylon_links = [
        _placed(Box(16.0, 8.0, 44.0), x=-28.0, y=cutaway_y, z=SHAFT_CENTER_Z + 58.0, label="upper_service_pylon_forward"),
        _placed(Box(16.0, 8.0, 38.0), x=72.0, y=cutaway_y, z=SHAFT_CENTER_Z + 56.0, label="upper_service_pylon_rear"),
    ]
    return _label(
        Compound(children=[top_spine, inlet_top, fan_case_top, core_top, rear_flare_top, front_bulkhead, rear_bulkhead, *pylon_links]),
        "upper_cutaway_outline",
    )


def make_combustor_chamber():
    chamber = _placed(_x_cylinder(32.0, 42.0), x=54.0, z=SHAFT_CENTER_Z, label="annular_combustor_body")
    chamber_cut = _placed(_x_cylinder(21.0, 44.0), x=54.0, z=SHAFT_CENTER_Z)
    chamber = chamber - chamber_cut
    fuel_bosses = []
    for index in range(12):
        angle = index * 360.0 / 12.0
        y = 35.0 * cos(radians(angle))
        z = SHAFT_CENTER_Z + 35.0 * sin(radians(angle))
        boss = _placed(_x_cylinder(2.4, 10.0), x=54.0, y=y, z=z, label=f"combustor_fuel_nozzle_{index + 1:02d}")
        fuel_bosses.append(boss)
    liner_ribs = [
        _placed(_x_cylinder(34.0, 3.0), x=35.0, z=SHAFT_CENTER_Z, label="combustor_front_rib"),
        _placed(_x_cylinder(34.0, 3.0), x=73.0, z=SHAFT_CENTER_Z, label="combustor_rear_rib"),
    ]
    liner_tiles = []
    for ring_index, x in enumerate((43.0, 54.0, 65.0), start=1):
        for index in range(18):
            angle = index * 360.0 / 18.0 + (ring_index % 2) * 10.0
            y, z = _polar_point(31.0, angle)
            liner_tiles.append(
                _placed(
                    _x_cylinder(1.1, 1.7),
                    x=x,
                    y=y,
                    z=z,
                    label=f"combustor_liner_tile_{ring_index}_{index + 1:02d}",
                )
            )
    return _label(Compound(children=[chamber, *fuel_bosses, *liner_ribs, *liner_tiles]), "combustor_chamber")


def make_external_pipes():
    upper_fuel_line = _pipe_run(
        [(-95.0, -68.0, 171.0), (-32.0, -68.0, 171.0), (28.0, -62.0, 162.0), (82.0, -56.0, 158.0)],
        3.0,
        "upper_fuel_line",
    )
    lower_oil_line = _pipe_run(
        [(-88.0, 66.0, 76.0), (-16.0, 66.0, 74.0), (48.0, 58.0, 83.0), (120.0, 52.0, 94.0)],
        3.0,
        "lower_oil_line",
    )
    sensor_boxes = [
        _placed(Box(12.0, 8.0, 8.0), x=-18.0, y=-68.0, z=171.0, label="sensor_box_forward"),
        _placed(Box(12.0, 8.0, 8.0), x=72.0, y=52.0, z=94.0, label="sensor_box_rear"),
    ]
    return _label(Compound(children=[upper_fuel_line, lower_oil_line, *sensor_boxes]), "external_pipe_detail")


def make_afterburner_nozzle():
    flame_holder = _placed(_x_cylinder(44.0, 6.0), x=138.0, z=SHAFT_CENTER_Z, label="afterburner_flameholder_ring")
    flame_cut = _placed(_x_cylinder(30.0, 8.0), x=138.0, z=SHAFT_CENTER_Z)
    flame_holder = flame_holder - flame_cut
    petals = []
    for index in range(12):
        angle = index * 360.0 / 12.0
        petal = _radial_blade(
            x=150.0,
            inner_radius=42.0,
            outer_radius=70.0,
            chord=8.0,
            thickness_x=22.0,
            angle_deg=angle,
            sweep_deg=0.0,
            label=f"variable_nozzle_petal_{index + 1:02d}",
        )
        petals.append(petal)
    tail_cone = _placed(
        Rot(0, 90, 0) * Cone(bottom_radius=52.0, top_radius=37.0, height=22.0),
        x=146.0,
        z=SHAFT_CENTER_Z,
        label="tapered_exhaust_nozzle",
    )
    tail_cut = _placed(_x_cylinder(34.0, 26.0), x=146.0, z=SHAFT_CENTER_Z)
    return _label(Compound(children=[flame_holder, tail_cone - tail_cut, *petals]), "afterburner_nozzle")


def make_rotor_stack():
    shaft = _placed(_x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH), z=SHAFT_CENTER_Z, label="8mm_shaft_reference")
    spinner = _placed(
        Rot(0, -90, 0) * Cone(bottom_radius=27.0, top_radius=4.0, height=34.0),
        x=-138.0,
        z=SHAFT_CENTER_Z,
        label="spinner_nose_cone",
    )
    spinner_back = _placed(
        Rot(0, -90, 0) * Cone(bottom_radius=31.0, top_radius=23.0, height=18.0),
        x=-118.0,
        z=SHAFT_CENTER_Z,
        label="spinner_back_fairing",
    )
    spinner_stripes = [
        _placed(_x_torus(major_radius=23.0, minor_radius=0.9), x=-129.0, z=SHAFT_CENTER_Z, label="spinner_stripe_front"),
        _placed(_x_torus(major_radius=27.0, minor_radius=0.9), x=-121.0, z=SHAFT_CENTER_Z, label="spinner_stripe_rear"),
    ]
    fan = _fan_stage(-112.0, FAN_RADIUS, 13.0, 24, "large_front_swept_fan")
    compressor_stages = [
        _stage_disk(-78.0, 46.0, 6.5, 20, "ipc_stage_1", hub_radius=17.0, blade_sweep=18.0),
        _stage_disk(-64.0, 44.0, 6.5, 20, "ipc_stage_2", hub_radius=16.5, blade_sweep=17.0),
        _stage_disk(-50.0, 42.0, 6.5, 20, "ipc_stage_3", hub_radius=16.0, blade_sweep=16.0),
        _stage_disk(-36.0, 40.0, 6.0, 20, "ipc_stage_4", hub_radius=15.5, blade_sweep=15.0),
        _stage_disk(-22.0, 38.0, 6.0, 20, "ipc_stage_5", hub_radius=15.0, blade_sweep=14.0),
        _stage_disk(-8.0, 36.0, 6.0, 20, "ipc_stage_6", hub_radius=14.5, blade_sweep=13.0),
        _stage_disk(6.0, 34.5, 6.0, 20, "ipc_stage_7", hub_radius=14.0, blade_sweep=12.0),
        _stage_disk(22.0, 33.0, 5.5, 18, "hpc_stage_1", hub_radius=13.5, blade_sweep=12.0),
        _stage_disk(34.0, 31.0, 5.5, 18, "hpc_stage_2", hub_radius=13.0, blade_sweep=11.0),
        _stage_disk(46.0, 29.5, 5.5, 18, "hpc_stage_3", hub_radius=12.5, blade_sweep=10.0),
        _stage_disk(58.0, 28.0, 5.5, 18, "hpc_stage_4", hub_radius=12.0, blade_sweep=9.0),
        _stage_disk(70.0, 27.0, 5.5, 18, "hpc_stage_5", hub_radius=12.0, blade_sweep=8.0),
    ]
    turbine_stages = [
        _stage_disk(84.0, 38.0, 6.5, 20, "n3_high_pressure_turbine", hub_radius=16.0, blade_sweep=-18.0),
        _stage_disk(98.0, 44.0, 7.0, 20, "n2_intermediate_turbine", hub_radius=18.0, blade_sweep=-20.0),
        _stage_disk(112.0, 50.0, 7.5, 22, "n1_low_pressure_turbine_1", hub_radius=20.0, blade_sweep=-22.0),
        _stage_disk(124.0, 53.0, 7.5, 22, "n1_low_pressure_turbine_2", hub_radius=20.5, blade_sweep=-23.0),
        _stage_disk(136.0, 56.0, 7.5, 22, "n1_low_pressure_turbine_3", hub_radius=21.0, blade_sweep=-24.0),
        _stage_disk(148.0, REAR_TURBINE_RADIUS, 7.5, 22, "n1_low_pressure_turbine_4", hub_radius=22.0, blade_sweep=-25.0),
    ]
    core_drums = [
        _placed(_x_cylinder(22.0, 74.0), x=-36.0, z=SHAFT_CENTER_Z, label="ipc_core_drum"),
        _placed(_x_cylinder(20.0, 62.0), x=42.0, z=SHAFT_CENTER_Z, label="hpc_core_drum"),
        _placed(_x_cylinder(25.0, 72.0), x=118.0, z=SHAFT_CENTER_Z, label="lp_turbine_core_drum"),
    ]
    rotor_pulley = _placed(
        _x_cylinder(GT2_PULLEY_DIAMETER / 2.0, GT2_PULLEY_WIDTH),
        x=-9.0,
        z=SHAFT_CENTER_Z,
        label="rotor_gt2_pulley_envelope",
    )
    return _label(
        Compound(children=[shaft, spinner, spinner_back, *spinner_stripes, fan, *compressor_stages, *turbine_stages, *core_drums, rotor_pulley]),
        "rotor_cartridge",
    )


def make_stator_ring(
    name: str,
    x_position: float,
    inner_radius: float = 34.0,
    outer_radius: float = 52.0,
    vane_count: int = 26,
):
    return _open_vaned_ring(name, x_position, inner_radius, outer_radius, 7.0, vane_count)


def make_gearbox_cluster():
    ring_gear = _gear_wheel(-52.0, -24.0, 30.0, 9.0, 36, "planetary_ring_gear_visual")
    sun_gear = _gear_wheel(-52.0, -24.0, 11.0, 12.0, 18, "n3_sun_gear_visual")
    planet_gears = []
    for index in range(4):
        angle = index * 90.0 + 45.0
        y = -24.0 + 18.0 * cos(radians(angle))
        z = SHAFT_CENTER_Z + 18.0 * sin(radians(angle))
        planet_gears.append(_gear_wheel(-52.0, y, 8.5, 8.0, 14, f"planet_gear_{index + 1}"))
    side_gear = _gear_wheel(-28.0, 20.0, 18.0, 8.0, 20, "offset_accessory_gear")
    gearcase = _placed(Box(58.0, 74.0, 54.0), x=-48.0, y=-8.0, z=SHAFT_CENTER_Z, label="open_gearbox_case_envelope")
    sight_cut = _placed(Box(62.0, 61.0, 42.0), x=-48.0, y=-8.0, z=SHAFT_CENTER_Z)
    cover_bolts = _bolt_circle(-77.0, 30.0, 16, 1.2, 2.0, "gearbox_cover_bolt")
    return _label(Compound(children=[gearcase - sight_cut, ring_gear, sun_gear, *planet_gears, side_gear, cover_bolts]), "visible_gearbox_cluster")


def make_reference_hardware():
    motor = _placed(Box(MOTOR_LENGTH, MOTOR_WIDTH, MOTOR_HEIGHT), x=-72.0, z=BASE_Z + 8.0, label="dc_gearmotor_envelope")
    motor_pulley = _placed(
        _x_cylinder(GT2_PULLEY_DIAMETER / 2.0, GT2_PULLEY_WIDTH),
        x=-72.0,
        z=BASE_HEIGHT + 18.0,
        label="motor_gt2_pulley_envelope",
    )
    belt_front = _placed(Box(70.0, GT2_BELT_WIDTH, 4.0), x=-40.5, z=BASE_HEIGHT + 18.0, label="lower_gt2_belt_run")
    belt_vertical = _placed(Box(4.0, GT2_BELT_WIDTH, SHAFT_CENTER_Z - BASE_HEIGHT - 18.0), x=-9.0, z=(SHAFT_CENTER_Z + BASE_HEIGHT + 18.0) / 2.0, label="vertical_gt2_belt_run")
    bearings = [
        _placed(_x_cylinder(BEARING_OD / 2.0, BEARING_WIDTH), x=FRONT_BEARING_X, z=SHAFT_CENTER_Z, label="front_608_bearing_envelope"),
        _placed(_x_cylinder(BEARING_OD / 2.0, BEARING_WIDTH), x=REAR_BEARING_X, z=SHAFT_CENTER_Z, label="rear_608_bearing_envelope"),
    ]
    return _label(Compound(children=[motor, motor_pulley, belt_front, belt_vertical, *bearings]), "reference_hardware")


def build_printable_parts():
    return {
        "drive_base": make_base(),
        "service_cover": make_service_cover(),
        "lower_nacelle_shell": make_lower_nacelle(),
        "cutaway_upper_shell": make_cutaway_upper_shell(),
        "rotor_cartridge": make_rotor_stack(),
        "gearbox_cluster": make_gearbox_cluster(),
        "combustor_chamber": make_combustor_chamber(),
        "external_pipe_detail": make_external_pipes(),
        "afterburner_nozzle": make_afterburner_nozzle(),
        "stator_front": make_stator_ring("stator_front", -66.0, 36.0, 57.0, 30),
        "stator_mid": make_stator_ring("stator_mid", 24.0, 27.0, 45.0, 28),
        "stator_rear": make_stator_ring("stator_rear", 108.0, 35.0, 62.0, 32),
    }


def gen_step():
    parts = build_printable_parts()
    hardware = make_reference_hardware()
    return _label(Compound(children=[*parts.values(), hardware]), "powered_cutaway_turbine")


def export_printable_parts(output_dir: str | Path = "models/parts"):
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    generated = {}
    for name, part in build_printable_parts().items():
        target = output_path / f"{name}.step"
        export_step(part, str(target))
        generated[name] = target
    return generated


def export_printable_stls(output_dir: str | Path = "models/stl"):
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    generated = {}
    for name, part in build_printable_parts().items():
        target = output_path / f"{name}.stl"
        export_stl(part, str(target))
        generated[name] = target
    return generated


def export_assembly(path: str | Path = "models/turbine_assembly.step"):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    export_step(gen_step(), str(target))
    return target


if __name__ == "__main__":
    assembly = export_assembly()
    parts = export_printable_parts()
    stls = export_printable_stls()
    print(f"Assembly: {assembly}")
    for name, path in parts.items():
        print(f"{name}: {path}")
    for name, path in stls.items():
        print(f"{name}_stl: {path}")
