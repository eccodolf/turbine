from __future__ import annotations

from math import cos, radians, sin
from pathlib import Path

from build123d import Box, Compound, Cone, Cylinder, Location, Rot, export_step, export_stl

# Coordinate convention:
# X is the turbine shaft axis, Y is lateral width, Z is vertical/up.
# Assembly origin sits at the center of the display base footprint.

PRINTER_BED = (325.0, 325.0, 350.0)

NACELLE_OD = 160.0
NACELLE_LENGTH = 270.0
NACELLE_WALL = 4.0
NACELLE_RADIUS = NACELLE_OD / 2.0
SHAFT_CENTER_Z = 125.0

BASE_LENGTH = 300.0
BASE_WIDTH = 140.0
BASE_HEIGHT = 42.0
BASE_Z = BASE_HEIGHT / 2.0

SHAFT_DIAMETER = 8.0
SHAFT_RADIUS = SHAFT_DIAMETER / 2.0
SHAFT_LENGTH = 300.0
BEARING_OD = 22.0
BEARING_WIDTH = 7.0
BEARING_CLEARANCE = 0.35
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


def _placed(shape, x: float = 0.0, y: float = 0.0, z: float = 0.0, label: str | None = None):
    placed = Location((x, y, z)) * shape
    return _label(placed, label) if label else placed


def _blade(length: float, width: float, thickness: float, angle_deg: float, radius: float, label: str):
    blade = Box(length, thickness, width)
    blade = Rot(0, 0, angle_deg) * blade
    y = radius * cos(radians(angle_deg))
    z = SHAFT_CENTER_Z + radius * sin(radians(angle_deg))
    return _placed(blade, y=y, z=z, label=label)


def _stage_disk(x: float, outer_radius: float, thickness: float, blade_count: int, label: str):
    hub = _placed(_x_cylinder(radius=12.0, length=thickness), x=x, z=SHAFT_CENTER_Z, label=f"{label}_hub")
    ring = _placed(
        _x_cylinder(radius=outer_radius * 0.42, length=thickness * 0.7),
        x=x,
        z=SHAFT_CENTER_Z,
        label=f"{label}_inner_disk",
    )
    blades = []
    for index in range(blade_count):
        angle = index * 360.0 / blade_count
        blade = _blade(
            length=thickness,
            width=outer_radius * 0.55,
            thickness=max(4.0, outer_radius * 0.08),
            angle_deg=angle + 18.0,
            radius=outer_radius * 0.58,
            label=f"{label}_blade_{index + 1:02d}",
        )
        blades.append(_placed(blade, x=x))
    return _label(Compound(children=[hub, ring, *blades]), label)


def make_base():
    base = Box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
    belt_slot = _placed(Box(38.0, 34.0, BASE_HEIGHT + 4.0), x=-8.0, z=BASE_Z + 5.0)
    motor_pocket = _placed(Box(MOTOR_LENGTH, MOTOR_WIDTH, MOTOR_HEIGHT), x=-72.0, z=BASE_Z + 8.0)
    wiring_channel = _placed(Box(92.0, 16.0, 16.0), x=-106.0, y=43.0, z=BASE_Z + 7.0)
    service_recess = _placed(Box(130.0, 96.0, 7.0), x=-64.0, z=BASE_HEIGHT - 1.0)
    base = base - belt_slot - motor_pocket - wiring_channel - service_recess

    tower_left = _placed(Box(20.0, 24.0, SHAFT_CENTER_Z - BASE_HEIGHT), x=-126.0, z=(BASE_HEIGHT + SHAFT_CENTER_Z) / 2.0)
    tower_right = _placed(Box(20.0, 24.0, SHAFT_CENTER_Z - BASE_HEIGHT), x=126.0, z=(BASE_HEIGHT + SHAFT_CENTER_Z) / 2.0)
    front_bearing = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + 3.5, length=14.0),
        x=-126.0,
        z=SHAFT_CENTER_Z,
        label="front_bearing_carrier",
    )
    rear_bearing = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + 3.5, length=14.0),
        x=126.0,
        z=SHAFT_CENTER_Z,
        label="rear_bearing_carrier",
    )
    shaft_clearance = _placed(_x_cylinder(radius=SHAFT_RADIUS + 1.2, length=BASE_LENGTH + 4.0), z=SHAFT_CENTER_Z)
    bearing_cut_front = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + BEARING_CLEARANCE, length=BEARING_WIDTH + 2.0),
        x=-126.0,
        z=SHAFT_CENTER_Z,
    )
    bearing_cut_rear = _placed(
        _x_cylinder(radius=BEARING_OD / 2.0 + BEARING_CLEARANCE, length=BEARING_WIDTH + 2.0),
        x=126.0,
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


def _nacelle_shell_segment(label: str, keep_lower: bool):
    outer = _placed(_x_cylinder(NACELLE_RADIUS, NACELLE_LENGTH), z=SHAFT_CENTER_Z, label=f"{label}_outer")
    inner = _placed(_x_cylinder(NACELLE_RADIUS - NACELLE_WALL, NACELLE_LENGTH + 4.0), z=SHAFT_CENTER_Z, label=f"{label}_inner_cut")
    shell = outer - inner
    if keep_lower:
        cut_upper = _placed(Box(NACELLE_LENGTH + 8.0, NACELLE_OD + 8.0, NACELLE_RADIUS + 4.0), z=SHAFT_CENTER_Z + NACELLE_RADIUS / 2.0)
        shell = shell - cut_upper
    else:
        cut_lower = _placed(Box(NACELLE_LENGTH + 8.0, NACELLE_OD + 8.0, NACELLE_RADIUS + 4.0), z=SHAFT_CENTER_Z - NACELLE_RADIUS / 2.0)
        cutaway_window = _placed(
            Box(NACELLE_LENGTH + 18.0, NACELLE_RADIUS + 42.0, NACELLE_OD + 18.0),
            y=NACELLE_RADIUS * 0.48,
            z=SHAFT_CENTER_Z + NACELLE_RADIUS * 0.28,
        )
        shell = shell - cut_lower - cutaway_window
    inlet_lip = _placed(_x_cylinder(NACELLE_RADIUS + 4.0, 10.0), x=-NACELLE_LENGTH / 2.0, z=SHAFT_CENTER_Z)
    inlet_cut = _placed(_x_cylinder(NACELLE_RADIUS - 11.0, 12.0), x=-NACELLE_LENGTH / 2.0, z=SHAFT_CENTER_Z)
    nozzle = _placed(
        Rot(0, 90, 0) * Cone(bottom_radius=NACELLE_RADIUS - 8.0, top_radius=NACELLE_RADIUS - 24.0, height=26.0),
        x=NACELLE_LENGTH / 2.0 + 8.0,
        z=SHAFT_CENTER_Z,
    )
    nozzle_cut = _placed(
        Rot(0, 90, 0) * Cone(bottom_radius=NACELLE_RADIUS - 17.0, top_radius=NACELLE_RADIUS - 32.0, height=30.0),
        x=NACELLE_LENGTH / 2.0 + 8.0,
        z=SHAFT_CENTER_Z,
    )
    shell = Compound(children=[shell, inlet_lip - inlet_cut, nozzle - nozzle_cut])
    return _label(shell, label)


def make_lower_nacelle():
    return _nacelle_shell_segment("lower_nacelle_shell", keep_lower=True)


def make_cutaway_upper_shell():
    return _nacelle_shell_segment("cutaway_upper_shell", keep_lower=False)


def make_rotor_stack():
    shaft = _placed(_x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH), z=SHAFT_CENTER_Z, label="8mm_shaft_reference")
    fan = _stage_disk(-112.0, 64.0, 13.0, 16, "front_fan")
    compressor_1 = _stage_disk(-52.0, 47.0, 10.0, 14, "compressor_stage_1")
    compressor_2 = _stage_disk(0.0, 40.0, 10.0, 14, "compressor_stage_2")
    turbine_1 = _stage_disk(58.0, 43.0, 11.0, 12, "turbine_stage_1")
    turbine_2 = _stage_disk(104.0, 38.0, 11.0, 12, "turbine_stage_2")
    rotor_pulley = _placed(
        _x_cylinder(GT2_PULLEY_DIAMETER / 2.0, GT2_PULLEY_WIDTH),
        x=-9.0,
        z=SHAFT_CENTER_Z,
        label="rotor_gt2_pulley_envelope",
    )
    return _label(Compound(children=[shaft, fan, compressor_1, compressor_2, turbine_1, turbine_2, rotor_pulley]), "rotor_cartridge")


def make_stator_ring(name: str, x_position: float):
    outer = _placed(_x_cylinder(62.0, 8.0), x=x_position, z=SHAFT_CENTER_Z, label=f"{name}_outer")
    inner_cut = _placed(_x_cylinder(46.0, 10.0), x=x_position, z=SHAFT_CENTER_Z)
    ring = outer - inner_cut
    vanes = []
    for index in range(10):
        angle = index * 36.0
        vane = _blade(length=8.0, width=22.0, thickness=3.6, angle_deg=angle - 16.0, radius=53.0, label=f"{name}_vane_{index + 1:02d}")
        vanes.append(_placed(vane, x=x_position))
    return _label(Compound(children=[ring, *vanes]), name)


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
        _placed(_x_cylinder(BEARING_OD / 2.0, BEARING_WIDTH), x=-126.0, z=SHAFT_CENTER_Z, label="front_608_bearing_envelope"),
        _placed(_x_cylinder(BEARING_OD / 2.0, BEARING_WIDTH), x=126.0, z=SHAFT_CENTER_Z, label="rear_608_bearing_envelope"),
    ]
    return _label(Compound(children=[motor, motor_pulley, belt_front, belt_vertical, *bearings]), "reference_hardware")


def build_printable_parts():
    return {
        "drive_base": make_base(),
        "service_cover": make_service_cover(),
        "lower_nacelle_shell": make_lower_nacelle(),
        "cutaway_upper_shell": make_cutaway_upper_shell(),
        "rotor_cartridge": make_rotor_stack(),
        "stator_front": make_stator_ring("stator_front", -82.0),
        "stator_mid": make_stator_ring("stator_mid", -24.0),
        "stator_rear": make_stator_ring("stator_rear", 84.0),
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
