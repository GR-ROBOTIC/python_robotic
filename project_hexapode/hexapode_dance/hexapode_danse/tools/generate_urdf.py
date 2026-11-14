"""Génère urdf/hexapod.urdf à partir de hexapod/config.py.

Usage : python tools/generate_urdf.py
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from hexapod import config as C  # noqa: E402


def box_inertia(m, x, y, z):
    return m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12


def cyl_x_inertia(m, r, l):
    side = m * (3 * r * r + l * l) / 12
    return m * r * r / 2, side, side


def inertial(m, xyz, I):
    return (f'    <inertial>\n      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
            f'      <mass value="{m}"/>\n'
            f'      <inertia ixx="{I[0]:.6e}" ixy="0" ixz="0" iyy="{I[1]:.6e}" iyz="0" izz="{I[2]:.6e}"/>\n'
            f'    </inertial>\n')


def cylinder_x(length, radius, material, collision=True):
    geo = f'<cylinder length="{length}" radius="{radius}"/>'
    org = f'<origin xyz="{length / 2:.4f} 0 0" rpy="0 {math.pi / 2:.5f} 0"/>'
    s = f'    <visual>\n      {org}\n      <geometry>{geo}</geometry>\n      <material name="{material}"/>\n    </visual>\n'
    if collision:
        s += f'    <collision>\n      {org}\n      <geometry>{geo}</geometry>\n    </collision>\n'
    return s


def limb(name, length, mass, material, foot=False):
    s = f'  <link name="{name}">\n'
    s += inertial(mass, f"{length / 2:.4f} 0 0", cyl_x_inertia(mass, C.LEG_RADIUS, length))
    s += cylinder_x(length, C.LEG_RADIUS, material)
    # petite « articulation » visible en début de segment
    s += (f'    <visual>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
          f'      <geometry><sphere radius="{C.LEG_RADIUS * 1.5}"/></geometry>\n'
          f'      <material name="joint"/>\n    </visual>\n')
    if foot:
        geo = f'<sphere radius="{C.FOOT_RADIUS}"/>'
        org = f'<origin xyz="{length} 0 0" rpy="0 0 0"/>'
        s += f'    <visual>\n      {org}\n      <geometry>{geo}</geometry>\n      <material name="foot"/>\n    </visual>\n'
        s += f'    <collision>\n      {org}\n      <geometry>{geo}</geometry>\n    </collision>\n'
    s += '  </link>\n'
    return s


def joint(name, parent, child, xyz, rpy, axis, seg):
    lo, hi = C.JOINT_LIMITS[seg]
    return (f'  <joint name="{name}" type="revolute">\n'
            f'    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
            f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n    <axis xyz="{axis}"/>\n'
            f'    <limit lower="{lo}" upper="{hi}" effort="{C.EFFORT}" velocity="{C.VELOCITY}"/>\n'
            f'    <dynamics damping="0.05" friction="0.0"/>\n  </joint>\n')


def build():
    L, W, T = C.BODY_LENGTH, C.BODY_WIDTH, C.BODY_THICK
    out = ['<?xml version="1.0"?>\n',
           '<!-- Généré par tools/generate_urdf.py : ne pas éditer à la main -->\n',
           '<robot name="hexapod">\n',
           '  <material name="body"><color rgba="0.15 0.15 0.18 1"/></material>\n',
           '  <material name="head"><color rgba="0.85 0.2 0.15 1"/></material>\n',
           '  <material name="coxa"><color rgba="0.95 0.55 0.1 1"/></material>\n',
           '  <material name="femur"><color rgba="0.25 0.25 0.28 1"/></material>\n',
           '  <material name="tibia"><color rgba="0.95 0.55 0.1 1"/></material>\n',
           '  <material name="joint"><color rgba="0.7 0.7 0.75 1"/></material>\n',
           '  <material name="foot"><color rgba="0.1 0.1 0.1 1"/></material>\n']

    # Corps : boîte + « tête » rouge à l'avant (pour voir le sens de marche)
    out.append('  <link name="base_link">\n')
    out.append(inertial(C.BODY_MASS, "0 0 0", box_inertia(C.BODY_MASS, L, W, T)))
    out.append(f'    <visual>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               f'      <geometry><box size="{L} {W} {T}"/></geometry>\n'
               f'      <material name="body"/>\n    </visual>\n')
    out.append(f'    <visual>\n      <origin xyz="{L / 2} 0 0.005" rpy="0 0 0"/>\n'
               f'      <geometry><sphere radius="0.03"/></geometry>\n'
               f'      <material name="head"/>\n    </visual>\n')
    out.append(f'    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               f'      <geometry><box size="{L} {W} {T}"/></geometry>\n    </collision>\n')
    out.append('  </link>\n')

    for name, x, y, ang in C.LEGS:
        a = ang * C.DEG
        out.append(joint(f"{name}_coxa", "base_link", f"{name}_coxa_link",
                         f"{x} {y} 0", f"0 0 {a:.6f}", "0 0 1", "coxa"))
        out.append(limb(f"{name}_coxa_link", C.COXA_LEN, C.COXA_MASS, "coxa"))
        out.append(joint(f"{name}_femur", f"{name}_coxa_link", f"{name}_femur_link",
                         f"{C.COXA_LEN} 0 0", "0 0 0", "0 1 0", "femur"))
        out.append(limb(f"{name}_femur_link", C.FEMUR_LEN, C.FEMUR_MASS, "femur"))
        out.append(joint(f"{name}_tibia", f"{name}_femur_link", f"{name}_tibia_link",
                         f"{C.FEMUR_LEN} 0 0", "0 0 0", "0 1 0", "tibia"))
        out.append(limb(f"{name}_tibia_link", C.TIBIA_LEN, C.TIBIA_MASS, "tibia", foot=True))

    out.append('</robot>\n')
    return "".join(out)


if __name__ == "__main__":
    path = Path(C.URDF_PATH)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build(), encoding="utf-8")
    print(f"URDF écrit : {path}  ({len(C.JOINT_NAMES)} articulations)")
