#!/usr/bin/env python3
"""Build the Marvin Pro MuJoCo assets from the robot's own description package.

Source of truth: robot_description/marvin_description/ in this repo (a vendored
copy of the package downloaded from the real robot controller 2026-09-08;
ground-truth mount geometry in robot_description/robot_runtime_summary.yaml).
Override with --src-desc if you have a fresher description package elsewhere.

This script:
  1. copies the STL meshes into assets/marvinpro/meshes/ (lowercase names)
  2. adapts marvin_pro_sim.xml:
     - rewrites mesh file paths to the local copies (the source package's torso
       mesh path meshes/base/ is stale; only meshes/base/new/ exists on disk)
     - injects a simplified 2-finger parallel gripper into each flange frame
       (real gripper identified as the KLM OmniGripper, DM4310-2EC motor —
       see robot_runtime_summary.yaml and the Apex docs; horizontal jaws along
       flange Y, flange->claw tip 0.18 m as measured on the real robot)
     - strips the source "home" keyframe (SCENE_XML defines the complete
       keyframe for the assembled scene, cones included)
  3. writes scene.xml (floor/lights, measured work table, white target frame,
     two free red cones for Stack_two_cones; includes the robot model)

Outputs:
  assets/marvinpro/marvin_pro_dual_gripper.xml
  assets/marvinpro/scene.xml
"""

import argparse
import re
import shutil
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = SIM_ROOT / "assets" / "marvinpro"
MESH_DIR = ASSET_DIR / "meshes"

DEFAULT_SRC_DESC = SIM_ROOT / "robot_description" / "marvin_description"

# ---------------------------------------------------------------------------
# Simplified KLM OmniGripper (DM4310-2EC) per arm, mounted in the flange
# frame. Dimensions measured on the real robot 2026-09-10 (tape):
#   max jaw opening 0.08 m -> 0..0.04 m per finger (0 = closed)
#   housing 0.14 m along flange Y (opening axis)
#   crank/motor pivot axis ~0.08 m from flange along +Z (user estimate)
#   flange -> claw tip 0.18 m (cross-checked against eef poses + video:
#   consistent with table top z ~= 0.73 m; the old 0.10 m was wrong)
# Dataset/policy convention: 0 = fully open, 1 = fully closed (driver maps
# cmd -> motor 0..1.6 rad, MIT Kp=3.0 Kd=0.12). Sim joint position =
# per-finger offset = gap/2, so map policy u -> ctrl 0.04*(1-u). Mid-stroke
# linearity is an approximation (crank-slider curve not yet calibrated).
# ---------------------------------------------------------------------------
GRIPPER_STROKE = 0.04
GRIPPER_FORCE = 20.0
GRIPPER_OPEN_HOME = 0.04  # fully open (motor 0 rad power-on state)

# Home pose of the 14 arm joints (from the source MJCF) plus the 4 gripper
# slide joints (open), in joint document order: arm_L, gripper_L, arm_R,
# gripper_R. The complete scene keyframe is defined in SCENE_XML below.
ROBOT_HOME_QPOS = (
    f"1.7 -1.1 -1.1 -2 -0.37 0.13 0.55 {GRIPPER_OPEN_HOME} {GRIPPER_OPEN_HOME} "
    f"-1.7 -1.1 1.1 -2 0.37 0.13 -0.55 {GRIPPER_OPEN_HOME} {GRIPPER_OPEN_HOME}"
)

def claw_plate_mesh() -> tuple[str, str]:
    """Inline MJCF mesh of one claw plate: a y-thin prism whose X-Z profile
    tapers to a point. Local z 0.01..0.08 (tip), inner (gripping) face at y=0.
    Profile estimated from the Stack_two_cones video; refine with calipers."""
    profile = [(-0.015, 0.01), (0.015, 0.01), (0.015, 0.035), (0.0, 0.08), (-0.015, 0.035)]
    n = len(profile)
    verts = [f"{x:.4f} {y:.4f} {z:.4f}" for y in (-0.004, 0.0) for x, z in profile]
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [f"{i} {j} {j + n}", f"{i} {j + n} {i + n}"]      # side walls
    faces += [f"0 {i + 1} {i}" for i in range(1, n - 1)]            # y=-0.004 cap
    faces += [f"{n} {n + i} {n + i + 1}" for i in range(1, n - 1)]  # y=0 cap
    return " ".join(verts), " ".join(faces)


_claw_v, _claw_f = claw_plate_mesh()

GRIPPER_BODY = """
        <!-- KLM OmniGripper: flange->pivot 0.08 m is a CYLINDER of the flange
             cross-section (dia ~0.078 from the Link7 mesh, per user 2026-09-10),
             then the housing 0.14 m (flange Y, opening axis); claw tips at
             flange+0.18. Webcam lump is a cosmetic placeholder. -->
        <body name="gripper_base_{s}" pos="0 0 0.005">
          <inertial pos="0 0 0.05" mass="0.5" diaginertia="0.001 0.0003 0.001"/>
          <geom type="cylinder" size="0.039 0.0375" pos="0 0 0.0425" class="grip_col" rgba="0.15 0.15 0.16 1"/>
          <geom type="box" size="0.033 0.07 0.02" pos="0 0 0.09" class="grip_col" rgba="0.06 0.06 0.07 1"/>
          <geom type="box" size="0.008 0.02 0.015" pos="-0.039 0 0.09" contype="0" conaffinity="0" rgba="0.06 0.06 0.07 1"/>
          <body name="claw_a_{s}" pos="0 0 0.095">
            <joint name="gripper_a_{s}" type="slide" axis="0 -1 0" range="0 {stroke}" damping="1" armature="0.001"/>
            <inertial pos="0 -0.012 0.04" mass="0.08" diaginertia="3e-5 3e-5 1e-5"/>
            <geom type="box" size="0.015 0.0125 0.0125" pos="0 -0.0125 0.0125" class="grip_col" rgba="0.06 0.06 0.07 1"/>
            <geom type="mesh" mesh="claw_plate" class="grip_col" rgba="0.72 0.73 0.76 1"/>
          </body>
          <body name="claw_b_{s}" pos="0 0 0.095">
            <joint name="gripper_b_{s}" type="slide" axis="0 1 0" range="0 {stroke}" damping="1" armature="0.001"/>
            <inertial pos="0 0.012 0.04" mass="0.08" diaginertia="3e-5 3e-5 1e-5"/>
            <geom type="box" size="0.015 0.0125 0.0125" pos="0 0.0125 0.0125" class="grip_col" rgba="0.06 0.06 0.07 1"/>
            <geom type="mesh" mesh="claw_plate" quat="0 0 0 1" class="grip_col" rgba="0.72 0.73 0.76 1"/>
          </body>
        </body>"""

GRIPPER_ACTUATOR = (
    '    <position joint="gripper_a_{s}" name="gripper_a_{s}_act" kp="40" '
    'forcerange="-{force} {force}" ctrlrange="0 {stroke}"/>\n'
    '    <position joint="gripper_b_{s}" name="gripper_b_{s}_act" kp="40" '
    'forcerange="-{force} {force}" ctrlrange="0 {stroke}"/>'
)

MESH_RENAMES = {
    **{f"../meshes/m6/{n}_L.STL": f"meshes/{n.lower()}_l.stl" for n in ["Base"] + [f"Link{i}" for i in range(1, 8)]},
    **{f"../meshes/m6/{n}_R.STL": f"meshes/{n.lower()}_r.stl" for n in ["Base"] + [f"Link{i}" for i in range(1, 8)]},
    "../meshes/base/ZJ_Robot_link.STL": "meshes/zj_robot_link.stl",
}

# ---------------------------------------------------------------------------
# Stack_two_cones environment (measured 2026-09-09 on the real setup):
#   - base_link origin -> near table edge = 0.22 m (+X)
#   - table width along X (near -> far edge) = 0.75 m; length along Y set
#     arbitrarily large (2 m) for now
#   - table-top height 0.76 m NOT measured yet (kept from the placeholder)
#   - table-top height z = 0.73 m: ESTIMATE from eef poses + video (tip
#     heights vs cone contacts; consistent across anchors to ~±1 cm),
#     not yet tape-measured
# Colors sampled (median RGB) from the quad-tile video of episode
# my_bag-26-09-05-08-22-17: green mat = (42, 120, 96), red cone = (174, 0, 43).
# Cone diameters and frame widths measured with calipers 2026-09-10
# (base 0.075 m / top 0.025 m dia; frame outer 0.13 m / inner 0.09 m).
# Cone height 0.08 m is still an ESTIMATE from video.
# ---------------------------------------------------------------------------
TABLE_EDGE_X = 0.22
TABLE_DEPTH = 0.75          # along X
TABLE_LENGTH = 2.0          # along Y (arbitrary, per user)
TABLE_TOP_Z = 0.73          # eef+video estimate (see header), confirm by tape
TABLE_RGBA = "0.165 0.471 0.376 1"
CONE_RGBA = "0.682 0.008 0.169 1"
CONE_BASE_R = 0.0375
CONE_TOP_R = 0.0125
CONE_HEIGHT = 0.08
CONE_MASS = 0.04
CONE_SEGMENTS = 32
FRAME_OUTER = 0.13          # white tape square, outer side length
FRAME_STRIP = 0.02          # tape strip width ((outer - inner) / 2)
CONE1_XY = (0.45, 0.15)     # initial cone positions on the table (x, y)
CONE2_XY = (0.58, -0.13)


def cone_mesh() -> tuple[str, str]:
    """Inline MJCF mesh (vertex/face strings) of a z-up frustum, base at z=0."""
    import numpy as np

    verts = [
        f"{r * np.cos(2 * np.pi * i / CONE_SEGMENTS):.6f} "
        f"{r * np.sin(2 * np.pi * i / CONE_SEGMENTS):.6f} {z:.6f}"
        for z, r in ((0.0, CONE_BASE_R), (CONE_HEIGHT, CONE_TOP_R))
        for i in range(CONE_SEGMENTS)
    ]
    bot_c, top_c = 2 * CONE_SEGMENTS, 2 * CONE_SEGMENTS + 1
    verts += ["0 0 0", f"0 0 {CONE_HEIGHT:.6f}"]
    faces = []
    for i in range(CONE_SEGMENTS):
        j = (i + 1) % CONE_SEGMENTS
        t_i, t_j = CONE_SEGMENTS + i, CONE_SEGMENTS + j
        faces += [f"{i} {j} {t_j}", f"{i} {t_j} {t_i}",      # side
                  f"{bot_c} {j} {i}", f"{top_c} {t_i} {t_j}"]  # bottom, top caps
    return " ".join(verts), " ".join(faces)


_cone_v, _cone_f = cone_mesh()

SCENE_XML = f"""<mujoco model="marvinpro_scene">
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-20" offwidth="1280" offheight="960"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture name="groundplane" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2"
             width="512" height="512" mark="edge" markrgb="0.8 0.8 0.8"/>
    <material name="groundplane" texture="groundplane" texrepeat="5 5" texuniform="true"
              reflectance="0.2"/>
    <!-- Frustum approximating the red training cone; base sits at local z=0. -->
    <mesh name="red_cone" vertex="{_cone_v}" face="{_cone_f}"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <!-- Work table in front of the robot (+X forward); dimensions measured
         2026-09-09, see the constants block in build_scene.py. -->
    <body name="table" pos="{TABLE_EDGE_X + TABLE_DEPTH / 2} 0 {TABLE_TOP_Z - 0.38}">
      <geom type="box" size="{TABLE_DEPTH / 2} {TABLE_LENGTH / 2} 0.02" pos="0 0 0.36" rgba="{TABLE_RGBA}"/>
      <geom type="box" size="0.03 0.03 0.36" pos="-0.32 -{TABLE_LENGTH / 2 - 0.07} 0"/>
      <geom type="box" size="0.03 0.03 0.36" pos="-0.32 {TABLE_LENGTH / 2 - 0.07} 0"/>
      <geom type="box" size="0.03 0.03 0.36" pos="0.32 -{TABLE_LENGTH / 2 - 0.07} 0"/>
      <geom type="box" size="0.03 0.03 0.36" pos="0.32 {TABLE_LENGTH / 2 - 0.07} 0"/>
    </body>
    <!-- White tape target frame on the table (stacking zone); visual-only so it
         never disturbs cone contacts. -->
    <body name="target_frame" pos="0.50 0 {TABLE_TOP_Z + 0.001}">
      <geom type="box" size="{FRAME_OUTER / 2} {FRAME_STRIP / 2} 0.0005" pos="0 {FRAME_OUTER / 2 - FRAME_STRIP / 2} 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
      <geom type="box" size="{FRAME_OUTER / 2} {FRAME_STRIP / 2} 0.0005" pos="0 -{FRAME_OUTER / 2 - FRAME_STRIP / 2} 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
      <geom type="box" size="{FRAME_STRIP / 2} {FRAME_OUTER / 2 - FRAME_STRIP} 0.0005" pos="{FRAME_OUTER / 2 - FRAME_STRIP / 2} 0 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
      <geom type="box" size="{FRAME_STRIP / 2} {FRAME_OUTER / 2 - FRAME_STRIP} 0.0005" pos="-{FRAME_OUTER / 2 - FRAME_STRIP / 2} 0 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
    </body>
    <!-- The two cones to stack (free bodies). -->
    <body name="cone_1" pos="{CONE1_XY[0]} {CONE1_XY[1]} {TABLE_TOP_Z + 0.001}">
      <freejoint/>
      <inertial pos="0 0 {CONE_HEIGHT / 2 - 0.01}" mass="{CONE_MASS}" diaginertia="2e-5 2e-5 8e-6"/>
      <geom type="mesh" mesh="red_cone" rgba="{CONE_RGBA}"/>
    </body>
    <body name="cone_2" pos="{CONE2_XY[0]} {CONE2_XY[1]} {TABLE_TOP_Z + 0.001}">
      <freejoint/>
      <inertial pos="0 0 {CONE_HEIGHT / 2 - 0.01}" mass="{CONE_MASS}" diaginertia="2e-5 2e-5 8e-6"/>
      <geom type="mesh" mesh="red_cone" rgba="{CONE_RGBA}"/>
    </body>
  </worldbody>
  <include file="marvin_pro_dual_gripper.xml"/>
  <!-- Complete home keyframe for the assembled scene (moved out of the robot
       model: MuJoCo zero-pads short keyframes, which gave the cone free joints
       invalid zero quaternions). Order follows joint document order:
       cone_1 (x y z qw qx qy qz), cone_2, then the 18 robot joints. -->
  <keyframe>
    <key name="home" qpos="{CONE1_XY[0]} {CONE1_XY[1]} {TABLE_TOP_Z + 0.001} 1 0 0 0 {CONE2_XY[0]} {CONE2_XY[1]} {TABLE_TOP_Z + 0.001} 1 0 0 0 {ROBOT_HOME_QPOS}"/>
  </keyframe>
</mujoco>
"""


def copy_meshes(src_desc: Path) -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for side in ("L", "R"):
        for n in ["Base"] + [f"Link{i}" for i in range(1, 8)]:
            src = src_desc / "meshes" / "m6" / f"{n}_{side}.STL"
            dst = MESH_DIR / f"{n.lower()}_{side.lower()}.stl"
            shutil.copyfile(src, dst)
    shutil.copyfile(src_desc / "meshes" / "base" / "new" / "ZJ_Robot_link.STL", MESH_DIR / "zj_robot_link.stl")


def adapt_model(src_model: Path) -> str:
    text = src_model.read_text()
    for old, new in MESH_RENAMES.items():
        if old not in text:
            raise RuntimeError(f"expected mesh path not found in source MJCF: {old}")
        text = text.replace(old, new)

    # Gripper collision class (real mass, collides with world and other arm).
    text = text.replace(
        '    <default class="collision">',
        '    <default class="grip_col">\n      <geom contype="1" conaffinity="1"/>\n    </default>\n    <default class="collision">',
        1,
    )

    # Claw plate mesh asset for the grippers.
    text = text.replace(
        "  </asset>",
        f'    <mesh name="claw_plate" vertex="{_claw_v}" face="{_claw_f}"/>\n  </asset>',
        1,
    )

    # Inject the gripper into each flange body, right after the tool body.
    for side, tool in (("L", "left_tool"), ("R", "right_tool")):
        # The tool body block ends with the </body> following its <site/>.
        pat = re.compile(r'(<body name="%s"[^>]*>.*?</body>)' % tool, re.S)
        m = pat.search(text)
        if not m:
            raise RuntimeError(f"tool body {tool} not found")
        gripper = GRIPPER_BODY.format(s=side, stroke=GRIPPER_STROKE)
        text = text[: m.end(1)] + gripper + text[m.end(1):]

    # Gripper actuators at the end of the actuator block.
    acts = "\n".join(
        GRIPPER_ACTUATOR.format(s=s, force=GRIPPER_FORCE, stroke=GRIPPER_STROKE) for s in ("L", "R")
    )
    text = text.replace("  </actuator>", acts + "\n  </actuator>", 1)

    # Strip the source "home" keyframe: with the two cone free bodies the
    # assembled scene has nq=32, and MuJoCo zero-pads short keyframes (invalid
    # zero quaternions for the cones). The complete keyframe is defined in
    # SCENE_XML instead; check it still matches the source home pose.
    old_key = 'qpos="1.7 -1.1 -1.1 -2 -0.37 0.13 0.55 -1.7 -1.1 1.1 -2 0.37 0.13 -0.55"'
    if old_key not in text:
        raise RuntimeError("home keyframe not found")
    text, n = re.subn(r"\s*<keyframe>.*?</keyframe>\n", "\n", text, flags=re.S)
    if n != 1:
        raise RuntimeError("expected exactly one keyframe block")
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--src-desc",
        type=Path,
        default=DEFAULT_SRC_DESC,
        help="path to the marvin_description package (default: vendored copy in this repo)",
    )
    args = parser.parse_args()
    src_desc = args.src_desc.resolve()
    if not (src_desc / "mjcf" / "marvin_pro_sim.xml").exists():
        raise SystemExit(f"marvin_pro_sim.xml not found under {src_desc}/mjcf/")
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    copy_meshes(src_desc)
    model_path = ASSET_DIR / "marvin_pro_dual_gripper.xml"
    model_path.write_text(adapt_model(src_desc / "mjcf" / "marvin_pro_sim.xml"))
    (ASSET_DIR / "scene.xml").write_text(SCENE_XML)
    print(f"source description: {src_desc}")
    print(f"wrote {model_path}")
    print(f"wrote {ASSET_DIR / 'scene.xml'}")


if __name__ == "__main__":
    main()
