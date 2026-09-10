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
     - injects the KLM OmniGripper (DM4310-2EC) into each flange frame,
       assembled from the official Tianji URDF (robot_description/official_urdf/;
       flange->claw tip 0.176 m, max opening 0.082 m; tape-verified 2026-09-10)
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
OFFICIAL_URDF = SIM_ROOT / "robot_description" / "official_urdf"

# ---------------------------------------------------------------------------
# KLM OmniGripper (DM4310-2EC) per arm, assembled from the official Tianji
# URDF (robot_description/official_urdf/Pro-m6-696.urdf, L-side subtree
# reused for both flange frames). Verified against tape measurements
# 2026-09-10:
#   model flange->claw tip 0.176 m   (tape 0.18)
#   model max opening 0.082 m        (tape 0.08) -> 0..0.04 m per finger
#   carrier/rail housing 0.149 m     (tape 0.14)
# Joint convention follows the URDF: q=0 = OPEN (matches the driver power-on
# state, motor 0 rad), q=0.04 = closed -> map policy u -> ctrl 0.04*u.
# Actuator/joint params mapped from the real MIT driver (Apex docs: Kp=3.0,
# Kd=0.12; motor range 0..1.6 rad) via the crank ratio ~40 rad/m (1.6 rad over
# 0.04 m/finger, linear approx): kp = 3.0*40^2 = 4800 N/m, damping = 0.12*1600
# = 192 N s/m, force cap ~150 N (continuous tau ~4 Nm x 40; grasp measured
# 60-76 N). Body masses are estimates (~0.8 kg total): the URDF's SolidWorks
# inertials are unreliable (e.g. a 5.4 kg wrist camera).
# ---------------------------------------------------------------------------
GRIPPER_STROKE = 0.04
GRIPPER_FORCE = 150.0
GRIPPER_KP = 4800.0
GRIPPER_DAMPING = 192.0
GRIPPER_OPEN_HOME = 0.0  # open (URDF/model zero pose)

# Home pose of the 14 arm joints (from the source MJCF) plus the 4 gripper
# slide joints (open), in joint document order: arm_L, gripper_L, arm_R,
# gripper_R. The complete scene keyframe is defined in SCENE_XML below.
ROBOT_HOME_QPOS = (
    f"1.7 -1.1 -1.1 -2 -0.37 0.13 0.55 {GRIPPER_OPEN_HOME} {GRIPPER_OPEN_HOME} "
    f"-1.7 -1.1 1.1 -2 0.37 0.13 -0.55 {GRIPPER_OPEN_HOME} {GRIPPER_OPEN_HOME}"
)

# Gripper meshes vendored from the official URDF package (L side, reused for
# both arms): official_urdf/meshes/<src> -> assets/marvinpro/meshes/gripper/<dst>
GRIPPER_MESHES = {
    "Link8_L.STL": "base.stl",
    "Arm_L9_Link.STL": "carrier.stl",
    "Arm_L10_Link.STL": "finger_a.stl",
    "Arm_L11_Link.STL": "finger_b.stl",
    "Link_Left_wrist.STL": "wrist_cam.stl",
}

GRIPPER_MESH_ASSETS = "\n".join(
    f'    <mesh name="grip_{n}" content_type="model/stl" file="meshes/gripper/{f}"/>'
    for n, f in (("base", "base.stl"), ("carrier", "carrier.stl"), ("finger_a", "finger_a.stl"),
                 ("finger_b", "finger_b.stl"), ("wrist_cam", "wrist_cam.stl"))
)

# Body frames below are baked from the URDF into the sim flange frame (the
# URDF TCP_Link sits 0.007 m beyond the sim flange: -0.095 vs -0.088 m on
# Link7 Y). Finger slide axes are carrier-frame +-z == flange -+Y.
GRIPPER_BODY = """
        <body name="gripper_base_{s}" pos="0 0 0.007">
          <inertial pos="0.0335 0.0008 0.0162" mass="0.45" diaginertia="0.00034 0.00067 0.00072"/>
          <geom type="mesh" mesh="grip_base" class="grip_col" rgba="0.12 0.12 0.13 1"/>
          <body name="wrist_cam_{s}" pos="0.06 0 0.064822" quat="0.549549 -0.549438 -0.445050 0.445022">
            <inertial pos="0.0002 0.0265 0.0015" mass="0.08" diaginertia="2e-5 2e-5 2e-5"/>
            <geom type="mesh" mesh="grip_wrist_cam" class="grip_col" rgba="0.06 0.06 0.07 1"/>
          </body>
          <body name="grip_carrier_{s}" pos="0 0 -0.031" quat="0 0 0.707107 -0.707107">
            <inertial pos="-0.0009 -0.0896 0.0003" mass="0.20" diaginertia="0.00047 0.00049 0.00021"/>
            <geom type="mesh" mesh="grip_carrier" contype="0" conaffinity="0" rgba="0.12 0.12 0.13 1"/>
            <!-- Collision: only the two slider blocks at the opening-axis
                 extremes (sized from the carrier mesh occupancy map); the
                 middle channel stays open so a gripped cone's top can enter
                 between the plates (the convex hull of the full carrier mesh
                 filled that channel and blocked lifts). -->
            <geom type="box" size="0.022 0.0335 0.0175" pos="0 -0.0815 0.0525" class="grip_col" rgba="0.12 0.12 0.13 1"/>
            <geom type="box" size="0.022 0.0335 0.0175" pos="0 -0.0815 -0.0525" class="grip_col" rgba="0.12 0.12 0.13 1"/>
            <body name="grip_finger_a_{s}" pos="0 0 0">
              <joint name="gripper_a_{s}" type="slide" axis="0 0 -1" range="0 {stroke}" damping="{damp}" armature="0.05"/>
              <inertial pos="-0.0005 -0.1427 0.0575" mass="0.03" diaginertia="2.5e-5 5e-6 2.5e-5"/>
              <geom type="mesh" mesh="grip_finger_a" class="grip_col" rgba="0.72 0.73 0.76 1"/>
            </body>
            <body name="grip_finger_b_{s}" pos="0 0 0">
              <joint name="gripper_b_{s}" type="slide" axis="0 0 1" range="0 {stroke}" damping="{damp}" armature="0.05"/>
              <inertial pos="0.0005 -0.1427 -0.0576" mass="0.03" diaginertia="2.5e-5 5e-6 2.5e-5"/>
              <geom type="mesh" mesh="grip_finger_b" class="grip_col" rgba="0.72 0.73 0.76 1"/>
            </body>
          </body>
        </body>"""

GRIPPER_ACTUATOR = (
    '    <position joint="gripper_a_{s}" name="gripper_a_{s}_act" kp="{kp}" '
    'forcerange="-{force} {force}" ctrlrange="0 {stroke}"/>\n'
    '    <position joint="gripper_b_{s}" name="gripper_b_{s}_act" kp="{kp}" '
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
#   - table-top z = 0.718 m in base_link frame: 0.80 m tape-measured from the
#     floor (2026-09-10) minus 0.082 m (floor is at base_link z=-0.082, the
#     bottom of the base mesh); matches the earlier eef+video estimate 0.73 m
# Colors sampled (median RGB) from the quad-tile video of episode
# my_bag-26-09-05-08-22-17: green mat = (42, 120, 96), red cone = (174, 0, 43).
# Cone diameters and frame widths measured with calipers 2026-09-10
# (base 0.075 m / top 0.025 m dia; frame outer 0.13 m / inner 0.09 m).
# Cone height 0.08 m is still an ESTIMATE from video.
# ---------------------------------------------------------------------------
TABLE_EDGE_X = 0.22
TABLE_DEPTH = 0.75          # along X
TABLE_LENGTH = 2.0          # along Y (arbitrary, per user)
FLOOR_Z = -0.082            # floor in base_link frame = bottom of the base mesh
TABLE_HEIGHT = 0.80         # tape-measured 2026-09-10, floor -> table top
TABLE_TOP_Z = TABLE_HEIGHT + FLOOR_Z  # 0.718 m in base_link frame
TABLE_RGBA = "0.165 0.471 0.376 1"
CONE_RGBA = "0.682 0.008 0.169 1"
CONE_BASE_R = 0.0375
CONE_TOP_R = 0.0125
CONE_HEIGHT = 0.08
CONE_MASS = 0.04
CONE_SEGMENTS = 32
# Collision shell: CONE_WALL_SEGMENTS thin boxes forming the frustum wall
# (open bottom -> cones can nest for stacking). Boxes are 8 mm thick so the
# gripper plates sink into the soft contact without tunneling through.
# solref timeconst 0.010 tuned so the simulated jaw gap at full close matches
# the real grasp feedback (sim 30.4 mm vs teleop 31-34 mm); equivalent to a
# side-wall compliance of ~9e3 N/m under ~65-76 N per-finger grip force.
CONE_WALL_SEGMENTS = 12
CONE_WALL_HALF_THICK = 0.004
CONE_WALL_SOLREF = "0.01 1"
CONE_WALL_FRICTION = "1.0 0.02 0.0005"  # slide, torsion, roll — resists in-grasp pivoting
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


def _quat_of(R) -> str:
    import numpy as np

    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(R[i, i] - R[j, j] - R[k, k] + 1, 1e-12)) * 2
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[i + 1] = 0.25 * s
        q[j + 1] = (R[j, i] + R[i, j]) / s
        q[k + 1] = (R[k, i] + R[i, k]) / s
    q = q / np.linalg.norm(q)
    return f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"


def cone_shell_geoms() -> str:
    """Collision shell of a hollow cone: CONE_WALL_SEGMENTS tilted thin boxes
    forming the frustum wall (open bottom, so cones nest for stacking), plus
    the top cap disk. Positions are local to the cone body (base at z=0)."""
    import numpy as np

    alpha = np.arctan((CONE_BASE_R - CONE_TOP_R) / CONE_HEIGHT)  # wall slope
    r_mid = (CONE_BASE_R + CONE_TOP_R) / 2
    half_len = CONE_HEIGHT / 2 / np.cos(alpha)
    half_wid = r_mid * np.tan(np.pi / CONE_WALL_SEGMENTS) * 1.1  # 10% overlap
    out = []
    for i in range(CONE_WALL_SEGMENTS):
        th = 2 * np.pi * i / CONE_WALL_SEGMENTS
        xhat = np.array([-np.sin(th), np.cos(th), 0.0])
        zhat = np.array([-np.sin(alpha) * np.cos(th), -np.sin(alpha) * np.sin(th), np.cos(alpha)])
        yhat = np.cross(zhat, xhat)
        R = np.column_stack([xhat, yhat, zhat])
        c = np.array([r_mid * np.cos(th), r_mid * np.sin(th), CONE_HEIGHT / 2])
        out.append(
            f'      <geom type="box" size="{half_wid:.5f} {CONE_WALL_HALF_THICK} {half_len:.5f}" '
            f'pos="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}" quat="{_quat_of(R)}" class="cone_col"/>'
        )
    out.append(
        f'      <geom type="cylinder" size="{CONE_TOP_R} 0.0015" '
        f'pos="0 0 {CONE_HEIGHT - 0.0015}" class="cone_col"/>'
    )
    return "\n".join(out)


_cone_v, _cone_f = cone_mesh()
_cone_shell = cone_shell_geoms()

SCENE_XML = f"""<mujoco model="marvinpro_scene">
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-20" offwidth="1280" offheight="960"/>
  </visual>
  <default>
    <!-- Cone side-wall compliance ~9e3 N/m measured from gripper feedback in
         the teleop data (see the constants block). -->
    <default class="cone_col">
      <geom solref="{CONE_WALL_SOLREF}" friction="{CONE_WALL_FRICTION}" group="3" condim="6"/>
    </default>
  </default>
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
    <geom name="floor" size="0 0 0.05" type="plane" pos="0 0 {FLOOR_Z}" material="groundplane"/>
    <!-- Work table in front of the robot (+X forward); dimensions measured
         2026-09-09/10, see the constants block in build_scene.py. -->
    <body name="table" pos="{TABLE_EDGE_X + TABLE_DEPTH / 2} 0 {TABLE_TOP_Z}">
      <geom type="box" size="{TABLE_DEPTH / 2} {TABLE_LENGTH / 2} 0.02" pos="0 0 -0.02" rgba="{TABLE_RGBA}"/>
      <geom type="box" size="0.03 0.03 {(TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}" pos="-0.32 -{TABLE_LENGTH / 2 - 0.07} {-0.04 - (TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}"/>
      <geom type="box" size="0.03 0.03 {(TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}" pos="-0.32 {TABLE_LENGTH / 2 - 0.07} {-0.04 - (TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}"/>
      <geom type="box" size="0.03 0.03 {(TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}" pos="0.32 -{TABLE_LENGTH / 2 - 0.07} {-0.04 - (TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}"/>
      <geom type="box" size="0.03 0.03 {(TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}" pos="0.32 {TABLE_LENGTH / 2 - 0.07} {-0.04 - (TABLE_TOP_Z - 0.04 - FLOOR_Z) / 2}"/>
    </body>
    <!-- White tape target frame on the table (stacking zone); visual-only so it
         never disturbs cone contacts. -->
    <body name="target_frame" pos="0.50 0 {TABLE_TOP_Z + 0.001}">
      <geom type="box" size="{FRAME_OUTER / 2} {FRAME_STRIP / 2} 0.0005" pos="0 {FRAME_OUTER / 2 - FRAME_STRIP / 2} 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
      <geom type="box" size="{FRAME_OUTER / 2} {FRAME_STRIP / 2} 0.0005" pos="0 -{FRAME_OUTER / 2 - FRAME_STRIP / 2} 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
      <geom type="box" size="{FRAME_STRIP / 2} {FRAME_OUTER / 2 - FRAME_STRIP} 0.0005" pos="{FRAME_OUTER / 2 - FRAME_STRIP / 2} 0 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
      <geom type="box" size="{FRAME_STRIP / 2} {FRAME_OUTER / 2 - FRAME_STRIP} 0.0005" pos="-{FRAME_OUTER / 2 - FRAME_STRIP / 2} 0 0" contype="0" conaffinity="0" rgba="0.9 0.9 0.9 1"/>
    </body>
    <!-- The two cones to stack (free bodies). Hollow thin-walled shell
         (12 wall boxes + top cap) so they nest; solid mesh is visual-only. -->
    <body name="cone_1" pos="{CONE1_XY[0]} {CONE1_XY[1]} {TABLE_TOP_Z + 0.001}">
      <freejoint/>
      <inertial pos="0 0 {CONE_HEIGHT / 2}" mass="{CONE_MASS}" diaginertia="3.4e-5 3.4e-5 2.5e-5"/>
      <geom type="mesh" mesh="red_cone" contype="0" conaffinity="0" rgba="{CONE_RGBA}"/>
{_cone_shell}
    </body>
    <body name="cone_2" pos="{CONE2_XY[0]} {CONE2_XY[1]} {TABLE_TOP_Z + 0.001}">
      <freejoint/>
      <inertial pos="0 0 {CONE_HEIGHT / 2}" mass="{CONE_MASS}" diaginertia="3.4e-5 3.4e-5 2.5e-5"/>
      <geom type="mesh" mesh="red_cone" contype="0" conaffinity="0" rgba="{CONE_RGBA}"/>
{_cone_shell}
    </body>
  </worldbody>
  <!-- Grasp assist (runtime-toggled). Rigid + soft contacts reproduce the
       grasp STATE (jaw gap matches teleop feedback) but not retention through
       arm motion: the real cone's pocket-locking deformation is missing, so a
       physically-gripped cone migrates along the smooth plates and escapes.
       The rollout loop should activate the weld for the grasped (cone, arm)
       pair: write the current relative pose into eq_data[3:11] (pos of the
       carrier in the cone frame, then wxyz quat) and set data.eq_active=1;
       clear on gripper-open command. -->
  <equality>
    <weld name="weld_cone_1_L" body1="cone_1" body2="grip_carrier_L" active="false"/>
    <weld name="weld_cone_1_R" body1="cone_1" body2="grip_carrier_R" active="false"/>
    <weld name="weld_cone_2_L" body1="cone_2" body2="grip_carrier_L" active="false"/>
    <weld name="weld_cone_2_R" body1="cone_2" body2="grip_carrier_R" active="false"/>
  </equality>
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
    grip_dir = MESH_DIR / "gripper"
    grip_dir.mkdir(exist_ok=True)
    for src_name, dst_name in GRIPPER_MESHES.items():
        shutil.copyfile(OFFICIAL_URDF / "meshes" / src_name, grip_dir / dst_name)


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

    # Gripper mesh assets (files copied from the official URDF package).
    text = text.replace("  </asset>", GRIPPER_MESH_ASSETS + "\n  </asset>", 1)

    # Inject the gripper into each flange body, right after the tool body.
    for side, tool in (("L", "left_tool"), ("R", "right_tool")):
        # The tool body block ends with the </body> following its <site/>.
        pat = re.compile(r'(<body name="%s"[^>]*>.*?</body>)' % tool, re.S)
        m = pat.search(text)
        if not m:
            raise RuntimeError(f"tool body {tool} not found")
        gripper = GRIPPER_BODY.format(s=side, stroke=GRIPPER_STROKE, damp=GRIPPER_DAMPING)
        text = text[: m.end(1)] + gripper + text[m.end(1):]

    # Gripper actuators at the end of the actuator block.
    acts = "\n".join(
        GRIPPER_ACTUATOR.format(s=s, kp=GRIPPER_KP, force=GRIPPER_FORCE, stroke=GRIPPER_STROKE)
        for s in ("L", "R")
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
