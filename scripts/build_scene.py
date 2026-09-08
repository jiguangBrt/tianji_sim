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
       (real gripper identified as DM_Gripper / DM4310-2EC; CAD in
       assets/gripper_cad/; horizontal jaws along flange Y, flange->tip 0.10 m
       as measured on the real robot)
     - extends the "home" keyframe with the 4 gripper slide joints (open)
  3. writes scene.xml (floor/table/lights, includes the robot model)

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
# Simplified parallel gripper (per arm), mounted in the flange frame.
# Fingers extend along flange +Z (tool axis), open along flange X.
# stroke 0..0.04 m per finger; 0 = closed, 0.04 = fully open.
# ---------------------------------------------------------------------------
GRIPPER_STROKE = 0.04
GRIPPER_FORCE = 20.0
GRIPPER_OPEN_HOME = 0.03

GRIPPER_BODY = """
        <!-- Real CCS-696 opens HORIZONTALLY: at the home pose flange Y maps to
             world -Y (lateral), so the jaws slide along flange Y. Fingers
             extend along flange +Z (tool axis); flange->fingertip = 0.10 m
             (measured on the real robot 2026-09-08). Real gripper identified
             as the DM_Gripper (DM4310-2EC, Seeed SKU 100094243); CAD staged
             in sim/assets/gripper_cad/ for a future high-fidelity import. -->
        <body name="gripper_base_{s}" pos="0 0 0.01">
          <inertial pos="0 0 0.05" mass="0.3" diaginertia="0.0002 0.0002 0.0002"/>
          <geom type="box" size="0.022 0.03 0.015" pos="0 0 0.015" class="grip_col" rgba="0.25 0.25 0.28 1"/>
          <body name="finger_a_{s}" pos="0 0.02 0.02">
            <inertial pos="0 0 0.04" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
            <joint name="gripper_a_{s}" type="slide" axis="0 -1 0" range="0 {stroke}" damping="1" armature="0.001"/>
            <geom type="box" size="0.02 0.006 0.04" pos="0 0 0.04" class="grip_col" rgba="0.35 0.35 0.38 1"/>
          </body>
          <body name="finger_b_{s}" pos="0 -0.02 0.02">
            <inertial pos="0 0 0.04" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
            <joint name="gripper_b_{s}" type="slide" axis="0 1 0" range="0 {stroke}" damping="1" armature="0.001"/>
            <geom type="box" size="0.02 0.006 0.04" pos="0 0 0.04" class="grip_col" rgba="0.35 0.35 0.38 1"/>
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

SCENE_XML = """<mujoco model="marvinpro_scene">
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
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <!-- Placeholder work table in front of the robot (+X forward). -->
    <body name="table" pos="0.45 0 0.38">
      <geom type="box" size="0.40 0.60 0.02" pos="0 0 0.36" rgba="0.55 0.40 0.28 1"/>
      <geom type="box" size="0.03 0.03 0.36" pos="-0.35 -0.55 0"/>
      <geom type="box" size="0.03 0.03 0.36" pos="-0.35 0.55 0"/>
      <geom type="box" size="0.03 0.03 0.36" pos="0.35 -0.55 0"/>
      <geom type="box" size="0.03 0.03 0.36" pos="0.35 0.55 0"/>
    </body>
  </worldbody>
  <include file="marvin_pro_dual_gripper.xml"/>
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

    # Extend home keyframe: gripper joints appear right after each arm's
    # Joint7 in document order -> insert open values at positions 7-8 and end.
    old_key = 'qpos="1.7 -1.1 -1.1 -2 -0.37 0.13 0.55 -1.7 -1.1 1.1 -2 0.37 0.13 -0.55"'
    o = f"{GRIPPER_OPEN_HOME} {GRIPPER_OPEN_HOME}"
    new_key = f'qpos="1.7 -1.1 -1.1 -2 -0.37 0.13 0.55 {o} -1.7 -1.1 1.1 -2 0.37 0.13 -0.55 {o}"'
    if old_key not in text:
        raise RuntimeError("home keyframe not found")
    text = text.replace(old_key, new_key, 1)
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
