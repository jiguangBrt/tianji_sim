#!/usr/bin/env python3
"""Validate the Marvin Pro scene MJCF: compile, hold the home pose, render views.

Usage:
  MUJOCO_GL=egl python validate_mjcf.py [--seconds 3] [--out renders/]
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np

SIM_ROOT = Path(__file__).resolve().parents[1]
SCENE = SIM_ROOT / "assets" / "marvinpro" / "scene.xml"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--out", type=Path, default=SIM_ROOT / "renders")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    print(f"model: nq={model.nq} nv={model.nv} nbody={model.nbody} ngeom={model.ngeom} "
          f"nu={model.nu} nmesh={model.nmesh}")

    # Start from the colleague's home keyframe and servo-hold it.
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    # ctrl order (actuator document order) != qpos order (joint document order):
    # map each actuator's target joint value explicitly.
    for a in range(model.nu):
        j = model.actuator_trnid[a, 0]
        data.ctrl[a] = data.qpos[model.jnt_qposadr[j]]
    qpos0 = data.qpos.copy()

    steps = int(args.seconds / model.opt.timestep)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    drift = np.max(np.abs(data.qpos - qpos0))
    print(f"after {args.seconds:.1f}s home-pose hold: max qpos drift = {drift:.2e}, ncon={data.ncon}")
    if not np.all(np.isfinite(data.qpos)):
        raise SystemExit("FAIL: qpos contains NaN/inf")
    if drift > 0.05:
        print("WARN: joints drifted more than 0.05 rad/m from home")

    for name in ("flange_L", "flange_R", "left_tool", "right_tool", "cone_1", "cone_2"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        print(f"{name} world pos: {np.round(data.xpos[bid], 3)}")

    renderer = mujoco.Renderer(model, height=720, width=960)
    views = {
        "front": {"lookat": [0.2, 0.0, 0.9], "distance": 2.2, "azimuth": 180, "elevation": -20},
        "iso": {"lookat": [0.2, 0.0, 0.9], "distance": 2.4, "azimuth": 135, "elevation": -25},
        "flange_closeup": {"lookat": list(data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "flange_L")]),
                           "distance": 0.5, "azimuth": 200, "elevation": -15},
    }
    import PIL.Image

    for name, cam in views.items():
        c = mujoco.MjvCamera()
        c.lookat[:] = cam["lookat"]
        c.distance = cam["distance"]
        c.azimuth = cam["azimuth"]
        c.elevation = cam["elevation"]
        renderer.update_scene(data, camera=c)
        path = args.out / f"{name}.png"
        PIL.Image.fromarray(renderer.render()).save(path)
        print(f"rendered {path}")
    renderer.close()


if __name__ == "__main__":
    main()
