#!/usr/bin/env python3
"""Grasp smoke test: with cone_1 standing on the table, lower the left arm so
the plates straddle the cone at mid-upper height, close gently, then lift.

The real grip holds thanks to cone wall DEFORMATION (plates indent the soft
wall ~5-9 mm/side, measured from teleop gripper feedback). Reports the jaw
gap at grasp equilibrium — teleop reference: 31-34 mm.

Usage:
  MUJOCO_GL=egl python grasp_test.py [--render] [--tune]
"""

import argparse
import itertools
from pathlib import Path

import mujoco
import numpy as np

SIM_ROOT = Path(__file__).resolve().parents[1]
SCENE = SIM_ROOT / "assets" / "marvinpro" / "scene.xml"

GRIP_OPEN_GAP = 0.082  # finger inner-face gap at q=0
REAL_GRASP_GAP = 0.032  # teleop data: 31-34 mm
ARM_JOINTS = [f"Joint{i}_L" for i in range(1, 8)]


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qrot(q, v):
    return qmul(qmul(q, [0, *v]), qconj(q))[1:]


def joint_qpos_addr(model, name):
    return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]


def geom_world_aabb(model, data, gid):
    # geom_size of a mesh geom is its local AABB half-extent (geom frame = mesh frame)
    half = model.geom_size[gid]
    corners = np.array([[sx * half[0], sy * half[1], sz * half[2]]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    R = data.geom_xmat[gid].reshape(3, 3)
    W = (R @ corners.T).T + data.geom_xpos[gid]
    return W.min(0), W.max(0)


def run_once(model, render=False, video=False):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home"))
    for a in range(model.nu):
        data.ctrl[a] = data.qpos[model.jnt_qposadr[model.actuator_trnid[a, 0]]]
    mujoco.mj_forward(model, data)

    cone = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cone_1")
    cone_xy = data.xpos[cone][:2].copy()
    cone_z0 = data.xpos[cone][2]
    grip_z = cone_z0 + 0.05  # grip line: mid-upper cone (5/8 of 0.08 m height)

    flange = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "flange_L")
    finger_geoms = [g for b in ("grip_finger_a_L", "grip_finger_b_L")
                    for g in np.where(model.geom_bodyid == mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_BODY, b))[0]]

    writer = None
    renderer = None
    if video:
        import imageio.v2 as imageio

        renderer = mujoco.Renderer(model, height=480, width=640)
        writer = imageio.get_writer(SIM_ROOT / "renders" / "grasp_test.mp4", fps=30)

    def plate_band():
        los, his = zip(*(geom_world_aabb(model, data, int(g)) for g in finger_geoms))
        lo, hi = np.min(los, axis=0), np.max(his, axis=0)
        return lo, hi, (lo + hi) / 2

    def step(seconds, ctrl_overrides=None, ramp_from=None):
        n = int(seconds / model.opt.timestep)
        start = data.ctrl.copy()
        frame_every = max(1, round(1 / 30 / model.opt.timestep))
        for i in range(n):
            if ramp_from is not None:
                u = min(1.0, i * model.opt.timestep / 0.5)
                data.ctrl[:] = start + u * (ramp_from - start)
            if ctrl_overrides:
                for a, v in ctrl_overrides:
                    data.ctrl[a] = v
            mujoco.mj_step(model, data)
            if writer is not None and i % frame_every == 0:
                cam = mujoco.MjvCamera()
                cam.lookat[:] = [cone_xy[0], cone_xy[1], cone_z0 + 0.1]
                cam.distance, cam.azimuth, cam.elevation = 0.65, 200, -18
                renderer.update_scene(data, camera=cam)
                writer.append_data(renderer.render())

    # 1) lower the arm: pick the joint delta putting the plate band around the
    #    grip line with the cone axis centered between the plates.
    home_q = {j: data.qpos[joint_qpos_addr(model, j)] for j in ARM_JOINTS}
    best = None
    for jn, dq in itertools.product(("Joint2_L", "Joint3_L", "Joint4_L"), (-0.15, -0.25, -0.35, 0.15, 0.25, 0.35)):
        for j in ARM_JOINTS:
            data.qpos[joint_qpos_addr(model, j)] = home_q[j] + (dq if j == jn else 0)
        mujoco.mj_forward(model, data)
        lo, hi, mid = plate_band()
        score = abs(lo[2] - (cone_z0 + 0.01)) + 2 * abs(mid[0] - cone_xy[0]) + 2 * abs(mid[1] - cone_xy[1])
        if best is None or score < best[0]:
            best = (score, jn, dq)
    for j in ARM_JOINTS:
        data.qpos[joint_qpos_addr(model, j)] = home_q[j]
    data.qvel[:] = 0
    data.qacc_warmstart[:] = 0
    _, down_j, down_dq = best
    print(f"lower with {down_j} {down_dq:+.2f} rad")

    act_grip = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"gripper_{f}_L_act") for f in ("a", "b")]
    jnt_grip = [joint_qpos_addr(model, f"gripper_{f}_L") for f in ("a", "b")]

    def arm_ctrl(delta_j, delta_dq):
        for a in range(model.nu):
            j = model.actuator_trnid[a, 0]
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name in ARM_JOINTS:
                data.ctrl[a] = home_q[name] + (delta_dq if name == delta_j else 0)

    arm_ctrl(down_j, down_dq)
    target_arm = data.ctrl.copy()
    step(1.0, ramp_from=target_arm)  # ramp into the lowered pose, then settle
    lo, hi, mid = plate_band()
    print(f"plate band z=[{lo[2]:.3f},{hi[2]:.3f}] center=({mid[0]:.3f},{mid[1]:.3f}) "
          f"cone at ({cone_xy[0]:.3f},{cone_xy[1]:.3f}) grip_line_z={grip_z:.3f}")

    # 2) close the gripper with a ramp, then hold.
    close_ctrl = target_arm.copy()
    for a in act_grip:
        close_ctrl[a] = 0.04
    step(1.0, ramp_from=close_ctrl)
    step(0.5)
    q_a, q_b = (data.qpos[j] for j in jnt_grip)
    gap = GRIP_OPEN_GAP - q_a - q_b
    cone_ok = abs(data.xpos[cone][2] - cone_z0) < 0.02 and abs(data.xpos[cone][0] - cone_xy[0]) < 0.03
    print(f"after close: finger q=({q_a:.4f}, {q_b:.4f}) -> gap {gap * 1000:.1f} mm "
          f"(real: {REAL_GRASP_GAP * 1000:.0f} mm), cone still at table: {cone_ok}")

    # Grasp assist: weld the cone to the carrier at the current relative pose
    # (see scene.xml equality comment for why this exists).
    wid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "weld_cone_1_L")
    carrier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "grip_carrier_L")
    pc, qc = data.xpos[cone].copy(), data.xquat[cone].copy()
    pg, qg = data.xpos[carrier].copy(), data.xquat[carrier].copy()
    model.eq_data[wid, 3:6] = qrot(qconj(qc), pg - pc)
    rel_q = qmul(qconj(qc), qg)
    model.eq_data[wid, 7:11] = rel_q / np.linalg.norm(rel_q)
    data.eq_active[wid] = 1
    print("weld assist ON (cone_1 <-> gripper_L)")

    # 3) lift: near-pure vertical flange translation via damped least-norm IK
    #    on the arm Jacobian (the fair retention test — no sideways wipe).
    cur_q = {j: data.qpos[joint_qpos_addr(model, j)] for j in ARM_JOINTS}
    flange_z0 = data.xpos[flange][2]
    arm_dofs = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM_JOINTS]
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, flange)
    J = jacp[:, arm_dofs]
    dq = np.linalg.lstsq(J.T @ J + 0.01 * np.eye(7), J.T @ np.array([0.0, 0.0, 0.1]), rcond=None)[0]
    print(f"lift IK (pure +z 0.1 m): dq={np.round(dq, 3)}")

    lift_ctrl = close_ctrl.copy()
    for a in range(model.nu):
        j = model.actuator_trnid[a, 0]
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name in ARM_JOINTS:
            lift_ctrl[a] = cur_q[name] + dq[ARM_JOINTS.index(name)]
    step(1.0, ramp_from=lift_ctrl)
    step(0.5)
    cone_rise = data.xpos[cone][2] - cone_z0
    flange_rise = data.xpos[flange][2] - flange_z0
    retained = flange_rise > 0.03 and cone_rise > 0.6 * flange_rise
    print(f"after lift: flange rose {flange_rise:+.3f} m, cone rose {cone_rise:+.3f} m -> "
          f"{'RETAINED' if retained else 'LOST'}")

    # 4) release: open the gripper and drop the weld; the cone should fall.
    data.eq_active[wid] = 0
    for a in act_grip:
        data.ctrl[a] = 0.0
    z_before = data.xpos[cone][2]
    step(0.8)
    released = data.xpos[cone][2] < z_before - 0.02
    print(f"after release: cone z {z_before:.3f} -> {data.xpos[cone][2]:.3f} -> "
          f"{'RELEASED' if released else 'STUCK'}")

    if render:
        renderer2 = mujoco.Renderer(model, height=720, width=960)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [cone_xy[0], cone_xy[1], cone_z0 + 0.1]
        cam.distance, cam.azimuth, cam.elevation = 0.6, 200, -15
        renderer2.update_scene(data, camera=cam)
        import PIL.Image

        out = SIM_ROOT / "renders" / "grasp_test.png"
        PIL.Image.fromarray(renderer2.render()).save(out)
        renderer2.close()
        print(f"rendered {out}")

    if writer is not None:
        writer.close()
        renderer.close()
        print(f"wrote {SIM_ROOT / 'renders' / 'grasp_test.mp4'}")

    return gap, cone_ok, retained, released


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--video", action="store_true", help="record renders/grasp_test.mp4")
    parser.add_argument("--tune", action="store_true", help="sweep cone solref timeconst values")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    cone_geoms = np.where(np.isin(
        [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) for g in range(model.ngeom)],
        ["cone_1", "cone_2"]))[0]

    if args.tune:
        for tc, width, power in itertools.product((0.010, 0.014, 0.02), (0.001, 0.005), (2.0, 3.0)):
            model.geom_solref[cone_geoms] = (tc, 1.0)
            model.geom_solimp[cone_geoms] = (0.9, 0.95, width, 0.5, power)
            gap, held, retained, _released = run_once(model)
            print(f"== tc={tc:5.3f} width={width} power={power}: gap={gap * 1000:5.1f} mm "
                  f"held={held} retained={retained}")
        return

    gap, held, retained, released = run_once(model, args.render, args.video)
    ok = held and retained and released and abs(gap - REAL_GRASP_GAP) < 0.008
    print("PASS" if ok else "FAIL")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
