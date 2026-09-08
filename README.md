# tianji_sim — Marvin Pro MuJoCo 仿真

天机 Marvin Pro 双臂机器人（M6 CCS-696 ×2 + DM_Gripper）的 MuJoCo 仿真资产与环境。

## 内容

- `robot_description/` — 真机控制器下载的机器人描述包（vendored，2026-09-08）：
  URDF/xacro、mesh、`mjcf/marvin_pro_sim.xml`（同事转换的 MJCF，含真实安装几何与
  调好的执行器参数）、`robot_runtime_summary.yaml`（真值安装变换/法兰系/夹爪接口）。
- `assets/marvinpro/` — 生成的成品模型 `marvin_pro_dual_gripper.xml`、场景 `scene.xml`、mesh。
- `assets/gripper_cad/` — DM_Gripper 真实 CAD（STL，不入库；来源 github.com/YlsonDdb/DM_Gripper）。
- `scripts/build_scene.py` — 从 `robot_description` 重新生成模型（复制 mesh、修路径、
  注入简化水平夹爪、扩展 home keyframe）。
- `scripts/validate_mjcf.py` — 编译、home 位姿伺服保持、离屏渲染验证。

## 用法

```bash
cd /home/jh/tianji_sim
uv sync                                    # 建环境（或 uv venv + uv pip install mujoco numpy pillow）
.venv/bin/python scripts/build_scene.py    # 可选：重新生成模型（assets/ 已含生成结果）
MUJOCO_GL=egl .venv/bin/python scripts/validate_mjcf.py
```

`build_scene.py --src-desc <path>` 可指定更新的 marvin_description 包。

## 约定

- 夹爪为简化双指模型：水平开合（沿法兰 Y），法兰到指尖 0.10 m（真机实测），
  每指行程 0–0.04 m。真实夹爪是 DM_Gripper（DM4310-2EC），高保真导入待做。
- 仿真环境（红锥任务、相机、数据采集）尚未开始，下一阶段接入 OpenPI WebSocket 协议。
