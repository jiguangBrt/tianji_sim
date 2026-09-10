# tianji_sim — Marvin Pro MuJoCo 仿真

天机 Marvin Pro 双臂机器人（M6 CCS-696 ×2 + DM_Gripper）的 MuJoCo 仿真资产与环境。

## 内容

- `robot_description/` — 真机控制器下载的机器人描述包（vendored，2026-09-08）：
  URDF/xacro、mesh、`mjcf/marvin_pro_sim.xml`（同事转换的 MJCF，含真实安装几何与
  调好的执行器参数）、`robot_runtime_summary.yaml`（真值安装变换/法兰系/夹爪接口）。
- `assets/marvinpro/` — 生成的成品模型 `marvin_pro_dual_gripper.xml`、场景 `scene.xml`、mesh。
- `assets/gripper_cad/` — Seeed DM_Gripper 的 CAD（STL，不入库；来源
  github.com/YlsonDdb/DM_Gripper）。**大概率非本机夹爪型号**，仅存档参考。
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

- 夹爪为简化 KLM OmniGripper 模型：水平开合（沿法兰 Y），最大开口 0.08 m
  （每指行程 0–0.04 m），壳体 0.14 m（沿开合轴），转轴距法兰 ~0.08 m，
  法兰到爪尖 0.18 m（2026-09-10 卷尺实测 + eef/视频交叉验证）。驱动：
  电机 ID 左 1 右 2、归一化指令 ×1.6 rad、MIT Kp=3.0 Kd=0.12（Apex 文档）。
  指令映射 u→0.04·(1−u) 为线性近似，行程标定待做；爪板轮廓为视频估计。
  （Seeed DM_Gripper 的 CAD 大概率是另一款产品，仅存档参考。）
- 红锥环境（`scene.xml`，2026-09-09 按真机实测/视频采样重建）：
  base_link → 近桌边 0.22 m（+X），桌面纵深 0.75 m，横向暂取 2 m，
  台面高 0.73 m（eef+视频估计，±1 cm，待卷尺确认）。绿色台面 RGB(42,120,96)、
  红锥 RGB(174,0,43) 采样自 Stack_two_cones 视频流中值。锥底/顶
  Ø0.075/0.025 m 与白框（外沿 0.13 m / 内沿 0.09 m）已卡尺实测，
  锥高 0.08 m 仍为视频估计值。
- 相机位姿（对齐真机三路）与数据采集尚未开始，下一阶段接入 OpenPI WebSocket 协议。
