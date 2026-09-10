# tianji_sim — Marvin Pro MuJoCo 仿真

天机 Marvin Pro 双臂机器人（M6 CCS-696 ×2 + DM_Gripper）的 MuJoCo 仿真资产与环境。

## 内容

- `robot_description/` — 真机控制器下载的机器人描述包（vendored，2026-09-08）：
  URDF/xacro、mesh、`mjcf/marvin_pro_sim.xml`（同事转换的 MJCF，含真实安装几何与
  调好的执行器参数）、`robot_runtime_summary.yaml`（真值安装变换/法兰系/夹爪接口）。
  `official_urdf/` 是天机官方 SolidWorks 导出模型（2026-09-10，来自完整包
  `Pro-m6-696/`，完整包不入库）：含夹爪（Link8/Arm9/10/11）、腕部相机与
  头部相机（depth + 双单目）的几何与坐标系。
- `assets/marvinpro/` — 生成的成品模型 `marvin_pro_dual_gripper.xml`、场景 `scene.xml`、mesh。
- `assets/gripper_cad/` — Seeed DM_Gripper 的 CAD（STL，不入库；来源
  github.com/YlsonDdb/DM_Gripper）。**大概率非本机夹爪型号**，仅存档参考。
- `scripts/build_scene.py` — 从 `robot_description` 重新生成模型（复制 mesh、修路径、
  注入官方夹爪、扩展 home keyframe、生成空心锥壳体）。
- `scripts/validate_mjcf.py` — 编译、home 位姿伺服保持、离屏渲染验证。
- `scripts/grasp_test.py` — 抓握冒烟测试：降臂夹取台面上的锥→闭合→抬起→释放，
  校验夹持开口与真机反馈一致（约 33 mm vs 真机 31–34 mm）。

## 用法

```bash
cd /home/jh/tianji_sim
uv sync                                    # 建环境（或 uv venv + uv pip install mujoco numpy pillow）
.venv/bin/python scripts/build_scene.py    # 可选：重新生成模型（assets/ 已含生成结果）
MUJOCO_GL=egl .venv/bin/python scripts/validate_mjcf.py
```

`build_scene.py --src-desc <path>` 可指定更新的 marvin_description 包。

## 约定

- 夹爪为 **KLM OmniGripper 官方几何**（转自天机官方 URDF `official_urdf/`，
  L 侧子树复用于两臂）：法兰到爪尖 0.176 m、最大开口 0.082 m、导轨壳体
  0.149 m——与 2026-09-10 卷尺实测（0.18 / 0.08 / 0.14 m）一致。每指行程
  0–0.04 m，**q=0 为全开**（对应电机 0 rad 上电态），策略映射 u→0.04·u。
  各连杆质量为估计值（URDF 自带 SW 惯量不可靠，如腕部相机 5.4 kg）。
  模型含腕部相机 `wrist_cam_L/R` 坐标系（头部 depth + 双单目坐标系在
  vendored URDF 里，后续相机标定用）。驱动参数（Apex 文档）：电机 ID 左 1
  右 2、归一化指令 ×1.6 rad、MIT Kp=3.0 Kd=0.12。
  （Seeed DM_Gripper 的 CAD 大概率是另一款产品，仅存档参考。）
- 红锥环境（`scene.xml`，2026-09-09 按真机实测/视频采样重建）：
  base_link → 近桌边 0.22 m（+X），桌面纵深 0.75 m，横向暂取 2 m，
  台面高 0.80 m（2026-09-10 卷尺实测，地面参考；base_link 系内 0.718 m，
  地面在 base_link z=−0.082 m 即底座 mesh 底面）。绿色台面 RGB(42,120,96)、
  红锥 RGB(174,0,43) 采样自 Stack_two_cones 视频流中值。锥底/顶
  Ø0.075/0.025 m 与白框（外沿 0.13 m / 内沿 0.09 m）已卡尺实测，
  锥高 0.08 m 仍为视频估计值。锥为**空心薄壳**（12 块 8 mm 厚壁盒 +
  顶盖，可嵌套叠放）；侧壁软接触 solref=0.010（等效 ~9e3 N/m，由遥操作
  夹爪反馈反推：闭合堵转 gap 31–34 mm vs 未形变 ~47.5 mm、每指 ~65–76 N）。
  注意：纯接触物理**无法复现**软锥形变带来的"包裹锁止"——运动中锥会沿
  光滑爪板迁移脱出；rollout 时用 `scene.xml` 里的 weld 等式约束做
  grasp assist（运行时切换 `eq_active`，详见 grasp_test.py 用法）。
- 相机位姿（对齐真机三路）与数据采集尚未开始，下一阶段接入 OpenPI WebSocket 协议。
