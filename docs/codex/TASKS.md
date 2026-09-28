# AILamp 课后迭代任务（`post-course` 分支）

**前提**：`post-course` 分支已经导入课程最终版 546cbeb（任务 P2-0，由维护者在本地完成）。

**使用方法**：一个任务开一个 PR，按编号顺序做。每个任务都必须满足下面三条：
- 先写会失败的测试，再实现；
- `uv run pytest -q` 全部通过；
- 在 PR 描述里写清验证了什么、用的什么命令。

**路径约定**：以下路径都相对 `ailamp_runtime/ailamp/`，另有注明的除外。文中的行号以 546cbeb 为准，改动后可能漂移。

---

## P2-1 修复「偏移为 0 被当成无偏移」

**问题**
- `models.py` 约第 50 行：`VisionEvent.normalized_offset` 的默认值是 `0.0`。
- `services/decision.py` 约第 79、83 行：用 `offset if offset else ±1.0` 做真值判断。
- 结果：模型返回「LEFT 且 offset=0.0」时，灯会按满步长转一步；「没有偏移信息」和「偏移正好为 0」两种情况无法区分。

**改法**
- 把 `normalized_offset` 改为 `Optional[float] = None`。
- 所有真值判断改成 `is None` 判断。
- 检查所有构造 `VisionEvent` 的地方：YOLO 路径、API 视觉路径、手势路径。

**验收**
- 新测试一：offset=0.0 时落在死区内，不产生动作。
- 新测试二：offset=None 时，按事件方向走默认步长。
- 原有测试全部通过。

## P2-2 统一单位语义（`delta_deg` 实际是归一化单位）

**问题**
- `services/motor.py` 第 41–42 行的注释写明：`delta_deg` 是沿用下来的旧名字，值其实是 LeLamp 归一化单位（−100…100）。
- 决策层按固定的「4 个单位」出步长。每重新校准一次，物理角度就跟着变：base_yaw 的 4 个单位按旧校准约 2.9°，按新校准约 1.6°。

**改法**
1. 新建 `units.py`，提供两个函数：`deg_per_unit(joint, calib)` 和 `deg_to_units(joint, deg, calib)`。
   - 换算公式：`deg_per_unit = (range_max − range_min) / 200 × 360 / 4096`。
   - `range_min`、`range_max` 取自 LeLamp 校准 JSON 中对应关节的原始 counts，舵机一圈是 4096 counts。
2. `JointDeltaCommand` 新增字段 `delta_units`。
   - 保留 `delta_deg` 作为只读别名，读取时发出 `DeprecationWarning`。
   - 更新 `tests/test_motor_service.py` 第 291–299 行对旧字段名的钉住测试。
3. 决策层的步长参数改为以度为单位（`step_deg`），下发前经 `deg_to_units` 换算成归一化单位。

**验收**
- 用两份 range 不同的校准数据，给同样的 `step_deg`，换算出的物理角度相同，误差在 1e-9 以内。
- 以 `output/hardware_backups/LeLamp-demo-calibration-20260908-7QE2U0/lelamp.json` 中 base_yaw 的数值为准，1 个单位约等于 0.40°。

## P2-3 补控制器测试，并把覆盖率接入 CI

**改法**
1. 为 `services/controller.py` 约第 926–927 行「增量超过 ±4 就拒绝」补一个测试：
   - 用 `apply_brain_plan` 提交一个增量为 5 的计划；
   - 断言它被拒绝，并且 FakeMotor 没有收到任何写入。
2. 用 coverage 报告找出 controller、decision、brain 里还没被覆盖的分支，逐个补上测试。
3. 修改 `.github/workflows/ci.yml`：
   - 测试步骤改为 `python -m pytest -q --cov=ailamp --cov-branch --cov-report=xml --cov-fail-under=80`；
   - 用 `actions/upload-artifact` 上传 coverage.xml；
   - 把 `pytest-cov` 加进 `[test]` extra，然后运行 `uv lock`。

**验收**
- 本地和 CI 上分支覆盖率都 ≥ 80%。
- 那一行 raise 已经被测试覆盖到。

## P2-4 语音工具统一经 LampController 仲裁

**问题**
- `agent/livekit_agent.py` 里，`AILampToolbox`（约第 51–65 行）自己建了 MotorService 和 LED 服务，绕过了 `LampController`。所以 `controller.stop()` 和 `arm(False)` 都管不到语音触发的动作。
- 真正建会话、注册 `function_tool` 的入口（约第 166 行）标着 `pragma: no cover`，没有任何测试。

**改法**
1. 新增 `build_voice_tools(controller, recording_names) -> list`，返回一组纯 Python 可调用对象，每个对应一个语音工具：
   - `play_recording`、`move_joints`、`set_light`、`set_mode` 这四个工具，都按同一个流程处理：
     1. 用 `services/brain.py` 的 `validate_actions`（及 `_validate_arguments`）校验参数，**不另写一套校验**；
     2. 读取控制器当前的代次号；
     3. 组装成 `BrainPlan`；
     4. 调用 `controller.apply_brain_plan(...)`，传入 `expected_generation` 和 `deadline`。
   - `stop`：直接调用 `controller.stop("voice")`。
2. LiveKit 的装配层只负责把这些可调用对象包装成 `function_tool`。这一层要尽量薄，允许继续标 `pragma: no cover`。
3. 删掉 `AILampToolbox` 里自建的 MotorService 和 LED 服务，改成注入同一个 `LampController`。

**验收**（全部使用假对象）
- 语音触发的动作执行到一半时调用 `stop`：代次号加一，后续帧被丢弃，FakeMotor 不再收到写入。
- `move_joints` 传入 +9：请求在到达控制器之前就被拒绝，控制器没有收到调用。
- 整个进程里只有一个 MotorService 实例，串口只有一个持有者。

## P2-5 MuJoCo 仿真后端

**改法**
1. 新增 `services/sim_backend.py`，写一个 `SimMotorBackend`，接口与 `services/motor_backend.py` 里的 `LeLampMotorBackend` 相同：`connect`、`disconnect`、`send_action`、读位置。
   - 加载 `simulation/ailamp_scene.xml`。
   - 把归一化目标换算成关节角后写入执行器的 `data.ctrl`，再调用 `mujoco.mj_step`。
   - 位置从 `data.qpos` 读取，再换算回归一化单位。
2. CLI 增加 `--backend {hardware,dry-run,sim}` 参数，`ailamp agent` 和 `ailamp brain` 两个命令都支持。
3. 新建 `docs/en/sim-backend-runbook.md`，写明维护者如何在本地用自己的 OpenAI/LiveKit key 驱动仿真灯，以及怎么录屏。
   - key 只从环境变量读取。
   - 文档里只写环境变量名，不写任何 key 的值。

**验收**（标 `sim`）
- 向 `SimMotorBackend` 发一个关节目标，推进 N 步后，读回的位置收敛到目标附近（容差写进测试）。
- 经 `LampController` 调用 `stop` 后，仿真关节保持在当前位置。

## P2-6 由单一数据源生成 URDF/MJCF（惯量与限位）

**问题**
- URDF 和 MJCF 都拷自上游，质量和惯量也沿用上游数据。
- 新底座只替换了可视网格，没有对应的惯量和碰撞体。
- 关节限位没有和校准数据对齐。

**改法**
1. 新建 `scripts/build_robot_models.py`，读取三样输入：上游模型、`scripts/generate_ailamp_adapters.py` 生成的底座网格、校准 JSON。
   - **底座惯量**：用 `trimesh` 的 `mesh.mass_properties` 计算体积、质心和惯量张量。
     - 等效密度 = PLA 1.24 g/cm³ × 等效填充率。
     - 等效填充率按「外壁 + 填充」的简单模型估算；估算方法和取值要写进代码注释和生成文件头部。
   - **碰撞体**：用凸包或少量盒体近似，不直接使用可视网格。
   - **关节限位**：原始 counts → 归一化值 → 弧度，同时写入 MJCF 的 `range` 和 URDF 的 `<limit>`。
   - 输出 `simulation/ailamp_robot.xml` 和 `simulation/ailamp_robot.urdf`，两个文件头部都注明「生成文件，勿手改」。
2. 新增测试：
   - URDF 和 MJCF 的关节名、转轴、限位完全一致；
   - 用 MuJoCo 分别加载两份模型，在 100 组固定种子的随机关节角下，比较各 body 的世界坐标，误差 < 1e-6 m。

**验收**
- 上述测试通过。
- 生成过程可复现：连续生成两次，输出逐字节一致。

## P2-7 带断言的 MuJoCo 测试进 CI

**改法**
1. 新建 `tests/sim/`，并在 `pyproject.toml` 注册 `sim` 标记。没装 mujoco 时用 `pytest.importorskip("mujoco")` 跳过。测试内容：
   - 模型能正常加载；
   - 执行器数量和名称正确；
   - 每段录制都经 `mj_step` 完整回放，全程无 NaN、不越关节限位，跟踪误差上界写进测试；
   - 把 `cli.py` 里 `sim_check` 的各项检查改写为断言。
2. 在 `.github/workflows/ci.yml` 里新增一个 job：
   - `apt-get install -y libosmesa6`；
   - 安装 `.[simulation,test]`；
   - 设 `MUJOCO_GL=osmesa`；
   - 运行 `python -m pytest -m sim -q`。

**验收**
- 两个 job 在 GitHub Actions 上都通过。
- `pytest -m sim` 在本地同样通过。

## P2-8 仿真闭环视觉跟随

**现状**
- 当前实现是事件阈值（±0.20）+ 固定步长 + 1.5 s 冷却，各关节独立控制，没有图像雅可比。
- 按新校准，最快跟随速度约 1.1°/s。

**改法**
1. 在 MJCF 的灯头上加一个相机（camera 元素）。目标沿用 `sim-check` 已有的虚拟目标滑块关节。
2. 感知有两种实现，都要做成可替换的组件：
   - **真值投影**：用相机内参把目标的世界坐标投影到图像平面，得到归一化水平偏移。这是默认实现，专门用来测控制环。
   - **YOLOv8n 检测（可选）**：对渲染帧运行检测，前提是环境里已经预下载了权重。如果渲染出的目标检测不出来，就在结果里如实记录。
3. 新增连续跟随控制器：PD + 限速 + 死区，输出偏航角速度，再按 P2-2 的方法换算成关节增量。
   - 输出仍然经过 `LampController`，受 per-joint 限幅约束。
   - 旧的事件式控制器保留，作为对照。
4. 新建 `scripts/eval_visual_following.py`，对新旧两个控制器分别测：
   - 阶跃目标：调节时间、稳态误差（度）；
   - 匀速目标：最大可跟随角速度（度/秒）。

   结果写入 `docs/sim_results/visual_following.json`，并生成一张曲线图。

**验收**
- 评估脚本在固定种子下可以复现。
- 在 README 的 P2-11 表里如实填写，并注明「仿真」。

## P2-9 Pico LED 通道加固（CircuitPython）

**现状**
- `firmware/pico_led_controller/code.py` 用 `sys.stdin`/`print` 收发，走的是 REPL console 口。
- 串口路径写死在 `config/hardware.toml`：舵机 `/dev/ttyACM0`，Pico `/dev/ttyACM1`。
- 真机上 LED 通道当时不可用，根因没有记录。

**改法**
1. 固件：
   - 新增 `boot.py`，执行 `usb_cdc.enable(console=True, data=True)`。
   - `code.py` 改为在 `usb_cdc.data` 上收发。
   - 协议加序号和 CRC-8：请求格式 `<seq> <CMD> <args>*<crc8hex>\n`，回复格式 `<seq> OK|ERR <reason>`。
   - 解析和编码逻辑抽成 `firmware/pico_led_controller/ledproto.py`，只用 CircuitPython 与 CPython 都有的语法和模块，这样能在 CPython 下单测。
2. 主机端 `services/led_serial.py`：
   - 用 `serial.tools.list_ports` 按 VID/PID 和接口描述找到 data 口。具体 ID 以 CircuitPython 对应板型的 USB 配置为准，并在代码注释里写明出处。
   - 找不到时回退到 `/dev/serial/by-id/...`；配置里显式指定的路径优先。
   - 加上超时、重试，以及对序号不匹配或 CRC 错误的处理。
3. 新增 `deploy/udev/99-ailamp.rules`：生成 `/dev/ailamp-led`，以及 `/dev/ailamp-servo`（舵机板 VID 为 1a86）。
4. 同步更新部署文档（`docs/en`、`docs/zh`）。

**验收**
- `ledproto` 在 CPython 下单测通过：往返编解码、CRC 错误、序号错位。
- 主机端用 `FakeSerial` 测试：乱码、超时重试、端口自动发现。
- 以上都没有在硬件上验证，PR 和 README 里要写明这一点。

## P2-10 底座生成器修正

**改法**（`scripts/generate_ailamp_adapters.py`）
1. 舵机腔：第 48–49 行，腔体 24.7 mm 比舵机本体 24.72 mm 还小 0.02 mm。
   - 引入参数 `SERVO_FIT_CLEARANCE_MM`，默认每边 0.15 mm，腔体尺寸改为「舵机尺寸 + 2 × 间隙」。
   - 测试断言：腔体 ≥ 舵机 + 2 × 间隙。
2. Jetson 螺柱孔距：第 148–149 行的 84×64 mm 目前没有出处。
   - 对照 NVIDIA Jetson Nano Developer Kit 载板的官方机械图核对，在代码注释里写上出处。
   - 查不到出处的，改为可配置参数，并标注 `TODO: verify against NVIDIA drawing`。
3. 重新生成检入的 3MF/STL，并更新逐字节一致性测试的基准。

**验收**
- `tests/test_ailamp_adapters.py` 全部通过。
- 间隙的新测试通过。

## P2-11 README「Verification status」表

在 README 里新增一节，用三栏把每项功能的验证状态写清楚：
- **课程期间在真机上做到的**：按 `docs/zh/Nano连接记录.md` 的记录填写，包括人工托举下的单轴往返、五轴保持、一次 wake_up 回放；完整动作剧本没有跑通。
- **课后只在仿真或单元测试中验证的**：P2-1 至 P2-10 的内容。
- **仍未验证的**：视觉 API、YOLO、LiveKit 语音、LED 在真机上的运行；OpenAI 的真实调用。

**验收**：表里每一行都能对应到一条日志、一个测试或一个结果文件。
