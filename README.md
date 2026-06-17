# RMTT NMPC 自主飞行工作区

这个工作区用于用 RoboMaster SDK 直接控制 RMTT，并把 NMPC 控制核心集成到本地 Python 链路中。飞行控制使用 RoboMaster SDK 的 `flight.rc(a,b,c,d)`，杆量范围是 `[-100, 100]`。

起飞和降落走 RoboMaster SDK 的 `drone.flight.takeoff()` / `drone.flight.land()`，不是手写杆量起降。辨识和 XYZ 飞行的定位都来自 VRPN，不使用无人机 OSD 位置或 ROS topic。

下面的命令默认在仓库根目录运行。

## 目录结构

```text
rmtt/           无人机连接、配网、查 IP、电量、起降等基础能力
rmtt_control/   预检、VRPN、辨识、模型拟合、NMPC、XYZ 任务
scripts/        可以直接运行的小工具入口
nmpc/           本地 NMPC 控制核心
models/         RMTT 速度模型，默认是 bootstrap 模型
native/         原生 VRPN helper
tests/          离线测试和审计
```

## 本地配置

不要把具体 IP、Wi-Fi 名称、密码、本机路径写死到代码或提交文档里。仓库提供配置模板，真实配置放到本机忽略文件：

```bash
cp config/rmtt.example.json config/rmtt.local.json
```

编辑 `config/rmtt.local.json`。`drone.ip` 填扫描到的 RMTT 地址，`drone.ap_local_ip` 填电脑连接 RMTT 热点时的本机地址，`wifi` 填目标路由信息，`vrpn` 填定位服务信息，其中 `vrpn.port` 是数字或 `null`。

`config/rmtt.local.json` 已加入 `.gitignore`。代码默认读取它；如果需要临时覆盖，可以设置同名环境变量，例如 `RMTT_IP`、`RMTT_VRPN_HOST`、`RMTT_CONFIG`。

## Clone 后初始化

从仓库 clone 下来后，到执行配网前，可以先运行：

```bash
python3 scripts/setup_workspace.py
```

这个脚本只做本地准备，不连接无人机、不配网、不发杆量。它会安装 Python 依赖、创建 `config/rmtt.local.json`、检查核心 import，并运行离线校验。

如果本机已经装好 VRPN 开发库，还可以顺手构建 VRPN helper：

```bash
python3 scripts/setup_workspace.py --build-vrpn-helper
```

如果不想安装依赖，只想补齐配置文件：

```bash
python3 scripts/setup_workspace.py --skip-pip
```

如果只想快速创建配置，不跑离线校验：

```bash
python3 scripts/setup_workspace.py --skip-pip --skip-validate
```

## 总流程

推荐流程分成三个阶段，其中“自主辨识”和“飞 XYZ”是两次独立飞行：

```text
1. 初始化
   配网 -> 切到路由网络 -> 查 RMTT IP

2. 自主辨识
   检查 VRPN/无人机 -> 连接无人机 -> 起飞 -> 自主辨识 -> 降落
   -> 拟合模型 -> 检查模型可用性

3. 飞 XYZ 任务
   检查 VRPN/无人机/模型 -> 连接无人机 -> 起飞 -> 飞 waypoint/XYZ -> 降落
```

不要用 `rmtt_control.rmtt_nmpc_workflow --stages preflight,identify,xyzway --takeoff --land` 表达这个目标；那个 workflow 更偏向“一次起飞，辨识后空中交接到 xyzway”。当前推荐把第 2 阶段和第 3 阶段分开跑。

## 0. 工作区检查

```bash
python3 -m rmtt_control.validate_workspace
```

这个命令不连接无人机，不发杆量。

## 1. 初始化：配网和查 IP

配网前，电脑需要先连到 RMTT 自己的 Wi-Fi 热点。

```bash
python3 scripts/wifi_test.py
```

配网命令发出后，RMTT 会切到路由模式。然后电脑也切到同一个路由网络，再查飞机 IP：

```bash
python3 scripts/scan_ip.py
```

把扫描得到的飞机 IP 写入 `config/rmtt.local.json` 的 `drone.ip`。

## 2. 飞行前检查

构建 VRPN helper：

```bash
./build_vrpn_helper.sh
```

检查 VRPN 是否有 pose：

```bash
python3 -m rmtt_control.preflight_check \
  --check-vrpn-helper \
  --check-vrpn
```

检查无人机连接和电量：

```bash
python3 -m rmtt_control.preflight_check \
  --check-drone \
  --min-battery 30
```

单独读电量：

```bash
python3 scripts/battery_monitor.py \
  --timeout 8
```

## 3. 阶段二：自主辨识

这个阶段是一次独立飞行：连接、起飞、辨识、降落、拟合模型、检查模型质量。

```bash
python3 -m rmtt_control.identify_pipeline \
  --axes pitch,roll,throttle,yaw \
  --pass-count 2 \
  --signals step \
  --amplitudes 10,20 \
  --field-limit 1.5 \
  --z-min 0.25 \
  --z-max 2.0 \
  --method auto \
  --recenter \
  --send \
  --confirm-risk \
  --takeoff \
  --land \
  --fit \
  --backup \
  --quality-gate \
  --quality-fail-on-bootstrap \
  --quality-require-validation
```

说明：

- `--send --confirm-risk` 才会真实发杆量。
- `--takeoff --land` 表示这个阶段自己起飞、自己降落。
- `--recenter` 会在 pitch/roll 激励后用 NMPC 回到初始 VRPN pose 附近。
- `--pass-count 2` 会采两轮独立数据：第一轮写入 `train/`，第二轮写入 `validate/`。模型只用 train 拟合，用 validate 计算独立验证指标。
- 每个轴采集结束后会检查 CSV 质量；全部采集完成后会先降落，再拟合模型并执行模型质量检查。
- 辨识 CSV 会写到 `identify_run_时间戳/train/` 和 `identify_run_时间戳/validate/`，或你指定的 `--output-dir` 下。
- `--fit` 会更新 `models/rmtt_velocity_model.json`。
- `--quality-gate --quality-fail-on-bootstrap --quality-require-validation` 会在模型不可用或缺少独立验证时让命令失败。
- 当前 `step` 协议每个轴每轮约 12 秒；两轮四轴激励约 96 秒。加上轴间稳定、pitch/roll 的 recenter、起飞降落和连接开销，通常按 3 到 5 分钟预估。若 VRPN 丢帧、recenter 多次接近超时、或安全边界触发，时间会变长或直接失败。

干跑时去掉：

```text
--send --confirm-risk --takeoff --land
```

## 4. 检查模型

辨识完成后，单独检查模型质量：

```bash
python3 -m rmtt_control.model_quality \
  --model models/rmtt_velocity_model.json \
  --fail-on-bootstrap \
  --require-validation \
  --min-samples 30 \
  --min-r2 0.2 \
  --max-nrmse 0.8
```

如果这里失败，不建议进入 XYZ 飞行阶段。先看辨识 CSV 质量、VRPN 是否稳定、激励幅度是否足够、飞行空间是否太小。

## 5. 阶段三：飞 XYZ 任务

这个阶段是另一架次独立飞行：检查模型、连接、起飞、飞 waypoint、降落。

```bash
python3 -m rmtt_control.xyzway_nmpc \
  --model models/rmtt_velocity_model.json \
  --waypoints waypoints/line-guide-diagonal-3d.json \
  --source vrpn \
  --controller mission \
  --method auto \
  --field-limit 1.5 \
  --z-min 0.25 \
  --z-max 2.0 \
  --log-csv xyzway_run.csv \
  --require-real-model \
  --quality-require-validation \
  --send \
  --confirm-risk \
  --takeoff \
  --land
```

说明：

- `--source vrpn` 表示 XYZ 飞行定位来自 VRPN。
- `--controller mission` 使用本地 `NmpcMissionController` gate。
- `--require-real-model --quality-require-validation` 会拒绝 bootstrap、低质量模型，或缺少独立验证指标的模型。
- `--takeoff --land` 表示 XYZ 阶段自己起飞、自己降落。

干跑 XYZ，不连接飞机、不发杆量：

```bash
python3 -m rmtt_control.xyzway_nmpc \
  --source static \
  --controller mission \
  --waypoints waypoints/line-guide-diagonal-3d.json \
  --max-waypoint-sec 2 \
  --log-csv xyzway_dryrun.csv
```

## 航线文件

默认纳入仓库的实飞航线是：

```text
waypoints/line-guide-diagonal-3d.json
```

这条航线从原点出发，经过四个对角/角点，再回到原点。最大水平坐标绝对值是 `1.1m`，最高高度是 `1.5m`，匹配 README 中 `--field-limit 1.5 --z-max 2.0` 的默认安全边界。

自定义航线可以是列表：

```json
[
  {"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0, "hold_sec": 1.0}
]
```

也可以是带 `waypoints` 字段的对象：

```json
{
  "waypoints": [
    {"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0, "hold_sec": 1.0}
  ]
}
```

## 常用单独工具

扫描 IP：

```bash
python3 scripts/scan_ip.py
```

配网：

```bash
python3 scripts/wifi_test.py \
  --local-ip "<电脑连接RMTT热点时的本机IP>" \
  --ssid "<目标路由SSID>" \
  --password "<目标路由密码>"
```

起飞降落测试：

```bash
python3 scripts/takeoff_land.py \
  --confirm-risk
```

这个测试会调用 RoboMaster SDK 的起飞/降落接口。

## 重要边界

- 本仓库不依赖 ROS 控制链路；VRPN 只作为定位输入。
- 真实发杆量必须显式加 `--send --confirm-risk`。
- 起飞和降落使用 RoboMaster SDK，不用杆量模拟。
- 默认模型是 bootstrap 模型，只适合干跑和链路检查；真实 XYZ 飞行前必须先完成自主辨识并通过模型质量检查。
- 当前推荐把“自主辨识”和“飞 XYZ”分成两个独立阶段，不做空中交接。
