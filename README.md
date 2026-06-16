# RMTT 非 ROS 控制工作区

这个工作区用于用 RoboMaster SDK 直接控制 RMTT，并把 NMPC 控制核心集成到本地 Python 链路中。飞行控制使用 RoboMaster SDK 的 `flight.rc(a,b,c,d)`，杆量范围是 `[-100, 100]`。

起飞和降落走 RoboMaster SDK 的 `drone.flight.takeoff()` / `drone.flight.land()`，不是手写杆量起降。辨识和 XYZ 飞行的定位都来自 VRPN，不使用无人机 OSD 位置或 ROS topic。

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

## 环境变量

不要把具体 IP、Wi-Fi 名称、密码、本机路径写死到代码或文档命令里。先在终端里按现场环境设置：

```bash
export RMTT_REPO_ROOT="$(pwd)"
export PYTHONPATH="$RMTT_REPO_ROOT"
export RMTT_IP="<扫描得到的RMTT_IP>"
export RMTT_WIFI_SSID="<目标路由SSID>"
export RMTT_WIFI_PASSWORD="<目标路由密码>"
export RMTT_AP_LOCAL_IP="<电脑连接RMTT热点时的本机IP>"
export RMTT_VRPN_TRACKER="<VRPN_TRACKER_NAME>"
export RMTT_VRPN_HOST="<VRPN_SERVER_HOST>"
export RMTT_VRPN_PORT="<VRPN_SERVER_PORT>"
```

如果需要运行 NMPC 源码对照审计，并且本机有参考源码目录，额外设置：

```bash
export NMPC_SOURCE_ROOT="<参考NMPC源码目录>"
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
cd "$RMTT_REPO_ROOT"
python3 -m rmtt_control.validate_workspace
```

这个命令不连接无人机，不发杆量。

## 1. 初始化：配网和查 IP

配网前，电脑需要先连到 RMTT 自己的 Wi-Fi 热点。

```bash
cd "$RMTT_REPO_ROOT"
python3 scripts/wifi_test.py \
  --local-ip "$RMTT_AP_LOCAL_IP" \
  --ssid "$RMTT_WIFI_SSID" \
  --password "$RMTT_WIFI_PASSWORD"
```

配网命令发出后，RMTT 会切到路由模式。然后电脑也切到同一个路由网络，再查飞机 IP：

```bash
python3 scripts/scan_ip.py
```

把扫描得到的飞机 IP 写入当前终端：

```bash
export RMTT_IP="<扫描得到的RMTT_IP>"
```

## 2. 飞行前检查

构建 VRPN helper：

```bash
cd "$RMTT_REPO_ROOT"
./build_vrpn_helper.sh
```

检查 VRPN 是否有 pose：

```bash
python3 -m rmtt_control.preflight_check \
  --check-vrpn-helper \
  --check-vrpn \
  --tracker "$RMTT_VRPN_TRACKER" \
  --host "$RMTT_VRPN_HOST" \
  --port "$RMTT_VRPN_PORT"
```

检查无人机连接和电量：

```bash
python3 -m rmtt_control.preflight_check \
  --check-drone \
  --ip "$RMTT_IP" \
  --min-battery 30
```

单独读电量：

```bash
python3 scripts/battery_monitor.py \
  --ip "$RMTT_IP" \
  --timeout 8
```

## 3. 阶段二：自主辨识

这个阶段是一次独立飞行：连接、起飞、辨识、降落、拟合模型、检查模型质量。

```bash
cd "$RMTT_REPO_ROOT"
python3 -m rmtt_control.identify_pipeline \
  --ip "$RMTT_IP" \
  --axes pitch,roll,throttle,yaw \
  --signals step \
  --amplitudes 10,20 \
  --field-limit 1.5 \
  --z-min 0.25 \
  --z-max 2.0 \
  --tracker "$RMTT_VRPN_TRACKER" \
  --host "$RMTT_VRPN_HOST" \
  --port "$RMTT_VRPN_PORT" \
  --method auto \
  --recenter \
  --send \
  --confirm-risk \
  --takeoff \
  --land \
  --fit \
  --backup \
  --quality-gate \
  --quality-fail-on-bootstrap
```

说明：

- `--send --confirm-risk` 才会真实发杆量。
- `--takeoff --land` 表示这个阶段自己起飞、自己降落。
- `--recenter` 会在 pitch/roll 激励后用 NMPC 回到初始 VRPN pose 附近。
- 辨识 CSV 会写到 `identify_run_时间戳/` 或你指定的 `--output-dir`。
- `--fit` 会更新 `models/rmtt_velocity_model.json`。
- `--quality-gate --quality-fail-on-bootstrap` 会在模型不可用时让命令失败。

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
  --min-samples 30 \
  --min-r2 0.2 \
  --max-nrmse 0.8
```

如果这里失败，不建议进入 XYZ 飞行阶段。先看辨识 CSV 质量、VRPN 是否稳定、激励幅度是否足够、飞行空间是否太小。

## 5. 阶段三：飞 XYZ 任务

这个阶段是另一架次独立飞行：检查模型、连接、起飞、飞 waypoint、降落。

```bash
cd "$RMTT_REPO_ROOT"
python3 -m rmtt_control.xyzway_nmpc \
  --ip "$RMTT_IP" \
  --model models/rmtt_velocity_model.json \
  --waypoints example_waypoints.json \
  --source vrpn \
  --controller mission \
  --tracker "$RMTT_VRPN_TRACKER" \
  --host "$RMTT_VRPN_HOST" \
  --port "$RMTT_VRPN_PORT" \
  --method auto \
  --field-limit 1.5 \
  --z-min 0.25 \
  --z-max 2.0 \
  --log-csv xyzway_run.csv \
  --require-real-model \
  --send \
  --confirm-risk \
  --takeoff \
  --land
```

说明：

- `--source vrpn` 表示 XYZ 飞行定位来自 VRPN。
- `--controller mission` 使用本地 `NmpcMissionController` gate。
- `--require-real-model` 会拒绝 bootstrap 或低质量模型。
- `--takeoff --land` 表示 XYZ 阶段自己起飞、自己降落。

干跑 XYZ，不连接飞机、不发杆量：

```bash
python3 -m rmtt_control.xyzway_nmpc \
  --source static \
  --controller mission \
  --waypoints example_waypoints.json \
  --max-waypoint-sec 2 \
  --log-csv xyzway_dryrun.csv
```

## waypoint 文件格式

`example_waypoints.json` 可以是列表：

```json
[
  {"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0, "hold_sec": 1.0}
]
```

也可以是：

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
  --local-ip "$RMTT_AP_LOCAL_IP" \
  --ssid "$RMTT_WIFI_SSID" \
  --password "$RMTT_WIFI_PASSWORD"
```

起飞降落测试：

```bash
python3 scripts/takeoff_land.py \
  --ip "$RMTT_IP" \
  --confirm-risk
```

这个测试会调用 RoboMaster SDK 的起飞/降落接口。

## 重要边界

- 本仓库不依赖 ROS 控制链路；VRPN 只作为定位输入。
- 真实发杆量必须显式加 `--send --confirm-risk`。
- 起飞和降落使用 RoboMaster SDK，不用杆量模拟。
- 默认模型是 bootstrap 模型，只适合干跑和链路检查；真实 XYZ 飞行前必须先完成自主辨识并通过模型质量检查。
- 当前推荐把“自主辨识”和“飞 XYZ”分成两个独立阶段，不做空中交接。
