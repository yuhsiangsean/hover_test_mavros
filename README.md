# hover_test_mavros

`hover_test`（PX4 + uXRCE-DDS）的 **ArduPilot + MAVROS** 版本：用鍵盤指令讓單架無人機
GUIDED → 解鎖 → 起飛懸停 → 水平移動 / 轉向 → 降落。

驗證環境：Pixhawk 6C、ArduCopter 4.7.2、ROS 2 Humble（Jetson 容器）、OptiTrack 外部定位、
MAVROS 經 TELEM2（MAVLink2，921600）連線。

```
hover_commander --[/hover_test/command, /hover_test/yaw_command, /hover_test/position_command]-->
hover_control   --[/mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/takeoff,
                    /mavros/setpoint_raw/local]--> MAVROS --MAVLink--> ArduPilot
```

## 座標系

ROS 這一側全部是 **ENU、z 朝上**，MAVROS 自己轉成 ArduPilot 的 NED，程式裡不做任何轉換。

| 項目 | 定義 |
|---|---|
| x、y | 動捕（Motive）的 x、y 軸；原點 = **動捕原點**（不是起飛點） |
| z | 朝上；懸停高度 = 起飛點 z + `hover_altitude_m` |
| 航向 | 0° = 動捕 +X，**逆時針為正** |

## 指令（在 `hover_commander` 視窗輸入）

| 指令 | 作用 | 可用狀態 |
|---|---|---|
| `1` | 記下起飛點 → GUIDED → 解鎖 → 起飛到 `hover_altitude_m` | `idle` |
| `2` | 切 `LAND`，ArduPilot 降落並在觸地後自動上鎖 | `taking_off`、`hovering` |
| `3` | 目標往動捕 **+X** 移 `horizontal_step_m` | `hovering` |
| `4` | 目標往動捕 **+Y** 移 `horizontal_step_m` | `hovering` |
| `5 <deg>` | 航向設為 `<deg>`（0 = 動捕 +X，逆時針為正），以 `yaw_rate_dps` 走最短路徑轉過去 | 任何狀態（懸停時才送出） |
| `6 <x> <y>` | 飛到**動捕絕對座標** (x, y)（m）；`6 0 0` = 動捕原點 | `hovering` |

- 水平目標（3 / 4 / 6）以 `horizontal_speed_mps` 漸進，不會直接跳過去。
- 目標離目前位置超過 `max_goal_distance_m`（預設 3 m）會被拒絕並印警告。
- 沒送過 `5` 之前不控制航向（`IGNORE_YAW`），飛機維持解鎖時的航向。
- `hover_control` 按 `Ctrl+C`：先切 `LAND` 再結束程式。ArduPilot 會自己完成降落與上鎖。

## 狀態機

| 狀態 | 行為 |
|---|---|
| `idle` | 在地上，不送任何東西給飛控 |
| `taking_off` | 請求 `GUIDED`（5 秒內沒進入就放棄）→ 請求解鎖（失敗 5 次放棄）→ 解鎖後**立刻**送 `cmd/takeoff`（ArduPilot 在地上時，GUIDED 位置目標不會讓它起飛；解鎖後 `DISARM_DELAY` 內沒起飛會自動上鎖）→ 高度到懸停高度 −`hover_reached_tol_m` 進入 `hovering` |
| `hovering` | 每 0.1 s 送 `/mavros/setpoint_raw/local`，目標往 goal 漸進 |
| `landing` | 請求 `LAND`，等到上鎖回 `idle` |

**遙控器接管：** 程式把飛機切進 GUIDED（或 LAND）之後，只要模式被遙控器 / 地面站改成別的，
程式就停止送任何指令並回到 `idle`，不會把模式搶回來。切到 `LAND` 則視為降落，等它上鎖。

## 參數（`hover_control`，可用 `--ros-args -p` 覆蓋）

| 參數 | 預設 | 說明 |
|---|---|---|
| `mavros_ns` | `/mavros` | MAVROS 命名空間（啟動 MAVROS 時的 `namespace:=mavros`） |
| `hover_altitude_m` | `0.5` | 懸停高度（起飛點上方，m） |
| `horizontal_speed_mps` | `0.2` | 水平目標漸進速度 |
| `horizontal_step_m` | `0.5` | 指令 3 / 4 的步長 |
| `yaw_rate_dps` | `30.0` | 指令 5 的轉向速度（deg/s） |
| `max_goal_distance_m` | `3.0` | 水平目標與目前位置的最大距離 |
| `hover_reached_tol_m` | `0.1` | 判定到達懸停高度的容許誤差 |

爬升速度由 ArduPilot 的起飛控制決定，不是這個節點的參數。

## 建置

```bash
cd ~/shared/ros2_ws/src
git clone https://github.com/yuhsiangsean/hover_test_mavros.git
cd ~/shared/ros2_ws
colcon build --packages-select hover_test_mavros
source install/setup.bash
```

需要 `ros-humble-mavros`、`ros-humble-mavros-extras`。

## 執行

先啟動 MAVROS + 動捕（`drone_mocap` 的 launch），確認 `/mavros/state` 的 `connected: true`、
`/mavros/local_position/pose` 有值：

```bash
ros2 launch ~/shared/ros2_ws/src/drone_mocap/launch/mocap_mavros.launch.py
```

再開兩個終端機：

```bash
ros2 run hover_test_mavros hover_control
ros2 run hover_test_mavros hover_commander
```

## 第一次測試的順序

1. **拆槳**。只輸入 `1`，看 `hover_control` 的 log：`GUIDED` → `ARM accepted` → `TAKEOFF accepted`；
   接著立刻輸入 `2`，確認切到 `LAND` 並上鎖。
2. 解鎖被拒絕時看 `ros2 topic echo /mavros/statustext/recv` 的 `PreArm:` 訊息
   （例如 `RC not found`、`Battery 1 unhealthy`），處理完再試。
3. 裝槳後：低高度、動捕範圍中央、遙控器拿在手上（隨時可切 Stabilize / Land 接管）。

## 和 hover_test（PX4）的差別

| | hover_test（PX4） | hover_test_mavros（ArduPilot） |
|---|---|---|
| 介面 | `px4_msgs`、`/MAV1/fmu/in/*`、`/MAV1/fmu/out/*` | `mavros_msgs`、`/mavros/*` |
| 外部控制模式 | OFFBOARD（需先持續送 setpoint） | GUIDED |
| 起飛 | 高度 setpoint 漸進 | `cmd/takeoff`（MAV_CMD_NAV_TAKEOFF） |
| 降落 | 自己漸進降高度再 disarm | `LAND` 模式，ArduPilot 自動上鎖 |
| 座標 | NED，相對起飛點 | ENU，動捕絕對座標 |
| 航向 | 順時針為正 | 逆時針為正（動捕 +X = 0°） |
| 航向不控制 | `yaw = NaN` | `type_mask` 加 `IGNORE_YAW` |
| QoS | BEST_EFFORT + TRANSIENT_LOCAL | `qos_profile_sensor_data` |

## Test

```bash
colcon test --packages-select hover_test_mavros
colcon test-result --verbose
```
