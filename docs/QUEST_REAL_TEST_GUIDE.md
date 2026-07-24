# Quest VR 控制 Piper：真实测试操作手册

更新时间：2026-07-21

## 1. 当前控制链路

```text
Quest 旧版 APK
    ↓ ADB / 位姿日志
ROS2 Quest 客户端
    ↓ /joint_command
Piper daemon
    ↓ Piper SDK / CAN
Piper 机械臂
```

当前使用固定头显、右手柄控制：

- 头显戴在脖子上并固定，测试时不要移动头显。
- 只移动右手柄，不使用头显位姿作为操控输入。
- 按住右手柄 B 才允许机械臂跟随。
- 松开 B 后保持最后有效目标，不应突然下落。
- A+B 是归位操作，真实测试时不要误按。
- 不启动摄像头、WebRTC、精细模式或 VR 实时调参界面。

## 2. 推荐的真实测试启动方式

工作机终端执行：

```bash
cd /home/mc509/Workspace/VLA/quest
./vr_piper_test.sh
```

脚本会依次完成：

1. 检查 `can0` 是否存在。
2. 检查 Quest 是否通过 ADB 连接。
3. 等待安全确认。
4. 启动或复用 Piper daemon。
5. 启动 ROS2 VR 控制客户端。

看到提示后，确认机械臂周围安全，再输入：

```text
YES
```

然后按住右手柄 B，先做小幅度移动。

## 3. 测试前检查

机械臂周围必须清空，急停可用，机械臂当前姿态稳定。

检查 CAN：

```bash
ip -details link show can0
```

正常应看到：

```text
can state ERROR-ACTIVE
bitrate 1000000
sample-point 0.750
```

检查 Quest：

```bash
adb devices -l
```

应看到 Quest 状态为 `device`。

检查当前进程：

```bash
ps -eo pid,stat,cmd | grep -E "piper_daemon|quest_single_piper" | grep -v grep
```

正常情况下，运行测试时有一个 `piper_daemon` 和一个 Quest 客户端。

## 4. 手动启动或重连 CAN0

通常不需要手动配置，`vr_piper_test.sh` 会调用 `piper_daemon_start.sh`。

如果 USB-CAN 重新插拔后发送失败，先停止旧 daemon：

```bash
cd /home/mc509/Workspace/VLA/quest
./piper_daemon_stop.sh
```

这一步会让 Piper 暂时失能。

然后手动配置 `can0`：

```bash
sudo ip link set can0 down
```

```bash
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.750
```

确认状态：

```bash
ip -details link show can0
```

重新建立 Piper SDK/CAN 会话：

```bash
cd /home/mc509/Workspace/VLA/quest
./piper_daemon_start.sh
```

之后再运行 `vr_piper_test.sh`。不要在 daemon 正在使用 CAN 时直接 down/up `can0`。

## 5. 只控制右手柄的正确姿势

1. 头显固定在脖子上，头部保持相对稳定。
2. 右手柄放在舒适、可持续追踪的位置。
3. 不要用头显移动来“带动”机械臂。
4. 松开 B 时不要移动手柄；重新按 B 时先保持手柄静止约 1 秒。
5. 按住 B 后只做小幅度移动，确认机械臂响应后再扩大动作。
6. 看到机械臂接近工作空间边界时，立即减小动作，不要持续顶住边界。

## 6. 当前 VR 参数表

以下是 `quest_single_piper_node.py` 支持的参数。`vr_piper_test.sh` 传入的值是当前真实测试的有效值；未由脚本传入的参数使用节点默认值。

| 参数 | 当前测试值 | 作用 | 优化方向与注意事项 |
|---|---:|---|---|
| `translation_scale` | `1.2` | 手柄平移到机械臂平移的比例 | 越大，手柄移动越小也能产生更大机械臂位移；过大容易过灵敏。 |
| `rotation_scale` | `0.5` | 手柄旋转到末端旋转的比例 | 越大旋转越灵敏；当前先保持不变。 |
| `pose_smoothing_alpha` | `0.20` | 位姿平滑响应速度 | 越大响应快但噪声更明显；越小更稳但延迟更大。 |
| `rotation_smoothing_alpha` | `0.18` | 仅腕部旋转的平滑响应 | 越小旋转更稳但腕部响应更慢；平移不受影响。 |
| `fast_translation_response_alpha` | `0.30` | 大幅平移时的快速响应系数 | 只作用于大动作，减小跟随延迟。 |
| `fast_rotation_response_alpha` | `0.26` | 大幅旋转时的快速响应系数 | 只作用于大旋转，过大可能增加抖动。 |
| `wrist_pivot_offset_m` | `[0.0, 0.0, 0.0]` | 手柄追踪点到虚拟腕点的本地坐标偏移（米） | 先保持 0；标定后填写，方向错误会引入平移。 |
| `max_tracking_age_sec` | `0.12` | APK 位姿数据最大允许间隔（秒） | 超时后暂停跟随并保持当前位置；过大可能使用旧位姿。 |
| `b_anchor_settle_sec` | `0.16` | B 按下后的基准稳定等待时间（秒） | 等待手柄稳定后才开始控制。 |
| `b_anchor_max_translation_m` | `0.01` | 建立基准允许的位移变化（米） | 默认 1 cm。 |
| `b_anchor_max_rotation_rad` | `0.08` | 建立基准允许的旋转变化（弧度） | 默认约 4.6°。 |
| pose_filter_window | 3 | 原始手柄位姿窗口滤波帧数 | 3 帧降噪；增大将增加延迟。 |
| `pose_deadband_m` | `0.005` | 平移噪声死区，单位米 | 增大可减小抖动，但会损失小动作精度。 |
| `pose_deadband_rad` | `0.040` | 旋转噪声死区，单位弧度 | 增大可减小末端旋转抖动。 |
| `max_input_translation_step_m` | `0.03` | 单帧最大输入平移步长 | 减小更稳但更慢；增大更灵敏但更容易突跳。 |
| `max_input_rotation_step_rad` | `0.16` | 单帧最大输入旋转步长 | 减小可抑制旋转突跳。 |
| `max_input_discontinuity_m` | `0.15` | VR 跟踪跳变拒绝阈值，单位米 | 太小会频繁冻结；太大可能接受错误追踪跳变。 |
| `max_input_discontinuity_rad` | `0.60` | VR 姿态跳变拒绝阈值 | 用于保护错误姿态输入，不建议随意增大。 |
| `adaptive_translation_threshold_m` | `0.015` | 大动作/小动作自适应切换阈值 | 影响响应和稳定性的平衡。 |
| `adaptive_rotation_threshold_rad` | `0.08` | 姿态自适应切换阈值 | 增大可能提高响应，但会放大突变影响。 |
| `max_rotation_step_rad` | `0.16` | 单次最大末端旋转步长 | 增大可减少大幅旋转延迟，过大可能增加抖动。 |
| `max_cartesian_step_m` | 默认 `0.020` | 末端目标插值的最大平移步长 | 减小更平滑，增大更快；用于抑制大范围跳变。 |
| `workspace_min` | `[0.040,-0.65,0.040]` | 工作空间下限 `[X,Y,Z]`，单位米 | 不要超过机械臂真实安全范围。 |
| `workspace_max` | `[0.75,0.65,0.75]` | 工作空间上限 `[X,Y,Z]`，单位米 | 不要直接扩大，先确认机械臂和 IK 安全。 |
| `workspace_margin_m` | `0.02` | 工作空间内侧安全边距 | 增大可提前远离边界，减少边界抖动。 |
| reach_limit_m | 0.15 | 当前末端与目标位置的最大距离 | 超出部分丢弃，反向立即生效。 |
| `reach_limit_rad` | `0.80` | 当前末端与目标姿态的最大角度误差 | 防止姿态目标在关节限制附近累积。 |
| `speed_rate` | `10` | Piper SDK 速度等级 | 机械臂底层发送参数；只在安全测试中调整。 |
| max_joint_step_rad | 客户端 0.005；daemon 0.006 | 单次关节目标最大变化 | 越小越稳但跟随慢；50 Hz 下使用 0.006。
| command_rate_hz | 50 | Piper daemon 向 CAN 写入指令的频率 | 当前安全测试值；daemon 实际发送频率，不是 VR 客户端频率。 |
| `require_b_button` | `true` | 是否必须按住 B 才能控制 | 保持 `true`。 |
| `hold_when_b_released` | `true` | 松开 B 后保持最后目标 | 保持 `true`，防止机械臂突然下落。 |
| `home_requires_b` | `true` | A+B 才允许归位 | 保持 `true`，避免误触 A 归位。 |
| `b_debounce_sec` | 默认 `0.10` | B 键按下去抖时间 | B 键误触发时再调，不是首选调参项。 |
| `b_release_debounce_sec` | 默认 `0.80` | B 键松开去抖时间 | 过大会让松开响应变慢。 |
| `ik_max_joint_jump_deg` | 默认 `120.0` | IK 关节跳变拒绝阈值 | 用于安全保护，不建议为了灵敏度提高。 |
| `disable_on_exit` | 默认 `false` | 客户端退出时是否失能 | 当前由 daemon 持续持有连接；不要改为 `true`。 |

### 最常用的调参顺序

1. 手柄移动太远：先调大 `translation_scale`。
2. 机械臂太灵敏：调小 `translation_scale`。
3. 机械臂抖动：先调小 `pose_smoothing_alpha` 或调大两个 `pose_deadband`。
4. 反应延迟：调大 `pose_smoothing_alpha`，但要观察噪声。
5. 到边界抖动：增大 `workspace_margin_m`，不要先扩大工作空间。
6. 大动作被拒绝：检查 VR 跟踪是否跳变，不要直接放宽跳变保护。
7. 关节变化太慢：最后才考虑 `max_joint_step_rad` 和 `speed_rate`。

## 7. 脚本归类

### 真实 VR 测试：只用这个

| 脚本 | 用途 |
|---|---|
| `vr_piper_test.sh` | 唯一推荐的真实 VR 测试入口；检查 CAN/ADB，启动 daemon 和 VR 客户端。 |

### Piper 连接管理：通常由测试脚本调用

| 脚本 | 用途 |
|---|---|
| `piper_daemon_start.sh` | 启动持久 Piper SDK/CAN daemon。 |
| `piper_daemon_stop.sh` | 停止 daemon，并让 Piper 失能。 |
| `quest_ros2_env.sh` | 设置 ROS2 和 Python 环境，不负责控制机械臂。 |
| `run_quest_single_piper.sh` | 底层 ROS2 VR 客户端启动器，通常不要单独运行。 |

### 不要用于当前 VR 实测

| 脚本 | 原因 |
|---|---|
| `vr_piper_test_auto_can.sh` | 旧的自动 CAN 版本，参数已不同，可能重置 CAN，不作为当前入口。 |
| `teleop_piper.sh` | LeRobot/Piper 遥操链路，不是当前 Quest VR 链路。 |
| `record_piper.sh` | LeRobot 数据集录制，不要和 VR 实测同时启动。 |
| `questVR_ws/src/...` 下的 ROS1 脚本 | 旧 ROS1 链路，不要和当前 ROS2 VR 链路混用。 |
| `install_ros2_humble_safe.sh` | 安装脚本，不是测试脚本。 |
| `repair_ros2_dpkg_safe.sh` | 系统修复脚本，不是测试脚本。 |
| `questVR_ws_ros2/src/Piper_ros/...` 下的 ROS2 Piper 示例 | 底层/示例节点，不要与 Piper daemon 同时启动。 |

## 8. 正常停止

只结束本次 VR 控制客户端：

```text
Ctrl+C
```

这不会自动停止 daemon。下一次测试可以直接再次运行 `vr_piper_test.sh`。

如果确实要让机械臂失能：先支撑机械臂，再执行：

```bash
cd /home/mc509/Workspace/VLA/quest
./piper_daemon_stop.sh
```

## 9. 常见故障判断

### CAN 没有发送

查看：

```bash
ip -s -details link show can0
tail -50 /tmp/quest_piper_daemon.log
```

如果 daemon 检测到 CAN 总线不在正常状态、SDK 发送异常或电机使能状态掉落，会锁存发送故障，立即停止后续电机指令并只报警，不自动重启、不自动复位。需要先检查 CAN 线、USB-CAN、供电、温度和 Piper 报警，再手动停止并重新启动 daemon。
如果看到 `SEND_MESSAGE_FAILED (100017)` 且 TX 为 `0`，先重插 CAN/Piper 线，再停止并重新启动 daemon；不要先改 VR 参数。

### B 键按了但不动

查看客户端日志中的：

```text
B:1
b_gate=1
ik=ok
```

如果是 `B:0`，说明 APK 没有把右手柄 B 输入传入；如果 `B:1` 但 CAN TX 为 `0`，说明是 Piper/CAN 发送链路问题。

### 出现 tracking jump

日志中的：

```text
VR tracking jump rejected; release B to re-center
```

表示程序主动拒绝了突跳输入。松开 B，保持手柄静止，再重新按住 B；不要立刻提高跳变阈值。

## 10. 严格不要修改的内容

- Piper SDK 源码
- CAN 驱动和底层协议
- Piper URDF、Xacro、CSV、网格和惯性参数
- ROS1 工作区
- EvoDepth 启动文件和参数
- APK 源码和 APK 包

所有 VR 优化优先只通过 `vr_piper_test.sh` 的 ROS2 参数完成。
