# Quest 项目统一交接

更新时间：2026-07-22

## 唯一工作上下文

- 工作机：`mc509@192.168.50.167`
- 项目根目录：`/home/mc509/Workspace/VLA/quest`
- ROS2 环境：`quest/quest_ros2_env.sh`
- 本交接文件是后续继续项目的唯一摘要；此前分步 session、brief、report、review 均已合并清理。

## 用户确认的边界

- 不修改原 LeRobot 项目、Piper SDK、CAN、ROS/URDF/模型层、APK、EvoDepth 或相机逻辑。
- VR 录制程序只订阅 ROS2 状态，不启动机械臂、不连接 CAN、不启动相机。
- 原项目 `record.py` 保持不变；VR 数据集写入逻辑放在 `quest/piper-scripts/`。

## 已完成：VR → LeRobot 录制适配

文件：

- `piper-scripts/vr_record_piper.py`
- `piper-scripts/record_vr_piper.sh`
- `piper-scripts/tests/test_vr_record_piper.py`

数据来源：

- `/joint_command`：动作
- `/piper_measured_joint_state`：观测
- 两者都要求类型严格为 `sensor_msgs/msg/JointState`

数据格式：

- `observation.state`：7 维 float32
- `action`：7 维 float32
- 关节名：`joint_1..joint_6`、`gripper`
- 不录制图像
- 每回合按固定 FPS 写入 LeRobotDataset

启动：

```bash
cd /home/mc509/Workspace/VLA/quest
piper-scripts/record_vr_piper.sh 数据集名 任务名 回合数
```

常用参数：

```bash
piper-scripts/record_vr_piper.sh 数据集名 任务名 回合数 \
  --fps 30 \
  --episode-time-s 60 \
  --max-age-s 0.25 \
  --topic-timeout-s 5
```

续录使用 `--resume`。默认数据目录为：
`/home/mc509/Workspace/VLA/quest/datasets/<数据集名>`。

## 验证结果

- 单元测试：`23 passed`
- Python 编译检查：通过
- shell 语法检查：通过
- `--help` 无硬件启动检查：通过
- ROS 话题缺失/错误类型会在有限超时后明确报错
- 未启动真实机械臂、CAN、Piper、相机或 EvoDepth

## 下一步

1. 先启动现有 VR 控制链路，确认两个 ROS2 话题实际存在且类型正确。
2. 在机械臂安全、急停可用的情况下，用一个短回合做真实录制测试。
3. 检查生成数据的 7 维观测/动作是否同步、无空帧，再进行正式采集。

真实硬件测试前必须由用户明确确认，并保持机械臂周围安全。
