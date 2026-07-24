# Quest 单臂 Piper ROS2 运行说明

所有操作都在 `questVR_ws_ros2` 副本中进行。原始 ROS1 工作区不使用、不修改。

## 1. 激活环境

```bash
cd /home/mc509/Workspace/VLA/quest
source quest_ros2_env.sh
```

## 2. 先运行安全空发送模式

该模式读取 Quest 并做 IK，只发布 `/joint_command`，不会连接 CAN 或使能机械臂：

```bash
ros2 run quest_teleop_ros2 quest_single_piper_node
```

默认 USB ADB。使用无线 ADB 时：

```bash
ros2 run quest_teleop_ros2 quest_single_piper_node --ros-args -p quest_ip:=<QUEST_IP>
```

按右手柄 A 键记录基准位姿，按住 B 键开始发布单臂目标，右扳机控制夹爪量。

## 3. 连接真实机械臂前的显式开关

确认 CAN 名称、急停和机械臂当前状态后，才允许 SDK 连接：

```bash
ros2 run quest_teleop_ros2 quest_single_piper_node --ros-args \
  -p can_name:=can0 -p allow_hardware:=true
```

未设置 `allow_hardware:=true` 时，SDK 不会打开 CAN、使能机械臂或发送关节指令。
