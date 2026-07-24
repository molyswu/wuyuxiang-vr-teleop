# VR LeRobot Recording Design

日期：2026-07-22

## 目标

在 Quest 项目 `piper-scripts/` 中新增一套独立的数据集录制入口，复用
`Piper/lerobot-piper/src/lerobot/scripts/lerobot_record.py` 的 episode、帧、任务和
LeRobotDataset 写盘逻辑，但把 teleoperation 输入改为现有 VR ROS2 控制链路。

第一版只记录关节和动作，不记录摄像头，不启动 WebRTC，不占用 EvoDepth 的相机资源。

## 不可修改范围

- 不修改 `Piper/lerobot-piper` 的任何文件。
- 不修改 `quest/lerobot` 的任何文件。
- 不修改 Piper SDK、CAN 驱动、URDF、Xacro、CSV 或网格模型。
- 不启动第二个 Piper SDK/CAN 连接，不直接向机械臂发送控制帧。
- 不修改 ROS1/ROS2 机械臂底层节点或 EvoDepth 任务。

## 运行架构

```text
vr_piper_test.sh
  Quest APK -> Quest ROS2 client -> /joint_command -> Piper daemon -> Piper SDK/CAN -> arm
                                      |
                                      +-> VR recorder subscribes

Piper daemon -> /piper_measured_joint_state -> VR recorder subscribes

VR recorder -> LeRobotDataset files on disk
```

VR recorder 是只读 ROS2 订阅者：

- `/joint_command` (`sensor_msgs/msg/JointState`) 的前六个位置值作为 `action`。
- `/piper_measured_joint_state` (`sensor_msgs/msg/JointState`) 的前六个位置值作为
  `observation.state`。
- 第七个位置值（存在时）作为 gripper；缺失时使用 `0.0`。
- recorder 不创建 Piper SDK 对象，不调用 `ConnectPort`、`EnablePiper` 或
  `JointCtrl`。

## 新文件

- `piper-scripts/vr_record_piper.py`
  - 独立 ROS2 recorder 节点。
  - 创建或追加 `LeRobotDataset`。
  - 以固定 FPS 采集最新 observation/action 快照。
  - 处理 episode 时长、episode 数量、任务名、保存和 Ctrl+C 收尾。
- `piper-scripts/record_vr_piper.sh`
  - 激活 `vt` 环境和 `quest/lerobot/src`。
  - 检查 ROS2 话题存在，不负责启动 VR 客户端或 daemon。
  - 将命令行参数传给 Python recorder。

## 数据字段

第一版不使用视频特征，数据字段保持固定：

```text
observation.state: float[7]
action:            float[7]
task:              string
```

七个元素顺序为：

```text
[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, gripper]
```

关节单位使用当前 ROS2/Piper 链路输出的弧度，gripper 使用当前链路的米单位。
每个 frame 的 action 是 VR 客户端实际发布给 daemon 的目标，不是 recorder 自己重新计算
的目标；observation 是 daemon 发布的机械臂实测状态。

## 采样与同步

- 默认录制 FPS：30。
- recorder 每个采样周期读取最近一条 observation 和 action。
- 在收到第一条有效 observation 和 action 之前不写 frame。
- 若某个输入超时，停止写入并记录错误，避免写入错配数据。
- observation 和 action 使用同一采样时刻写入同一 frame；不重复开启 CAN 或读取 SDK。

## Episode 行为

- `--dataset.repo_id`、`--dataset.root`、`--dataset.single_task`、`--dataset.num_episodes`
  和 `--dataset.fps` 映射到 LeRobotDataset 配置。
- `--episode_time_s` 控制单个 episode 录制时间。
- 每个 episode 结束后调用 `save_episode()`。
- 追加已有数据集时加载已有元数据并校验字段/FPS兼容性。
- Ctrl+C 执行 `finalize()`，不停止 Piper daemon，不让 recorder 触碰机械臂使能状态。

## 启动顺序

终端一：

```bash
cd /home/mc509/Workspace/VLA/quest
./vr_piper_test.sh
```

终端二：

```bash
cd /home/mc509/Workspace/VLA/quest
./piper-scripts/record_vr_piper.sh \
  my_dataset \
  "task description" \
  5
```

VR 控制客户端必须先运行并发布两个 ROS2 话题；录制器只负责写盘。

## 错误处理

- ROS2 话题不存在：立即退出，不创建空数据集。
- observation/action 未同步：等待并提示，不写 frame。
- 消息维度小于 6：丢弃该消息并记录警告。
- 数据集字段或 FPS 不兼容：退出，不追加写入。
- Ctrl+C：保存当前 LeRobotDataset 的可完成状态并退出；不调用 Piper SDK。

## 测试

实现前先增加无硬件测试，覆盖：

1. 六关节加 gripper 的 ROS JointState 转换。
2. 缺失 gripper 时补零。
3. 不足六关节的消息被拒绝。
4. 没有 observation/action 时不写 frame。
5. 收到同步输入后写入正确的 observation/action/task 字段。
6. recorder 不导入或实例化 `piper_sdk`。
7. shell 启动器语法和 dry-run 参数检查。

测试通过后才做无硬件 ROS2 话题回放，最后再进行小规模真实录制。
