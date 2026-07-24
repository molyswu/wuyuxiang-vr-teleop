#!/usr/bin/env bash
set -eo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "请不要直接以 root 运行此脚本；脚本会在需要时调用 sudo。"
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  echo "不支持的系统：${PRETTY_NAME}"
  echo "本脚本只允许在 Ubuntu 22.04 上安装 ROS2 Humble。"
  exit 1
fi

echo "将安装 ROS2 Humble 及项目迁移所需的基础包。"
echo "不会修改 shell 启动文件，不会删除 ROS1，不会修改项目目录。"
read -r -p "继续？[y/N] " answer
[[ "${answer}" =~ ^[Yy]$ ]] || { echo "已取消。"; exit 0; }

sudo -v

sudo apt update
sudo apt install -y curl software-properties-common
sudo add-apt-repository -y universe
sudo apt update

ros_apt_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' | head -n 1)"
if [[ -z "${ros_apt_source_version}" ]]; then
  echo "无法获取 ROS 官方软件源版本号，请检查工作机网络后重试。"
  exit 1
fi

ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
deb_file="/tmp/ros2-apt-source.deb"
curl -fL -o "${deb_file}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb"
sudo dpkg -i "${deb_file}"
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-rosdep

source /opt/ros/humble/setup.bash
printf '\nROS2 安装验证：\n'
ros2 --help >/dev/null
ros2 doctor --report 2>/dev/null | sed -n '1,80p' || true
echo
echo "ROS2 Humble 安装完成。"
echo "本脚本没有修改 ~/.bashrc；使用时请执行：source /opt/ros/humble/setup.bash"
