#!/usr/bin/env bash
set -eo pipefail

packages=(
  python3-catkin-pkg
  python3-rospkg
  python3-rosdistro
  python3-catkin
  python3-rosmaster
  python3-roslib
  python3-roslaunch
  python3-rosgraph
  python3-rosclean
)

echo "将移除已确认的旧 ROS1 Python 依赖包："
printf '  %s\n' "${packages[@]}"
echo "不会使用 --force-overwrite，不会删除项目或 Conda 环境。"
read -r -p "继续？[y/N] " answer
[[ "${answer}" =~ ^[Yy]$ ]] || { echo "已取消。"; exit 0; }

sudo -v
sudo dpkg --remove --force-depends "${packages[@]}"
sudo apt-get -f install -y
sudo dpkg --configure -a

source /opt/ros/humble/setup.bash
ros2 --help >/dev/null
echo "ROS2 dpkg 修复完成。"
