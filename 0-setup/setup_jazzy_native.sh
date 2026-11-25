#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
ROS2_WS="${HOME}/ros2_ws"
EXERCISES_WS="${HOME}/exercises_ws"

echo "=== Installing ROS 2 Jazzy & Gazebo Harmonic dependencies ==="
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  git \
  python3-pip \
  python3-setuptools \
  python3-matplotlib \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-turtle-nest \
  ros-jazzy-tf-transformations \
  ros-jazzy-teleop-twist-keyboard

mkdir -p "${ROS2_WS}/src" "${EXERCISES_WS}/src"

echo "=== Copying course packages into ${ROS2_WS}/src ==="
cp -r "${REPO_ROOT}/packages/"* "${ROS2_WS}/src/"

cd "${ROS2_WS}/src"

if [ ! -d "andino" ]; then
  git clone https://github.com/Ekumen-OS/andino.git
  git -C andino checkout 974264b
fi

touch andino/andino_base/COLCON_IGNORE \
      andino/andino_bringup/COLCON_IGNORE \
      andino/andino_firmware/COLCON_IGNORE \
      andino/andino_hardware/COLCON_IGNORE

if [ ! -d "andino_gz" ]; then
  git clone -b jazzy https://github.com/Ekumen-OS/andino_gz.git
  git -C andino_gz checkout 073dc67
fi

echo "=== Applying Jazzy physics & Nav2 patch to andino_gz ==="
if git -C andino_gz apply --check "${REPO_ROOT}/docker/patches/andino_gz.patch" 2>/dev/null; then
  git -C andino_gz apply "${REPO_ROOT}/docker/patches/andino_gz.patch"
else
  echo "Patch already applied or skipped."
fi

echo "=== Installing rosdep dependencies and building workspaces ==="
source /opt/ros/jazzy/setup.bash
cd "${ROS2_WS}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install

cd "${EXERCISES_WS}"
source "${ROS2_WS}/install/setup.bash"
colcon build --symlink-install

echo ""
echo "Setup complete! Add the following lines to your ~/.bashrc if not already present:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${ROS2_WS}/install/setup.bash"
echo "  source ${EXERCISES_WS}/install/setup.bash"
echo "  export QT_QPA_PLATFORM=xcb"
