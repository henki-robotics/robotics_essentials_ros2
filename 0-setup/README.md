# Exercises 0 - Setup

## Prerequisites
- **Ubuntu 24.04 LTS** (configured for **ROS 2 Jazzy** and **Gazebo Harmonic**)
- Intel or AMD processor (x64 architecture). The pre-configured Docker image targets x64 platforms and will not run natively on Apple Silicon, Raspberry Pi, or Jetson boards.
  - For Apple Silicon users, see [this tutorial](apple_silicon_UTM_installation_steps.md) for setting up Ubuntu in a virtual machine with UTM.
- A computer with sufficient processing power to run Gazebo Harmonic smoothly.
- [Docker](https://docs.docker.com/engine/install/ubuntu/) (recommended) or a native Ubuntu 24.04 + ROS 2 Jazzy installation.
- Git (`sudo apt install git`)

<details>
  <summary>Docker installation instructions</summary>

  ### Installing Docker
  1. Set up Docker's `apt` repository by running these commands (one-by-one) in a new terminal:
```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

```

2. Install the Docker packages:

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

```

3. Verify your installation:

```bash
sudo docker run hello-world

```

### Run `docker` commands without `sudo`

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
docker run hello-world

```

### (Optional) NVIDIA GPU Support (NVIDIA Container Toolkit)

If you have an NVIDIA GPU with proprietary drivers installed on your Ubuntu host, install the NVIDIA Container Toolkit and uncomment the `runtime: nvidia` and `NVIDIA_*` lines in `docker/docker-compose.yaml` for hardware-accelerated Gazebo rendering:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
&& curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

```

</details>

---

## Option A: Docker Setup (Recommended)

All practical examples in this course are containerized with Docker (`osrf/ros:jazzy-desktop-full`), so no local ROS 2 or Gazebo installation is required.

1. Clone the repository (on the `jazzy` branch):
```bash
git clone -b jazzy https://github.com/henki-robotics/robotics_essentials_ros2.git

```


2. Create a workspace for your exercises (mounted automatically into the container):
```bash
mkdir -p $HOME/exercises_ws/src

```


3. Grant X11 display permissions to Docker containers (must be run once after every reboot):
```bash
xhost +

```


4. Build and launch the Docker container:
```bash
cd robotics_essentials_ros2/docker/
docker compose up

```


Wait until the container builds and starts.
5. Open a new terminal inside the running Docker container:
```bash
docker exec -it robotics_essentials_ros2 bash

```



---

## Option B: Native Ubuntu 24.04 Setup (Without Docker)

If you already have **Ubuntu 24.04** with **ROS 2 Jazzy** installed natively on your machine and prefer running without Docker, use the provided setup script. It installs all required Jazzy & Gazebo Harmonic packages, clones `andino` and `andino_gz`, ignores physical hardware packages, applies the Jazzy simulation physics & Nav2 patch (`docker/patches/andino_gz.patch`), and builds both `~/ros2_ws` and `~/exercises_ws`:

```bash
git clone -b jazzy https://github.com/henki-robotics/robotics_essentials_ros2.git
cd robotics_essentials_ros2/0-setup
./setup_jazzy_native.sh

```

> **Note:** If you are using a Wayland desktop session on Ubuntu 24.04, make sure `export QT_QPA_PLATFORM=xcb` is set in your `~/.bashrc` so Gazebo and RViz open reliably.

---

## Run the Simulation

1. Launch the Andino Gazebo Harmonic simulation:
```bash
ros2 launch andino_gz andino_gz.launch.py

```



On the first startup, Gazebo downloads world/model assets, so ensure you have an active internet connection. You may see `[ros_gz_sim]: Requesting list of world names` in the console while assets download.

2. Press the **Play** button in the bottom-left corner of the Gazebo window to unpause the physics simulation.

You have now set up everything needed for the exercises!

Next exercises: [Exercises 1: ROS 2 Introduction](/1-ros_2_introduction/README.md).
