# Exercises 0 - Setup

## Prerequisites
- Ubuntu
- Intel or AMD processor (with x64 processor architecture). This is the reason why the Docker container won't work directly for example on Apple Silicon, Raspberry or Jetson platforms.
- A computer with sufficient processing power to run Gazebo simulation smoothly.
- [Docker](https://docs.docker.com/engine/install/ubuntu/). Follow the link for the official tutorial
and the latest installation instructions. Optionally, you can follow the commands below, but they might get out of date.
- Git (`sudo apt install git`)

<details>
  <summary>Docker installation instructions</summary>

  ### Installing Docker
  ### Firstly follow these steps:
  1. Set up Docker's `apt` repository by running these commands (one-by-one) in a new terminal:
```
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
2. Install the Docker packages by running the following command in the same terminal:
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
3. Verify your installation by running this command:
```
sudo docker run hello-world
```

### Follow these next steps to be able to run `docker` commands without "sudo"
1. Create a `docker` user group.
```
sudo groupadd docker
```
2. Add your user to the `docker` group.
```
sudo usermod -aG docker $USER
```
3. Log out and log back in to your system, or run the following command:
```
newgrp docker
```
4. Test that you can run `docker` commands without `sudo`.
```
docker run hello-world
```

### (Optional) If you are running Docker on a Linux machine equipped with an Nvidia GPU and the proper installed drivers
#### Install the Nvidia Container Toolkit by following these steps:
1. Configure the production repository:
```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
&& curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
Optionally, configure the repository to use experimental packages:
```
sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
2. Update the packages list from the repository:
```
sudo apt-get update
```
3. Install the NVIDIA Container Toolkit packages:
```
sudo apt-get install -y nvidia-container-toolkit
```

#### Configure the Nvidia Container Toolkit for Docker:
1. Configure the container runtime by using the nvidia-ctk command:
```
sudo nvidia-ctk runtime configure --runtime=docker
```
2. Restart the Docker daemon:
```
sudo systemctl restart docker
```  

</details>

If one does not have Ubuntu natively installed on their machine, and they do not want to mess with a dual-boot or 
virtualized installation of Ubuntu, we provide complete installation steps on how to create your own custom [live-Ubuntu 
installation on an external USB stick](USB_stick_instructions.md).

All the practical examples in this course are containerized with Docker, so no 
ROS 2 or Gazebo simulation installations are required!

## Installation

1. Clone the repository:

    ```commandline
    git clone https://github.com/henki-robotics/robotics_essentials_ros2.git
    ```

1. Create a new workspace for your exercises. This will be automatically mounted and available from the Docker container
    ```commandline
    mkdir -p $HOME/exercises_ws/src
    ```

## Launching the Docker container

Before anything run the following command in a new terminal. This will give display permissions to your containers:

```commandline
xhost +
```

> **Note:** This has to be run everytime you restart your computer 

1. Use docker compose to build and run the Docker container, which includes the ROS 2, Gazebo and simulated Andino installation.
    ```commandline
    cd robotics_essentials_ros2/docker/
    docker compose up
    ```

    Wait until the container has been successfully launched
    <img src="/images/docker_compose_up.png" alt="Andino Simulation Screenshot">

1. Open a new terminal with CTRL+ALT+T (or by using right click on Desktop -> "Open in Terminal") and run this command to start a new terminal inside the Docker container:
    ```commandline
    docker exec -it robotics_essentials_ros2 bash
    ```

The `docker compose up` command launches the `robotics_essentials_ros2` Docker container, which essentially acts as a virtual environment.
It is a completely isolated environment, that includes all the software dependencies you need, including even the Ubuntu operating system.

The `docker exec` command on the other hand opens a new terminal inside this Docker container, allowing you to interact with it.

Refer to this section whenever you need to relaunch your Docker container, or start a new terminal during the upcoming exercises.

## Run the simulation

1. Verify that everything is running correctly, by starting the simulation with:
    ```commandline
    ros2 launch andino_gz andino_gz.launch.py
    ```

<details open>
<summary>Note: If Gazebo doesn't open up in a few minutes, follow the instructions under this dropdown.</summary>

On the first startup, Gazebo will download some assets, so make sure you have a good internet connection.
You might get multiple `[ros_gz_sim]: Requesting list of world names` messages logged in the console.
During this time, Gazebo might freeze, but pressing "Wait" repeatedly should eventually solve the issue.

Some people have reported this issue being persistent. In this case, you can try to follow the proposed solution
in the [Gazebo GitHub issue #38](https://github.com/gazebosim/gz-sim/issues/38), to disable the firewall for
the duration of the installation using `sudo ufw disable` -command. This has previously helped to solve the issue.
</details>

2. Press the play button in simulation to start it.

   <img src="/images/andino_sim_screenshot.png" alt="Andino Simulation Screenshot">


You've now successfully installed everything you need for the upcoming exercises!

Next exercises: [Exercises 1: ROS 2 Introduction](/1-ros_2_introduction/README.md).
