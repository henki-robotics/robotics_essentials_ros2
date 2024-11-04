# Exercises 0 - Setup

### Prerequisites
- Ubuntu
- [Docker and `docker compose`](https://docs.docker.com/engine/install/ubuntu/). Follow the link for the official tutorial
and the latest installation instructions. Optionally, you can follow the commands below, but they might get out of date:

<details>
  <summary>Installing Docker and "docker compose"</summary>

  ### Installing Docker
  ### Firstly follow these steps:
  1. Set up Docker's `apt` repositoryby running these commands (one-by-one) in a new terminal:
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

### Install `docker-compose`
Run these commands in a new terminal. `docker-compose` will help us run Docker images in an easier way, and it also 
allows running multiple images at the same time.
```
sudo apt-get update
sudo apt-get install docker-compose-plugin
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


All the practical examples on this course are containerized with Docker, so no 
ROS 2 or Gazebo simulation installations are required!

### Installation

1. Clone the repository:

```commandline
git clone https://github.com/henki-robotics/robotics_essentials_ros2.git
```

2. Create a new workspace for your exercises. This will be automatically mounted and available from the Docker container
```commandline
mkdir -p $HOME/exercises_ws/src
```

3. Use docker compose to build and run the Docker container, which includes the ROS 2, Gazebo and simulated Andino installation.
```commandline
cd robotics_essentials_ros2/docker/
docker compose up 
```

Wait until the container has been successfully launched
<img src="/images/docker_compose_up.png" alt="Andino Simulation Screenshot">


4. Open a new terminal and start interactive Docker terminal access
```commandline
docker exec -it robotics_essentials_ros2 bash
```

5. Verify that everything is running correctly, by starting the simulation with
```commandline
ros2 launch andino_gz andino_gz.launch.py
```

**Note:** Gazebo might take a long while to start and open up.

6. Press the play button in simulation to start it

<img src="/images/andino_sim_screenshot.png" alt="Andino Simulation Screenshot">