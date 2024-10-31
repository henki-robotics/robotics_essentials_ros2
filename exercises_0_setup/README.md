# Exercises 0 - Setup

### Prerequisites
- Ubuntu
- [Docker and docker compose](https://docs.docker.com/engine/install/ubuntu/)

All the practical examples on this course are containerized with Docker, so no 
ROS 2 or Gazebo simulation installations are required!

### Installation

1. Clone the repository:

```commandline
git clone https://github.com/henki-robotics/robotics_essentials_ros2.git
```

2. Use docker compose to build and run the Docker container, which includes the ROS 2, Gazebo and simulated Andino installation.
```commandline
cd robotics_essentials_ros2/docker/
docker compose up 
```

Wait until the container has been successfully launched
<img src="/images/docker_compose_up.png" alt="Andino Simulation Screenshot">


3. Open a new terminal and start an interactive terminal access inside the Docker container
```commandline
docker exec -it robotics_essentials_ros2 bash
```

4. Verify that everything is running correctly, by starting the simulation with
```commandline
ros2 launch andino_gz andino_gz.launch.py
```

**Note:** Gazebo might take a long while to start and open up.


<img src="/images/andino_sim_screenshot.png" alt="Andino Simulation Screenshot">