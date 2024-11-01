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