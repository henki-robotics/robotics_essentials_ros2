# Docker Cheat Sheet

List of all the used Docker commands during the exercises.

**Note:** "docker compose" -commands need to be run in the same location where the `docker-compose.yaml` file is:

    cd robotics_essentials_ros2/docker/

### Re-build a container
    docker compose up --build

### Run a container
    docker compose up

### Run a container in detached mode
    docker compose up -d

### Open a new terminal inside the Docker container <a id="new-terminal"></a>
    docker exec -it robotics_essentials_ros2 bash

### List all the running Docker containers
    docker ps

### Stop a running container
    docker stop robotics_essentials_ros2

### Remove a running container
    docker rm robotics_essentials_ros2

