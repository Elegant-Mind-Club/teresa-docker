#!/bin/bash

CONTAINER_NAME="teresa_dev_env"

# Function to count active shell sessions
count_shells() {
    docker exec $CONTAINER_NAME ps aux | grep -c "bash"
}

# Function to cleanup when last shell exits
cleanup() {
    echo -e "\nShell session ended"
    # Wait a moment for process to fully exit
    sleep 1
    
    # If this was the last shell, stop the container
    if [ "$(count_shells)" -le 1 ]; then
        echo "Last shell session closed, stopping container..."
        docker compose down
    else
        echo "Other shell sessions still active, keeping container running"
    fi
    exit 0
}

# Set up cleanup trap
trap cleanup EXIT

# Check if container is running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container already running, opening new shell..."
    docker exec -it $CONTAINER_NAME bash
else
    echo "Starting container..."
    docker compose up -d
    echo "Opening shell..."
    docker exec -it $CONTAINER_NAME bash
fi