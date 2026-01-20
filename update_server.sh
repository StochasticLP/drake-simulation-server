#!/bin/bash

# Stop existing container
echo "Stopping existing container..."
docker stop drake-server || true
docker rm drake-server || true

# Pull latest code
echo "Pulling latest code..."
git pull

# Build new image
echo "Building Docker image..."
docker build -t drake-sim-server .

# Run new container
echo "Starting new container..."
docker run -d -p 8080:8080 --name drake-server drake-sim-server

echo "Update complete!"
