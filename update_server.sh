#!/bin/bash

# Stop existing container
echo "Stopping existing container..."
docker stop drake-server || true
docker rm drake-server || true

# Pull latest image
echo "Pulling latest image..."
docker pull stochasticlp/drake-simulation-server:latest

# Run new container
echo "Starting new container..."
docker run -d -p 8080:8080 --name drake-server stochasticlp/drake-simulation-server:latest

echo "Update complete!"
