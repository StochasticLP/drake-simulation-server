# Drake Simulation Server Setup Guide

## 1. Cloudflare Tunnel Setup

This secure setup allows you to expose your local server to the internet without port forwarding.

### Prerequisites
- A Cloudflare account
- A domain name managed by Cloudflare (e.g., `lukedphillips.com`)

### Steps
1.  **Install `cloudflared`** on the host machine (your laptop or desktop).
    - **Linux**: `curl -L --output cloudflared.deb https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb && sudo dpkg -i cloudflared.deb`
    - **Windows**: Download from Cloudflare releases.

2.  **Authenticate**:
    ```bash
    cloudflared login
    ```

3.  **Create a Tunnel**:
    ```bash
    cloudflared tunnel create drake-sim
    ```

4.  **Configure the Tunnel**:
    - Go to Cloudflare Dashboard > Zero Trust > Networks > Tunnels.
    - Select `drake-sim`.
    - **Public Hostname**:
        - Subdomain: `drake` (e.g., `drake.lukedphillips.com`)
        - Service: `http://localhost:8080` (This points to our Nginx instance)

5.  **Run the Tunnel**:
    ```bash
    cloudflared tunnel run drake-sim
    ```

## 2. Docker Setup

The application is containerized for easy deployment.

### Prerequisites
- Docker installed.
- **Windows Users**: Ensure Docker Desktop is installed and **WSL 2 integration is enabled** for your Linux distribution (Settings > Resources > WSL Integration).

### Build the Image
Build the Docker image on your development machine:
```bash
docker build -t stochasticlp/drake-simulation-server .
```

### Push to Docker Hub
Push the image to your repository so it can be pulled on other machines:
```bash
docker push stochasticlp/drake-simulation-server:latest
```

### Run the Container
Run the container, mapping port 8080.
**Note**: We run with `--net=host` or ensure port mapping is correct. Since Cloudflared is running on the host and pointing to `localhost:8080`, simple port mapping works.

```bash
docker run -d -p 8080:8080 --name drake-server stochasticlp/drake-simulation-server:latest
```

## 3. Remote Updates

To update the server remotely:

1.  **Push changes** to your git repository and wait for CI/CD or **Build and Push** the new image from your laptop:
    ```bash
    docker build -t stochasticlp/drake-simulation-server .
    docker push stochasticlp/drake-simulation-server:latest
    ```
2.  **SSH** into the host machine (or use RustDesk).
3.  **Update the Container**:
    ```bash
    # Stop existing
    docker stop drake-server
    docker rm drake-server
    
    # Pull latest
    docker pull stochasticlp/drake-simulation-server:latest
    
    # Run new
    docker run -d -p 8080:8080 --name drake-server stochasticlp/drake-simulation-server:latest
    ```

## Troubleshooting
- **Nginx Config**: If simulations fail to load, check Nginx logs inside the container:
  ```bash
  docker exec -it drake-server cat /var/log/nginx/error.log
  ```
