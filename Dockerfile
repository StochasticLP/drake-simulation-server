FROM ubuntu:22.04

# Install basics and Nginx
# We explicitly install nginx here.
RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-venv \
    nginx \
    wget curl git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy requirements first to leverage Docker cache
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY . .

# Copy Nginx configuration
# We overwrite the default config
COPY nginx.conf /etc/nginx/nginx.conf

# setup start script
RUN chmod +x start.sh

# Expose port 8080 (Nginx)
EXPOSE 8080

# Start the application
CMD ["./start.sh"]
