import os
import subprocess

# Use standard conf.d so default Nginx setups pick it up (usually)
NGINX_CONF_DIR = "/etc/nginx/conf.d/simulations"

import tempfile

def ensure_conf_dir():
    """Ensure the Nginx configuration directory exists."""
    if not os.path.exists(NGINX_CONF_DIR):
        try:
            os.makedirs(NGINX_CONF_DIR, exist_ok=True)
        except PermissionError:
            print(f"Permission denied creating {NGINX_CONF_DIR}, trying sudo...")
            subprocess.call(['sudo', 'mkdir', '-p', NGINX_CONF_DIR])
            # Attempt to set permissions so we can write to it?
            # Or just rely on sudo for writing later.
            # Let's try to chown it to the current user if we just created it?
            # actually safe to just leave it and use sudo for writes.

def generate_nginx_conf(sid, port):
    """Generate an Nginx configuration file for a specific simulation session."""
    ensure_conf_dir()
    # Use a prefix to identify our files
    conf_path = os.path.join(NGINX_CONF_DIR, f"drake_sim_{sid}.conf")
    conf_content = f"""
    location /meshcat/{sid}/ {{
        proxy_pass http://localhost:{port}/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }}
    """
    
    try:
        with open(conf_path, "w") as f:
            f.write(conf_content)
    except PermissionError:
        print(f"Permission denied writing {conf_path}, trying sudo...")
        # Write to temp file then move
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp.write(conf_content)
            tmp_path = tmp.name
        
        subprocess.call(['sudo', 'mv', tmp_path, conf_path])
        subprocess.call(['sudo', 'chmod', '644', conf_path]) # Ensure readable by nginx
        
    return conf_path

def remove_nginx_conf(sid):
    """Remove the Nginx configuration file for a specific simulation session."""
    conf_path = os.path.join(NGINX_CONF_DIR, f"drake_sim_{sid}.conf")
    if os.path.exists(conf_path):
        try:
            os.remove(conf_path)
        except PermissionError:
             subprocess.call(['sudo', 'rm', conf_path])

def reload_nginx():
    """Reload Nginx to apply changes."""
    try:
        # Check if we are running as root or have sudo access
        if os.geteuid() == 0:
            subprocess.call(['nginx', '-s', 'reload'])
        else:
            subprocess.call(['sudo', 'nginx', '-s', 'reload'])
        print("NGINX reloaded successfully")
    except Exception as e:
        print(f"Error reloading NGINX: {e}")

def clear_all_confs():
    """Clear all simulation configurations (useful on startup)."""
    ensure_conf_dir()
    for filename in os.listdir(NGINX_CONF_DIR):
        file_path = os.path.join(NGINX_CONF_DIR, filename)
        if os.path.isfile(file_path):
            os.remove(file_path)
    reload_nginx()
