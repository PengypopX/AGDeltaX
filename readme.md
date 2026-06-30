## 🐳 Docker Development Environment

We use Docker to ensure a consistent development environment across the team. This container comes pre-configured with ROS 2 Jazzy, Intel RealSense drivers, YOLO dependencies (`uv`), and automatically handles the `setup.py` requirements.

### 1. Build the Image
From the root of the repository, build the Docker image (this will take a few minutes the first time):
```bash
docker build -t agdeltax_jazzy:latest .
