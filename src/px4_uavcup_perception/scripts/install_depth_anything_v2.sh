#!/usr/bin/env bash
set -euo pipefail

workspace_path="${ROS2_UAVCUP_WS:-$HOME/ros2_ws}"
venv_path="$workspace_path/.venv-depth"
repository_path="$workspace_path/third_party/Depth-Anything-V2"
model_directory="$workspace_path/models"
checkpoint_path="$model_directory/depth_anything_v2_metric_hypersim_vits.pth"
checkpoint_url="https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth?download=true"

python3 -m venv --system-site-packages "$venv_path"
source "$venv_path/bin/activate"
python -m pip install --upgrade pip
# ROS Humble's binary OpenCV modules on Ubuntu 22.04 require NumPy 1.x.
python -m pip install \
  'numpy<2' \
  'opencv-python<4.12' \
  filelock \
  fsspec \
  networkx \
  setuptools \
  sympy \
  jinja2 \
  matplotlib
# This workstation has no NVIDIA GPU. Explicitly use the official CPU wheel
# index so pip does not download several gigabytes of unused CUDA libraries.
python -m pip install \
  --index-url https://download.pytorch.org/whl/cpu \
  torch torchvision

mkdir -p "$workspace_path/third_party" "$model_directory"
if [[ ! -d "$repository_path/.git" ]]; then
  git clone --depth 1 \
    https://github.com/DepthAnything/Depth-Anything-V2.git \
    "$repository_path"
fi
if [[ ! -f "$checkpoint_path" ]]; then
  curl --fail --location "$checkpoint_url" --output "$checkpoint_path"
fi

echo "Depth Anything V2 environment is ready."
echo "Activate it before rebuilding/running: source $venv_path/bin/activate"
