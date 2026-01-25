#!/bin/bash
set -e

echo "Running post-create setup..."

# Update rosdep (user-specific)
rosdep update

# Install Python requirements if they exist
if [ -f "requirements.txt" ]; then
    echo "Installing Python requirements..."
    pip install -r requirements.txt
else
    echo "No requirements.txt found, skipping..."
fi


echo "Installing remaining ROS dependencies..."
source /opt/ros/humble/setup.bash

# Use -r flag to continue on errors (some packages are already installed)
rosdep install --from-paths src --ignore-src -r -y || true

echo "Running colcon build..."
colcon build --symlink-install

# Add workspace overlay to bashrc
if ! grep -q "source /workspaces/LLEAP/install/setup.bash" ~/.bashrc; then
    echo "source /workspaces/LLEAP/install/setup.bash" >> ~/.bashrc
fi

echo "✅ Workspace built successfully!"

echo "✅ Post-create setup complete!"