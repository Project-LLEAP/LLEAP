#!/bin/bash
# Helper script to launch Gazebo Fortress with WSL2 workaround for Ogre2 rendering

export IGN_GAZEBO_RESOURCE_PATH=~/.gz/sim/8
export MESA_GL_VERSION_OVERRIDE=4.5
export MESA_GLSL_VERSION_OVERRIDE=450

ros2 launch exo_gazebo gazebo_fortress.launch.py "$@"
