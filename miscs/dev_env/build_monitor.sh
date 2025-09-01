#!/bin/bash

echo "Building crl_humanoid_monitor package and dependencies..."

# Set the default build type
BUILD_TYPE=RelWithDebInfo

# Build only the monitor package and its dependencies with same flags as main build
colcon build \
        --packages-up-to crl_humanoid_monitor \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
