#!/bin/bash

# Observer Tasks Plugin Build and Install Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

echo "=== Observer Tasks Plugin Build and Install ==="

# Check if mc_rtc is installed
if ! pkg-config --exists mc_rtc; then
    echo "Error: mc_rtc is not found. Please install mc_rtc first."
    exit 1
fi

echo "Found mc_rtc: $(pkg-config --modversion mc_rtc)"

# Create build directory
if [ -d "$BUILD_DIR" ]; then
    echo "Removing existing build directory..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "Building..."
make -j$(nproc)

echo "Installing..."
if [ "$EUID" -eq 0 ]; then
    # Running as root
    make install
else
    # Try to install to user directory first
    if make install 2>/dev/null; then
        echo "Plugin installed to user directory successfully."
    else
        echo "Installation to user directory failed. Trying with sudo..."
        sudo make install
    fi
fi

echo "=== Installation completed successfully! ==="
echo ""
echo "You can now use the following tasks in your mc_rtc controllers:"
echo "  - ObserverbasedImpedance"
echo "  - ObserverbasedAdmittance"
echo ""
echo "See README.md for usage examples."