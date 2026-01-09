#!/bin/bash

# Start socat for UR Tool Communication (IP configurable)
# Usage: 
#   ./start_socat.sh                    # Use default IP
#   ./start_socat.sh 192.168.1.100      # Custom IP
#   ./start_socat.sh 192.168.1.100 54321 # Custom IP + port

# Default values
DEFAULT_ROBOT_IP="192.168.56.101"
DEFAULT_TCP_PORT="54321"
DEFAULT_DEVICE_NAME="/tmp/ttyUR"

# Use command line arguments or defaults
ROBOT_IP="${1:-$DEFAULT_ROBOT_IP}"
TCP_PORT="${2:-$DEFAULT_TCP_PORT}"
DEVICE_NAME="${3:-$DEFAULT_DEVICE_NAME}"

echo "============================================"
echo "Starting Tool Communication (socat)"
echo "============================================"
echo "Robot IP: $ROBOT_IP"
echo "TCP Port: $TCP_PORT"
echo "Device:   $DEVICE_NAME"
echo "============================================"
echo ""

# Kill existing socat processes
echo "[1/3] Cleaning up existing socat processes..."
pkill -f "socat.*$DEVICE_NAME" 2>/dev/null || true
rm -f $DEVICE_NAME 2>/dev/null || true
sleep 1

# Start socat in background
echo "[2/3] Starting socat..."
socat pty,link=$DEVICE_NAME,raw,ignoreeof,waitslave tcp:$ROBOT_IP:$TCP_PORT &
SOCAT_PID=$!

# Wait for device to be ready
sleep 2

# Verify
if [ -e "$DEVICE_NAME" ]; then
    echo "[3/3] ✓ Tool communication ready!"
    echo ""
    echo "  - Device: $DEVICE_NAME"
    echo "  - PID: $SOCAT_PID"
    echo "  - Robot: $ROBOT_IP:$TCP_PORT"
    echo ""
    echo "Now launch the gripper driver:"
    echo "  ros2 launch onrobot_gripper_driver onrobot_gripper.launch.py"
    echo ""
    echo "To stop:"
    echo "  kill $SOCAT_PID"
    echo "  or"
    echo "  ./stop_socat.sh"
else
    echo "[3/3] ✗ Failed! $DEVICE_NAME not created"
    echo ""
    echo "Check robot connection:"
    echo "  ping $ROBOT_IP"
    exit 1
fi
