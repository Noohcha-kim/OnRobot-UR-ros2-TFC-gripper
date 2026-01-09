#!/bin/bash

# Stop socat for UR Tool Communication
# Usage: ./stop_socat.sh

DEVICE_NAME="/tmp/ttyUR"

echo "============================================"
echo "Stopping Tool Communication (socat)"
echo "============================================"

# Find and kill socat processes
SOCAT_PIDS=$(pgrep -f "socat.*$DEVICE_NAME")

if [ -z "$SOCAT_PIDS" ]; then
    echo "No running socat processes found."
else
    echo "Stopping socat processes..."
    pkill -f "socat.*$DEVICE_NAME"
    sleep 1
    echo "✓ socat stopped"
fi

# Remove device file
if [ -e "$DEVICE_NAME" ]; then
    echo "Removing virtual device..."
    rm -f $DEVICE_NAME
    echo "✓ $DEVICE_NAME removed"
fi

echo ""
echo "Done!"
