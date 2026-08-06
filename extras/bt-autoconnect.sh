#!/bin/bash

DEVICE="A8:E6:E8:79:3C:B1"

# Only connect if not already connected
STATUS=$(bluetoothctl info "$DEVICE" | grep "Connected: yes")

if [ -z "$STATUS" ]; then
    echo "$(date): Device not connected, trying..."
    bluetoothctl connect "$DEVICE"
else
    echo "$(date): Already connected, skipping."
fi
