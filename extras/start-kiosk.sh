#!/bin/bash
# Wait for desktop to load
sleep 5
# Start the Vite dev server
#cd /data/trov_uicopy_queue
#cd /data/trov_ui
cd /data/trov_ui_indoor_outdoor
npm run dev &
# Source ROS2
source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash
# Wait until delivery_runner is publishing available routes
echo "Waiting for /delivery/available_routes publisher..."
until ros2 topic info /delivery/available_routes 2>/dev/null | grep -q "Publisher count: [1-9]"; do
    sleep 2
done
echo "Publisher found! Waiting 3s to settle..."
sleep 3
# Wait for Vite to be ready
until curl -s http://localhost:5173 > /dev/null; do
    sleep 1
done
# Open Firefox

sleep 15

firefox --kiosk http://localhost:5173
