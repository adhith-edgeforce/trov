# TROV — Autonomous Ground Vehicle

TROV is a skid-steer UGV developed for autonomous indoor navigation, with outdoor operation planned for a later phase. The platform is built on a NVIDIA Jetson AGX Orin and uses a RoboSense RS-Airy 3D LiDAR as its primary sensor for both mapping and navigation. A Pixhawk running ArduPilot acts as the motor controller — ROS2 sends velocity commands which get translated into MAVLink `ManualControl` messages that the Pixhawk uses to drive the ESCs. The robot is operated through a web interface that talks to the Jetson over a WebSocket connection and an HTTP API.

This document explains how the system is built, how each part works, and how to operate it.

---

## Hardware

| Component | Details |
|---|---|
| Compute | NVIDIA Jetson AGX Orin |
| LiDAR | RoboSense RS-Airy 3D LiDAR — Ethernet at `192.168.2.202` |
| IMU | Hi-Wonder 10-axis — USB at `/dev/ttyUSB0` |
| Flight Controller | Pixhawk (ArduPilot) — USB at `/dev/ttyACM0`, baud 57600 |
| Driving Camera | Front-facing camera, streamed through MediaMTX |
| Surveillance Camera | Secondary camera, streamed through MediaMTX |
| PTZ Camera | IMOU cloud-connected PTZ, controlled via IMOU API |
| Depth Camera | Intel RealSense — used for traversability (in progress) |
| Battery Monitor | ADS1115 ADC over I2C bus 7, address `0x48` |
| Touchscreen | XPT2046 resistive touchscreen — SPI, 1024×600 display |
| GPS | HRTK Mosaic — connected to Pixhawk, data comes through MAVROS; outdoor navigation planned |

---

## Software Stack

| Layer | What it uses |
|---|---|
| ROS2 | Humble Hawksbill |
| LiDAR driver | `rslidar_sdk` |
| IMU driver | `wit_ros2_imu` |
| Flight controller bridge | MAVROS — serial to Pixhawk |
| Odometry | RTAB-Map `icp_odometry` — LiDAR scans + IMU |
| SLAM | `slam_toolbox` (async mode) |
| Localization | `nav2_amcl` + `nav2_map_server` |
| Navigation | Nav2 — MPPI controller, SMAC planner, behavior trees, collision monitor |
| Drive bridge | Custom node in `cpp_pubsub` — translates `/cmd_vel` to MAVLink |
| Web bridge | ROSBridge WebSocket on port 9090 |
| HTTP API | Flask + Waitress on port 5000 |
| Video streaming | MediaMTX — RTSP port 8554, WebRTC port 8889 |
| PTZ control | IMOU cloud API over `aiohttp` |

---

## Repository Structure

```
trov_ws/
├── src/
│   ├── trov/                         # Main robot package
│   │   ├── behavior_trees/           # Custom Nav2 behavior trees
│   │   ├── config/
│   │   │   ├── indoor/               # AMCL, ICP odometry, Nav2 params
│   │   │   └── outdoor/              # EKF + GPS params (planned)
│   │   ├── description/              # Robot URDF (trov.urdf.xacro)
│   │   ├── launch/
│   │   │   ├── indoor/               # slam, localization, navigation, icp_odometry
│   │   │   └── outdoor/              # planned
│   │   ├── maps/                     # Saved PGM maps
│   │   ├── routes/                   # Waypoint YAML files
│   │   ├── scripts/                  # Python nodes — waypoint follower, recorder, delivery runner
│   │   └── src/                      # C++ nodes — battery, collision beacon, headlight, beeper, etc.
│   │   ├── extras/                   # Shell scripts, systemd services, config files
│   │   │   ├── trov_launch_files.sh          # Master indoor launch script
│   │   │   ├── trov_outdoor_launch_files.sh  # Outdoor stack launcher (in progress)
│   │   │   ├── restart_*.sh                  # Individual stack restart scripts
│   │   │   ├── start/stop/save_map*.sh       # Mapping scripts
│   │   │   ├── trov.service                  # systemd service for full stack
│   │   │   ├── trov-api.service              # systemd service for API server
│   │   │   ├── xpt2046-touch.service         # systemd service for touchscreen driver
│   │   │   ├── xpt2046_input.py              # XPT2046 SPI touchscreen driver
│   │   │   └── mediamtx.yml                  # MediaMTX stream configuration
│   ├── cpp_pubsub/                   # Drive bridge (cmd_vel → MAVROS)
│   ├── segformer_traversability_autoware/  # Traversability segmentation node
│   └── [third-party packages]        # See dependencies section
├── trov_api.py                       # Flask API server for the web UI
├── trov_launch_files.sh              # Master launch script
├── start_mapping.sh                  # Start SLAM session
├── stop_mapping.sh                   # Stop SLAM, save map to /tmp
├── save_map.sh <name>                # Move saved map into maps directory
├── restart_localization.sh <map>     # Hot-swap the active map
├── restart_navigation.sh             # Restart Nav2 only
└── restart_odometry.sh               # Restart ICP odometry only
```

---

## How the System Works

### The motion pipeline

Everything starts with Nav2's planner computing a path to a goal. The path goes to the MPPI local controller, which generates velocity commands on `/cmd_vel`. Those pass through a velocity smoother that limits acceleration, producing `/cmd_vel_smoothed`. The drive bridge node reads from `/cmd_vel_smoothed` and converts each message into a MAVLink `ManualControl` message, which MAVROS forwards to the Pixhawk over serial. The Pixhawk maps those values to PWM signals on its ESC outputs, spinning the wheels.

```
Nav2 planner
    ↓  /cmd_vel
Velocity smoother
    ↓  /cmd_vel_smoothed
trov_drive_bridge  (cpp_pubsub)
    ↓  /mavros/manual_control/send
MAVROS
    ↓  MAVLink serial  (/dev/ttyACM0 at 57600 baud)
Pixhawk  (ArduPilot)
    ↓  PWM  (1100–1900 µs)
Motor ESCs → wheels
```

The drive bridge is not a simple linear mapping. Because TROV is a skid-steer robot, blending forward motion and rotation simultaneously causes unpredictable physical behaviour. The bridge implements a five-state machine — **STRAIGHT, SLOWING, STOPPING, PIVOT, STALLED** — that strictly separates forward motion from turning.

When the robot is moving forward and a strong angular command arrives, the bridge forces a complete stop and holds zero output for two seconds before entering the pivot state. The angular input is classified by a threshold — weak angular values (nav stack drift, small residuals) are silently ignored during straight motion so the robot continues forward. Only values above the threshold trigger the stop-then-pivot sequence.

The bridge also watches odometry. If the robot is commanded to move but its position from `/odom` has not changed by more than 2 cm over a one-second window, a stall is declared. All output is zeroed for one second before motion resumes. This prevents the robot from burning out motors or staying stuck in a feedback loop when it physically cannot move.

MAVLink maps its `ManualControl` range to PWM linearly — MAVLink 0 → 1500 µs (neutral/stopped), MAVLink 1000 → 1900 µs (full forward).

### Odometry

The robot estimates its own motion using ICP odometry from RTAB-Map (`rtabmap_odom/icp_odometry`). This node takes the raw 3D point cloud from `/points` and the IMU data from `/imu/data`, and produces an odometry estimate on `/odom` along with the `odom → base_link` transform. The IMU is used as a motion guess between consecutive scan registrations — it does not fuse into the final position estimate directly, but helps ICP converge faster and more reliably, particularly during rotation when scan-to-scan matching is harder. The node is configured with `subscribe_scan_cloud: true` because the RS-Airy publishes a full 3D point cloud rather than a 2D laser scan.

### Localization

Once a map exists, the robot localizes itself within it using AMCL — a particle filter that maintains a probability distribution over where the robot could be, and updates it each time a new laser scan arrives. AMCL does not work with 3D point clouds directly. The cloud is first converted into a 2D laser scan by `pointcloud_to_laserscan`, which slices the cloud at a configurable height band. The current configuration uses 0.3m to 0.8m above the floor. The lower bound discards floor reflections and ground returns. The upper bound cuts off at roughly waist height, which means people walking through the environment mostly disappear from the scan. Only walls and fixed structures in that height band contribute to localization, making AMCL significantly more stable in occupied environments.

The map itself is a `.pgm` greyscale image paired with a `.yaml` file describing its resolution and origin. Maps are stored in `src/trov/maps/` and deployed to the install directory during `colcon build`. The localization launch file accepts a `map_name` argument, which it uses to dynamically construct the path to the corresponding YAML file — so switching maps requires only restarting the launch file with a different argument, not modifying any code.

### Navigation

Nav2 manages the full autonomy stack. The lifecycle manager is responsible for activating all Nav2 nodes in the correct sequence — controller server, planner server, behavior server, waypoint follower, velocity smoother, collision monitor, and BT navigator. On the Jetson, all these nodes take considerable time to initialize and register their lifecycle services. The lifecycle manager is therefore delayed by 30 seconds using a `TimerAction` in the launch file, with a `service_availability_timeout` of 60 seconds per node. Without these delays, the lifecycle manager would attempt to configure nodes before they have finished starting, causing the entire stack to fail.

The behavior trees in `src/trov/behavior_trees/` define how the robot responds when navigation fails or encounters obstacles. Four variants exist. The standard navigate-to-pose and navigate-through-poses trees include replanning and recovery behaviors — when the robot gets stuck, it spins in place, backs up, or re-plans from scratch before giving up. The "no clear" variants skip costmap clearing between navigation goals. These are used during delivery missions where clearing the costmap between waypoints would erase obstacle information that the robot still needs.

### Mapping

When building a map of a new space, `slam_toolbox` in async mode is used. The same `pointcloud_to_laserscan` conversion applies here, though with slightly different parameters — a wider field of view and lower height bounds compared to the localization configuration, so the SLAM map captures the environment fully. During a mapping session, localization and navigation are stopped completely so they do not interfere with the SLAM process.

Once the space has been explored by driving the robot around manually, the map is saved using `nav2_map_server`'s `map_saver_cli`, which writes a `.pgm` and `.yaml` to `/tmp/trov_map_temp`. The `save_map.sh` script then moves these into the maps directory under a user-provided name and patches the image path inside the YAML file — `map_saver_cli` writes the absolute temp path, which would be wrong after the file is moved.

### Peripheral nodes

Several C++ nodes handle physical hardware on the robot.

**Battery monitor** reads the ADS1115 ADC on I2C bus 7 at address 0x48 using single-shot conversion mode. Each reading writes a configuration to the ADC's config register to trigger a conversion, then polls the OS bit in the register until the conversion is complete (up to 20 attempts at 8ms intervals). The 16-bit signed result is converted to a voltage using the configured PGA range (±2.048V), then linearly mapped to a percentage using calibrated min/max voltage constants. The node publishes `/trov/battery/percent` as a Float32 and `/trov/battery/status` as a String (OK / LOW / CRITICAL) at 1Hz. The web UI reads both topics to display a battery indicator.

**Collision beacon** subscribes to `/points` and checks every incoming point cloud for points inside a cylindrical region of interest centred on the LiDAR origin — 30cm radius, 0.1m to 2.5m height. The height bounds reject floor returns below and open space above. If 7 or more points fall inside this cylinder, a Jetson GPIO line (gpiochip1, line 9, physical pin 16 on the AGX Orin) is driven HIGH, activating a physical warning light on the robot body. The cylinder is also published as a coloured marker on `/collision_roi_marker` for RViz visualization — green when clear, red when triggered. This functions as a close-proximity alarm, not as a navigation obstacle layer.

**Sensor health monitor** watches three ROS2 topics with configurable timeouts — `/points` at 1.0s, `/imu/data` at 0.5s, and `/mavros/global_position/global` at 1.0s. If a topic goes silent beyond its timeout, the corresponding flag goes false. Four Bool topics are published at 2Hz: `/sensor_health/lidar`, `/sensor_health/imu`, `/sensor_health/gps`, and `/sensor_health/all`. The web UI subscribes to these to display live sensor status indicators. A sensor that has never published anything is considered unhealthy immediately rather than waiting for a timeout.

**Headlight controller** listens to `/trov/headlight` (Bool). Publishing `true` sends a MAVLink `MAV_CMD_DO_SET_SERVO` command via MAVROS targeting servo output channel 9 at 2000µs PWM (on). Publishing `false` sends 1000µs (off). The headlight is physically wired to a servo output on the Pixhawk. This approach avoids needing any additional hardware between the Jetson and the light — the Pixhawk servo rail handles it directly.

**Floodlight** controls Jetson GPIO board pin 32 using the `Jetson.GPIO` Python library. It subscribes to `/gpio_pin32_control` (Bool) and drives the pin HIGH or LOW accordingly. GPIO is initialized LOW on startup and cleaned up on shutdown.

**Beeper** subscribes to `/mavros/manual_control/send` and plays an audio beep whenever the robot receives a non-zero drive command. It uses `sox` (`play` command) to play a configurable MP3 file at a configurable volume. Playback runs in a detached background thread so it never blocks the ROS2 callback. The beep path and volume are ROS2 parameters (`beep_sound_path`, `beep_volume`) so they can be changed without recompiling.

**XPT2046 touchscreen driver** (`xpt2046_input.py`) is a standalone Python script that runs as the `xpt2046-touch.service` systemd service. It reads touch coordinates from an XPT2046 resistive touchscreen over SPI and injects them into the Linux input system via `uinput`, making the touchscreen appear as a standard input device to the OS. The calibration uses a degree-4 bivariate polynomial fit to map raw ADC coordinates to screen pixels on a 1024×600 display. Two stages of filtering are applied: a hardware median filter (8 raw readings per sample, median taken) to remove ADC noise, followed by a jump filter that rejects sudden large position changes unless confirmed by a second consecutive reading, and an exponential moving average for smooth cursor movement.

### The API server

The API server (`trov_api.py`) runs as `trov-api.service` on port 5000 using Flask served by Waitress with 8 worker threads. It is what the web UI calls to perform any operation that requires a shell command or system interaction.

The robot control endpoints let the UI list available maps, check which key processes are alive by running `pgrep` on process name keywords, and start/stop/restart `trov.service` via `systemctl`. Localization, navigation, and odometry can each be hot-restarted individually — the API validates that the requested map exists before calling the restart script, and returns a 404 if it does not.

The mapping endpoints let the UI start a SLAM session, stop it and save the resulting map in a single call (with name collision checking before any file operations begin), and poll mapping status to check whether SLAM is running and whether a temp map is waiting to be committed.

The PTZ camera is controlled through the IMOU cloud API. Move operations take an operation string and a duration in milliseconds — the API sends the move command, waits for the specified duration, then sends a stop command automatically. The PTZ camera is identified by device ID and authenticated with app credentials stored in the server.

Two MJPEG camera streams are also served from this process — a driving camera and a surveillance camera, both sourced from MediaMTX at `rtsp://localhost:8554`. Each stream runs a background thread that captures frames from its RTSP source using OpenCV, encodes each frame as JPEG, and serves it as a multipart HTTP response. The web UI displays these as live video in the driving and monitoring views.

### The web bridge

ROSBridge WebSocket on port 9090 gives the web UI direct access to the ROS2 topic graph. The UI subscribes to topics like robot pose, Nav2 navigation status, battery level, sensor health, and costmap data, and publishes navigation goals and commands back — all in real time, without going through the API server. The API server handles operations that need shell access; ROSBridge handles everything that is purely ROS2 communication.

### Traversability segmentation

A separate workspace contains a SegFormer-based semantic segmentation node that runs on the Jetson's GPU. It subscribes to `/camera/camera/color/image_raw` from the RealSense and passes each frame through a SegFormer-B5 model fine-tuned on the ADE20K dataset. Every pixel in the output is classified into one of three traversability categories — safe (floor, road, path, ground surfaces), risky (stairs, ramps, vegetation, loose furniture), or blocked (walls, doors, people, vehicles, water, and anything else the robot cannot pass through). Two image topics are published: `/fusion_segmentation/traversability` is the three-colour traversability map, and `/fusion_segmentation/semantic` is a full colour-coded semantic map showing all detected classes. Preprocessing applies CLAHE contrast enhancement in LAB colour space to improve robustness in low-light environments. The model runs in FP16 with CUDA autocast. Inference timing is tracked per frame and logged every 10 frames so performance can be monitored on the Jetson GPU.

The traversability output is not yet connected to Nav2. The planned integration is a custom costmap layer that ingests the traversability image and marks risky and blocked pixels as obstacles in the costmap. This would allow the planner to route around camera-detected hazards in addition to the LiDAR obstacle layer — particularly useful for detecting low obstacles, transparent surfaces, and terrain changes that the LiDAR misses.

### Outdoor navigation (in progress)

Outdoor navigation is currently being developed and is not yet fully operational. The architecture uses a dual EKF setup from `robot_localization` — two EKF nodes running simultaneously, one for local odometry and one for GPS-fused global positioning.

EKF1 (`ekf_filter_node_odom`) handles the `odom → base_link` transform. It fuses ICP LiDAR odometry position and heading from `/odom` with IMU yaw and yaw rate from `/imu/data`. Using ICP absolute position rather than wheel velocity integration gives significantly better odometry in outdoor environments.

EKF2 (`ekf_filter_node_map`) handles the `map → odom` transform. It fuses GPS position from `/odometry/gps` (produced by `navsat_transform_node` from the raw MAVROS GPS fix) with IMU yaw. The `navsat_transform` node is configured to subscribe to EKF2's output (`odometry/filtered/global`) rather than EKF1's — this ensures the GPS odometry is published in the map frame, avoiding a circular TF dependency that would cause position explosion.

GPS data comes through MAVROS from the HRTK Mosaic, which is connected directly to the Pixhawk. The magnetic declination is set for the Adibatla area (−0.0089 radians).

The outdoor stack is launched separately from the indoor stack using `trov_outdoor_launch_files.sh`, which starts the LiDAR driver, IMU driver, MAVROS, drive bridge, and the `hrtk_odom` node. The remaining components — ICP odometry, dual EKF, and outdoor Nav2 — must currently be launched manually in separate terminals:

```bash
# Terminal 1 — ICP odometry
ros2 launch trov icp_odometry_outdoor.launch.py

# Terminal 2 — Dual EKF + navsat transform
ros2 launch trov dual_ekf_navsat.launch.py

# Terminal 3 — Nav2 outdoor navigation
ros2 launch trov navigation_outdoor.launch.py
```

Integration of these into a single automated outdoor launch script is planned.

---

## Full Startup

Everything is managed by `trov_launch_files.sh`. On boot, `trov.service` runs it automatically. To start manually:

```bash
cd /data/trov_ws

# Start with the last active map (saved automatically in ~/.trov_last_map)
./trov_launch_files.sh

# Or specify a map explicitly — this also saves it as the new default
./trov_launch_files.sh adibatla_indoor_box
```

The script performs hardware checks first — it pings the LiDAR, confirms the IMU and Pixhawk ports exist, and verifies the selected map file is present. If any of these fail the script exits before launching anything.

Nodes launch in this order:

```
 1.  RSLidar SDK                — LiDAR Ethernet driver
 2.  WIT IMU driver             — serial IMU on /dev/ttyUSB0
 3.  RealSense camera
 4.  MAVROS                     — Pixhawk serial bridge (waits 15s)
 5.  Drive bridge               — ros2 run cpp_pubsub drive
 6.  Sensor health monitor
 7.  Collision beacon
 8.  Battery monitor
 9.  Floodlight
10.  Headlight controller
11.  ROSBridge WebSocket        — port 9090
12.  MediaMTX                   — RTSP/WebRTC video
13.  ICP odometry               — waits for /points, /imu/data, TF
14.  Localization               — map_server + AMCL (waits for /odom)
15.  Navigation                 — Nav2 (waits for /map, polls until active)
16.  Waypoint follower
17.  Waypoint recorder
18.  Delivery runner            — waits for Nav2 to confirm active
```

The selected map is saved to `~/.trov_last_map` every time localization starts. On the next boot, this file is read automatically. If the saved map no longer exists, the script falls back to `indoor3`.

Once running, the terminal stays interactive:

```
r  →  Restart localization (prompts for a new map name)
n  →  Restart navigation
s  →  Show all node PIDs and alive/dead status
m  →  Check if /map is publishing
q  →  Quit and shut everything down cleanly
```

---

## Mapping — Building a New Map

```bash
# Start SLAM — kills localization and navigation first
./start_mapping.sh

# Drive the robot through the environment
# Use the web UI or keyboard teleop

# Save the map
./stop_mapping.sh            # writes /tmp/trov_map_temp.pgm + .yaml
./save_map.sh my_map_name    # moves into maps dir, patches YAML path

# Switch to the new map
./restart_localization.sh my_map_name
```

The same flow is available from the web UI via `POST /api/mapping/start` and `POST /api/mapping/stop_and_save`.

---

## Hot-Restarting Individual Stacks

```bash
# Swap to a different map (also restarts waypoint + delivery nodes)
./restart_localization.sh indoor3

# Restart Nav2 only (e.g. after changing nav2_params_indoor.yaml)
./restart_navigation.sh

# Restart ICP odometry (e.g. after drift or a crash)
./restart_odometry.sh
```

---

## Service Logs

```bash
sudo systemctl status trov
sudo systemctl status trov-api
sudo journalctl -u trov -f
sudo journalctl -u trov-api -f
```

---

## Useful ROS2 Commands

```bash
source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

# Sensor health and battery
ros2 topic echo /sensor_health/all
ros2 topic echo /trov/battery/percent

# Nav2 lifecycle state
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server

# Manual teleoperation (bypasses Nav2)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_smoothed

# Headlight and floodlight
ros2 topic pub /trov/headlight std_msgs/msg/Bool "data: true" --once
ros2 topic pub /gpio_pin32_control std_msgs/msg/Bool "data: true" --once

# Available maps
ls /data/trov_ws/install/trov/share/trov/maps/
```

---

## Build

```bash
cd /data/trov_ws

# Fast build — only our packages
colcon build --symlink-install --packages-select trov cpp_pubsub
source install/setup.bash

# Full build including all third-party packages
colcon build --symlink-install
```

---

## Web Interface

| Address | Purpose |
|---|---|
| `ws://<robot_ip>:9090` | ROSBridge WebSocket — live ROS2 topics |
| `http://<robot_ip>:5000` | HTTP API — maps, stack control, PTZ, mapping |
| `rtsp://<robot_ip>:8554` | RTSP video streams |
| `http://<robot_ip>:8889` | WebRTC video streams |

### API reference

| Method | Endpoint | Description |
|---|---|---|
| GET | `/api/maps` | List available map names |
| GET | `/api/status` | Check which processes are alive |
| GET | `/api/stack/status` | Check if `trov.service` is active |
| POST | `/api/stack/start` | Start the robot stack |
| POST | `/api/stack/stop` | Stop the robot stack |
| POST | `/api/stack/restart` | Restart the robot stack |
| POST | `/api/localization/restart` | Restart localization `{ "map_name": "..." }` |
| POST | `/api/navigation/restart` | Restart Nav2 |
| POST | `/api/odometry/restart` | Restart ICP odometry |
| POST | `/api/mapping/start` | Start SLAM mapping session |
| POST | `/api/mapping/stop_and_save` | Stop SLAM and save map `{ "map_name": "..." }` |
| GET | `/api/mapping/status` | Is SLAM running? Is temp map ready? |
| POST | `/api/ptz/move` | Move PTZ camera `{ "operation": "LEFT", "duration": 500 }` |
| POST | `/api/ptz/stop` | Stop PTZ movement |
| GET | `/api/ptz/info` | PTZ camera info |
| GET | `/api/health` | API server health |

---

## Third-Party Packages

| Package | Purpose |
|---|---|
| `rslidar_sdk` | RoboSense RS-Airy LiDAR driver |
| `wit_ros2_imu` | Hi-Wonder IMU driver |
| `robot_localization` | EKF — present for future outdoor GPS fusion via MAVROS |

Nav2 (`nav2_amcl`, `nav2_map_server`, `nav2_controller`, etc.) and `slam_toolbox` are installed as system packages via `apt`, not built from source. MAVROS is also a system package — it handles GPS data from the HRTK Mosaic, which connects directly to the Pixhawk rather than to the Jetson.

Traversability segmentation lives in a separate workspace — see the segformer repository (link TBD).

---

## Project Status

| Feature | Status |
|---|---|
| Indoor SLAM | ✅ Working |
| Indoor localization (AMCL) | ✅ Working |
| Indoor Nav2 autonomous navigation | ✅ Working |
| Waypoint recording and following | ✅ Working |
| Drive bridge (MAVROS / Pixhawk) | ✅ Working |
| Battery monitoring (I2C ADC) | ✅ Working |
| Collision beacon (LiDAR + GPIO) | ✅ Working |
| Sensor health monitoring | ✅ Working |
| Headlight + floodlight control | ✅ Working |
| Beeper (drive command audio feedback) | ✅ Working |
| XPT2046 touchscreen driver | ✅ Working |
| Web UI + ROSBridge + API server | ✅ Working |
| PTZ camera control (IMOU) | ✅ Working |
| Dual camera streaming (MediaMTX) | ✅ Working |
| Traversability segmentation (SegFormer) | 🔄 Running — Nav2 costmap integration planned |
| Outdoor navigation (dual EKF + GPS) | 🔄 In progress — manual launch only |