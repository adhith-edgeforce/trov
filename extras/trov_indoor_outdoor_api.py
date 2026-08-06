import os
import re
import subprocess
import asyncio
import traceback

import aiohttp
from flask import Flask, jsonify, request
from flask_cors import CORS
from imouapi.api import ImouAPIClient
from imouapi.exceptions import ImouException
from waitress import serve

# ─── PTZ config ───────────────────────────────────────────────────────────────
APP_ID     = "lc09e486a9ebaa4239"
APP_SECRET = "97a77a752d5248cbbacd25de36caef"
DEVICE_ID  = "A889AAMPGVAFE14"

VALID_OPERATIONS = {
    "UP", "DOWN", "LEFT", "RIGHT",
    "UPPER_LEFT", "UPPER_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT",
    "ZOOM_IN", "ZOOM_OUT", "STOP"
}

# ─── Robot API config ─────────────────────────────────────────────────────────
SCRIPTS_DIR = "/data/trov_ws"
ROS_ENV     = {**os.environ, "HOME": "/home/nvidia"}

# ══════════════════════════════════════════════════════════════════════════════
# ENVIRONMENT (indoor / outdoor) CONFIG
# ──────────────────────────────────────────────────────────────────────────────
# Everything that differs between indoor and outdoor lives HERE and nowhere else.
# Every endpoint below reads get_env_config() and works off this dict, so adding
# a third mode later = add one more block, no endpoint changes.
#
# ┌─────────────────────────────────────────────────────────────────────────────┐
# │  >>> FILL IN THE THREE  # TODO  VALUES IN THE "outdoor" BLOCK BEFORE USE  <<< │
# └─────────────────────────────────────────────────────────────────────────────┘
# ══════════════════════════════════════════════════════════════════════════════

# Persisted active environment — survives API restarts, same idea as .trov_last_map
ENV_STATE_FILE = "/home/nvidia/.trov_environment"
DEFAULT_ENV    = "indoor"

ENV_CONFIG = {
    "indoor": {
        "service":       "trov.service",
        "maps_dir":      "/data/trov_ws/install/trov/share/trov/maps",
        "map_ext":       ".yaml",                # 2D occupancy grid (map_server)
        "temp_map_path": "/tmp/trov_map_temp",
        "temp_map_exts": [".yaml", ".pgm"],      # both must exist = ready to save
        "scripts": {
            "localization": "restart_localization.sh",
            "navigation":   "restart_navigation.sh",
            "odometry":     "restart_odometry.sh",
            "mapping_start":"start_mapping.sh",
            "mapping_stop": "stop_mapping.sh",
            "mapping_save": "save_map.sh",
        },
        "processes": {
            "localization": "map_server",
            "navigation":   "bt_navigator",
            "odometry":     "icp_odometry",
            "rosbridge":    "rosbridge_websocket",
            "mediamtx":     "mediamtx",
            "mapping":      "slam_toolbox",
        },
    },
    "outdoor": {
        "service":       "trov_outdoor.service",
        # Outdoor UI list = converted 2D grids (.yaml) in grid_maps/.
        # Localization uses the matching .pcd in pcd_maps/ — the restart script
        # maps <name> back to pcd_maps/<name>.pcd itself (same filename stem).
        "maps_dir":      "/data/trov_ws/pcd_maps/grid_maps",
        "map_ext":       ".yaml",                # converted grid (display + maps list)
        # TODO ── set the temp path your outdoor save script writes to ──
        "temp_map_path": "/data/pcd_map_temp",
        "temp_map_exts": [".pcd"],               # ready-to-save check for outdoor
        "scripts": {
            "localization": "restart_localization_outdoor.sh",
            "navigation":   "restart_navigation_outdoor.sh",
            "odometry":     "restart_odometry_outdoor.sh",
            "mapping_start":"start_mapping_outdoor.sh",
            "mapping_stop": "stop_mapping_outdoor.sh",
            "mapping_save": "save_map_outdoor.sh",
        },
        "processes": {
            "localization": "outdoor_lidar_localization",
            "navigation":   "bt_navigator",
            "odometry":     "icp_odometry",
            "rosbridge":    "rosbridge_websocket",
            "mediamtx":     "mediamtx",
            # TODO ── set the pgrep keyword for your outdoor SLAM (lidarslam) node ──
            "mapping":      "scanmatcher_node",
        },
    },
}


def get_environment():
    """Read the persisted active environment; fall back to DEFAULT_ENV."""
    try:
        with open(ENV_STATE_FILE) as f:
            env = f.read().strip()
        if env in ENV_CONFIG:
            return env
    except OSError:
        pass
    return DEFAULT_ENV


def set_environment(env):
    """Persist the active environment so every request resolves to it."""
    with open(ENV_STATE_FILE, "w") as f:
        f.write(env)


def get_env_config():
    """Config block for whichever environment is currently active."""
    return ENV_CONFIG[get_environment()]


# ─── Flask app ────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.url_map.strict_slashes = False
CORS(app, resources={r"/*": {"origins": "*"}})


# ══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _cors_preflight():
    resp = app.make_response('')
    resp.status_code = 200
    resp.headers['Access-Control-Allow-Origin']  = '*'
    resp.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    resp.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
    return resp


def run_script(script_name: str, args: list = []):
    script_path = os.path.join(SCRIPTS_DIR, script_name)
    if not os.path.isfile(script_path):
        return None, f"Script not found: {script_path}"
    if not os.access(script_path, os.X_OK):
        return None, f"Script not executable: {script_path}"
    proc = subprocess.Popen(["/bin/bash", script_path] + args, env=ROS_ENV)
    return proc.pid, None


def run_systemctl(action: str, service: str):
    result = subprocess.run(
        ["sudo", "systemctl", action, service],
        capture_output=True, text=True
    )
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def is_service_active(service: str) -> str:
    result = subprocess.run(
        ["systemctl", "is-active", service],
        capture_output=True, text=True
    )
    return result.stdout.strip()


# ══════════════════════════════════════════════════════════════════════════════
# PTZ helpers
# ══════════════════════════════════════════════════════════════════════════════

def run_async(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


async def _ptz_move(operation, duration):
    async with aiohttp.ClientSession() as session:
        client = ImouAPIClient(APP_ID, APP_SECRET, session)
        await client.async_connect()
        await client.async_api_controlMovePTZ(DEVICE_ID, operation, duration)


async def _ptz_stop():
    async with aiohttp.ClientSession() as session:
        client = ImouAPIClient(APP_ID, APP_SECRET, session)
        await client.async_connect()
        await client.async_api_controlMovePTZ(DEVICE_ID, "STOP", 0)


async def _ptz_info():
    async with aiohttp.ClientSession() as session:
        client = ImouAPIClient(APP_ID, APP_SECRET, session)
        await client.async_connect()
        return await client.async_api_devicePTZInfo(DEVICE_ID)


# ══════════════════════════════════════════════════════════════════════════════
# PTZ routes
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/ptz/move', methods=['POST', 'OPTIONS'])
def ptz_move():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    data      = request.get_json(force=True)
    operation = str(data.get('operation', '')).upper()
    duration  = int(data.get('duration', 500))
    if operation not in VALID_OPERATIONS:
        return jsonify({'success': False, 'error': f"Invalid operation '{operation}'"}), 400
    try:
        run_async(_ptz_move(operation, duration))
        return jsonify({'success': True, 'operation': operation, 'duration': duration})
    except ImouException as e:
        return jsonify({'success': False, 'error': str(e)}), 500
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': repr(e)}), 500


@app.route('/api/ptz/stop', methods=['POST', 'OPTIONS'])
def ptz_stop():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    try:
        run_async(_ptz_stop())
        return jsonify({'success': True, 'operation': 'STOP'})
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/ptz/info', methods=['GET'])
def ptz_info():
    try:
        info = run_async(_ptz_info())
        return jsonify({'success': True, 'data': info})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# ══════════════════════════════════════════════════════════════════════════════
# ENVIRONMENT routes  (indoor / outdoor)
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/environment', methods=['GET'])
def environment_status():
    """What mode is the robot in, and is that mode's service running?"""
    env = get_environment()
    cfg = ENV_CONFIG[env]
    return jsonify({
        'environment':    env,
        'service':        cfg['service'],
        'service_status': is_service_active(cfg['service']),
        'available':      list(ENV_CONFIG.keys()),
    })


@app.route('/api/environment/switch', methods=['POST', 'OPTIONS'])
def environment_switch():
    """
    Switch modes atomically: stop every OTHER environment's service (mutual
    exclusivity — the robot is either indoor or outdoor, never both), persist
    the choice, then start the target service.

    If you would rather switch the mode WITHOUT auto-starting the stack (let the
    operator press Start manually), delete the run_systemctl('start', ...) block.
    """
    if request.method == 'OPTIONS':
        return _cors_preflight()

    data   = request.get_json(force=True)
    target = str(data.get('environment', '')).strip().lower()

    if target not in ENV_CONFIG:
        return jsonify({
            'error': f"Invalid environment '{target}'. Valid: {list(ENV_CONFIG.keys())}"
        }), 400

    # ── Stop the other mode(s) so two stacks never run at once ──
    for env_name, cfg in ENV_CONFIG.items():
        if env_name != target:
            run_systemctl('stop', cfg['service'])

    # ── Persist BEFORE starting so all other endpoints resolve correctly ──
    set_environment(target)

    # ── Start the target stack ──
    target_service = ENV_CONFIG[target]['service']
    code, _, err = run_systemctl('start', target_service)
    if code != 0:
        return jsonify({
            'error': f"Environment set to '{target}' but failed to start "
                     f"{target_service}: {err}",
            'environment': target,
        }), 500

    return jsonify({
        'status':      'ok',
        'environment': target,
        'service':     target_service,
        'message':     f"Switched to {target}. {target_service} starting; "
                       f"other stack(s) stopped.",
    })


# ══════════════════════════════════════════════════════════════════════════════
# ROBOT API routes  (all environment-aware)
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/maps', methods=['GET'])
def get_maps():
    """List maps for the ACTIVE environment (.yaml indoor / .pcd outdoor)."""
    cfg      = get_env_config()
    maps_dir = cfg['maps_dir']
    ext      = cfg['map_ext']

    if not os.path.isdir(maps_dir):
        return jsonify({'error': f'Maps folder not found: {maps_dir}'}), 500

    maps = sorted([
        f[:-len(ext)]
        for f in os.listdir(maps_dir)
        if f.endswith(ext)
    ])
    if not maps:
        return jsonify({'error': 'No maps found', 'environment': get_environment()}), 404

    return jsonify({'maps': maps, 'environment': get_environment()})


@app.route('/api/status', methods=['GET'])
def get_status():
    """Process health for the ACTIVE environment's node set."""
    processes = get_env_config()['processes']
    status = {}
    for name, keyword in processes.items():
        result = subprocess.run(['pgrep', '-f', keyword], capture_output=True)
        status[name] = 'running' if result.returncode == 0 else 'dead'
    return jsonify({'status': status, 'environment': get_environment()})


@app.route('/api/stack/status', methods=['GET'])
def stack_status():
    cfg = get_env_config()
    # Key kept as 'trov_service' for frontend compatibility (SystemControl.jsx).
    return jsonify({
        'trov_service': is_service_active(cfg['service']),
        'service':      cfg['service'],
        'environment':  get_environment(),
    })


@app.route('/api/stack/start', methods=['POST', 'OPTIONS'])
def stack_start():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    cfg     = get_env_config()
    service = cfg['service']
    if is_service_active(service) == 'active':
        return jsonify({'status': 'already_running',
                        'message': f'{service} is already active'})
    code, _, err = run_systemctl('start', service)
    if code != 0:
        return jsonify({'error': f'Failed to start {service}: {err}'}), 500
    return jsonify({'status': 'ok', 'message': f'{service} started'})


@app.route('/api/stack/stop', methods=['POST', 'OPTIONS'])
def stack_stop():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    service = get_env_config()['service']
    code, _, err = run_systemctl('stop', service)
    if code != 0:
        return jsonify({'error': f'Failed to stop {service}: {err}'}), 500
    return jsonify({'status': 'ok', 'message': f'{service} stopped'})


@app.route('/api/stack/restart', methods=['POST', 'OPTIONS'])
def stack_restart():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    service = get_env_config()['service']
    code, _, err = run_systemctl('restart', service)
    if code != 0:
        return jsonify({'error': f'Failed to restart {service}: {err}'}), 500
    return jsonify({'status': 'ok', 'message': f'{service} restarted'})


@app.route('/api/localization/restart', methods=['POST', 'OPTIONS'])
def restart_localization():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    cfg      = get_env_config()
    data     = request.get_json(force=True)
    map_name = data.get('map_name', '').strip()
    if not map_name:
        return jsonify({'error': 'map_name is required'}), 400

    map_path = os.path.join(cfg['maps_dir'], f"{map_name}{cfg['map_ext']}")
    if not os.path.isfile(map_path):
        return jsonify({'error': f"Map '{map_name}' not found. "
                                 f"Check /api/maps for available maps."}), 404

    pid, err = run_script(cfg['scripts']['localization'], [map_name])
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'status': 'ok',
                    'message': f'Localization restarting with map: {map_name}',
                    'environment': get_environment(), 'pid': pid})


@app.route('/api/navigation/restart', methods=['POST', 'OPTIONS'])
def restart_navigation():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    pid, err = run_script(get_env_config()['scripts']['navigation'])
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'status': 'ok', 'message': 'Navigation restarting',
                    'environment': get_environment(), 'pid': pid})


@app.route('/api/odometry/restart', methods=['POST', 'OPTIONS'])
def restart_odometry():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    pid, err = run_script(get_env_config()['scripts']['odometry'])
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'status': 'ok', 'message': 'Odometry restarting',
                    'environment': get_environment(), 'pid': pid})


# ══════════════════════════════════════════════════════════════════════════════
# MAPPING routes  (environment-aware)
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/mapping/start', methods=['POST', 'OPTIONS'])
def mapping_start():
    """Start SLAM for the active environment (slam_toolbox indoor / lidarslam outdoor)."""
    if request.method == 'OPTIONS':
        return _cors_preflight()
    pid, err = run_script(get_env_config()['scripts']['mapping_start'])
    if err:
        return jsonify({'error': err}), 500
    return jsonify({
        'status': 'ok',
        'message': 'Mapping started. Localization and navigation have been stopped.',
        'environment': get_environment(),
        'pid': pid
    })


@app.route('/api/mapping/stop_and_save', methods=['POST', 'OPTIONS'])
def mapping_stop_and_save():
    """Stop SLAM, save the map, move it to the maps dir — all in one call."""
    if request.method == 'OPTIONS':
        return _cors_preflight()

    cfg      = get_env_config()
    data     = request.get_json(force=True)
    map_name = data.get('map_name', '').strip()

    # ── Validate name ──────────────────────────────────────
    if not map_name:
        return jsonify({'error': 'map_name is required'}), 400
    if not re.match(r'^[a-zA-Z0-9_-]+$', map_name):
        return jsonify({
            'error': 'Invalid map name. Use only letters, numbers, underscores, hyphens.'
        }), 400

    # ── Check name collision before doing anything ─────────
    dest_map = os.path.join(cfg['maps_dir'], f"{map_name}{cfg['map_ext']}")
    if os.path.isfile(dest_map):
        return jsonify({
            'error': f"Map '{map_name}' already exists. Choose a different name."
        }), 409

    # ── Step 1: Stop SLAM and save to temp ─────────────────
    stop_result = subprocess.run(
        ['/bin/bash', os.path.join(SCRIPTS_DIR, cfg['scripts']['mapping_stop'])],
        env=ROS_ENV, capture_output=True, text=True
    )
    if stop_result.returncode != 0:
        return jsonify({
            'error': f"Failed to stop mapping: {stop_result.stderr.strip()}"
        }), 500

    # ── Step 2: Move temp map to maps dir with given name ──
    save_result = subprocess.run(
        ['/bin/bash', os.path.join(SCRIPTS_DIR, cfg['scripts']['mapping_save']), map_name],
        env=ROS_ENV, capture_output=True, text=True
    )
    if save_result.returncode != 0:
        return jsonify({
            'error': f"Mapping stopped but save failed: {save_result.stderr.strip()}"
        }), 500

    return jsonify({
        'status': 'ok',
        'message': f"Mapping stopped and map saved as '{map_name}'.",
        'environment': get_environment(),
        'map_name': map_name
    })


@app.route('/api/mapping/status', methods=['GET'])
def mapping_status():
    """Is SLAM running, and is a temp map waiting to be saved? (env-aware)"""
    cfg = get_env_config()

    result           = subprocess.run(['pgrep', '-f', cfg['processes']['mapping']],
                                       capture_output=True)
    mapping_running  = result.returncode == 0

    temp_map_ready = all(
        os.path.isfile(f"{cfg['temp_map_path']}{ext}")
        for ext in cfg['temp_map_exts']
    )

    return jsonify({
        'gmapping_running': mapping_running,   # key kept for frontend compatibility
        'temp_map_ready':   temp_map_ready,
        'temp_map_path':    cfg['temp_map_path'] if temp_map_ready else None,
        'environment':      get_environment(),
    })


# ══════════════════════════════════════════════════════════════════════════════
# Health
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok', 'device': DEVICE_ID,
                    'environment': get_environment()})


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("\n🤖 TROV API Server")
    print("─" * 50)
    print(f" Active environment: {get_environment()}")
    print(" PTZ control:")
    print("   POST /api/ptz/move { operation, duration }")
    print("   POST /api/ptz/stop")
    print("   GET  /api/ptz/info")
    print(" Environment:")
    print("   GET  /api/environment")
    print("   POST /api/environment/switch { environment }")
    print(" Robot API:")
    print("   GET  /api/maps")
    print("   GET  /api/status")
    print("   GET  /api/stack/status")
    print("   POST /api/stack/start")
    print("   POST /api/stack/stop")
    print("   POST /api/stack/restart")
    print("   POST /api/localization/restart { map_name }")
    print("   POST /api/navigation/restart")
    print("   POST /api/odometry/restart")
    print(" Mapping:")
    print("   POST /api/mapping/start")
    print("   POST /api/mapping/stop_and_save { map_name }")
    print("   GET  /api/mapping/status")
    print(" GET  /api/health")
    print("─" * 50)
    print(f"\n✅ Serving on http://0.0.0.0:5000\n")
    serve(app, host='0.0.0.0', port=5000, threads=8)


if __name__ == '__main__':
    main()
