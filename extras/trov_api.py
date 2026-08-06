# #!/usr/bin/env python3
# # ============================================================
# # trov_api.py
# # TROV Web API — runs as trov-api.service
# # Handles localization, navigation, odometry restarts,
# # map listing, and full stack control for the web UI.
# #
# # Place this file at: /data/trov_ws/trov_api.py
# # ============================================================

# import os
# import subprocess
# from fastapi import FastAPI, HTTPException
# from pydantic import BaseModel

# app = FastAPI()

# # ---------- Config ----------
# MAPS_DIR    = "/data/trov_ws/install/trov/share/trov/maps"
# SCRIPTS_DIR = "/data/trov_ws"
# ROS_ENV     = {
#     **os.environ,
#     "HOME": "/home/nvidia",
# }


# # ---------- Helper: run a shell script in background ----------
# def run_script(script_name: str, args: list[str] = []):
#     script_path = os.path.join(SCRIPTS_DIR, script_name)

#     if not os.path.isfile(script_path):
#         raise HTTPException(status_code=500, detail=f"Script not found: {script_path}")
#     if not os.access(script_path, os.X_OK):
#         raise HTTPException(status_code=500, detail=f"Script not executable: {script_path}")

#     cmd = ["/bin/bash", script_path] + args
#     proc = subprocess.Popen(cmd, env=ROS_ENV)
#     return proc.pid


# # ---------- Helper: run a systemctl command ----------
# def run_systemctl(action: str, service: str) -> tuple[int, str, str]:
#     result = subprocess.run(
#         ["sudo", "systemctl", action, service],
#         capture_output=True,
#         text=True
#     )
#     return result.returncode, result.stdout.strip(), result.stderr.strip()


# # ── GET /api/maps ─────────────────────────────────────────────────────────────
# @app.get("/api/maps")
# def get_maps():
#     """Return list of available map names from the maps folder."""
#     if not os.path.isdir(MAPS_DIR):
#         raise HTTPException(status_code=500, detail=f"Maps folder not found: {MAPS_DIR}")

#     maps = sorted([
#         f.replace(".yaml", "")
#         for f in os.listdir(MAPS_DIR)
#         if f.endswith(".yaml")
#     ])

#     if not maps:
#         raise HTTPException(status_code=404, detail="No maps found")

#     return {"maps": maps}


# # ── GET /api/status ───────────────────────────────────────────────────────────
# @app.get("/api/status")
# def get_status():
#     """Check which key processes are alive."""
#     processes = {
#         "localization": "map_server",
#         "navigation":   "bt_navigator",
#         "odometry":     "icp_odometry",
#         "rosbridge":    "rosbridge_websocket",
#         "mediamtx":     "mediamtx",
#     }

#     status = {}
#     for name, keyword in processes.items():
#         result = subprocess.run(["pgrep", "-f", keyword], capture_output=True)
#         status[name] = "running" if result.returncode == 0 else "dead"

#     return {"status": status}


# # ── GET /api/stack/status ─────────────────────────────────────────────────────
# @app.get("/api/stack/status")
# def stack_status():
#     """Check if trov.service is running."""
#     result = subprocess.run(
#         ["systemctl", "is-active", "trov.service"],
#         capture_output=True,
#         text=True
#     )
#     state = result.stdout.strip()   # "active", "inactive", "failed"
#     return {"trov_service": state}


# # ── POST /api/stack/start ─────────────────────────────────────────────────────
# @app.post("/api/stack/start")
# def stack_start():
#     """Start trov.service (full robot stack)."""
#     result = subprocess.run(
#         ["systemctl", "is-active", "trov.service"],
#         capture_output=True, text=True
#     )
#     if result.stdout.strip() == "active":
#         return {"status": "already_running", "message": "trov.service is already active"}

#     code, out, err = run_systemctl("start", "trov.service")
#     if code != 0:
#         raise HTTPException(status_code=500, detail=f"Failed to start trov.service: {err}")

#     return {"status": "ok", "message": "trov.service started"}


# # ── POST /api/stack/stop ──────────────────────────────────────────────────────
# @app.post("/api/stack/stop")
# def stack_stop():
#     """Stop trov.service (full robot stack)."""
#     code, out, err = run_systemctl("stop", "trov.service")
#     if code != 0:
#         raise HTTPException(status_code=500, detail=f"Failed to stop trov.service: {err}")

#     return {"status": "ok", "message": "trov.service stopped"}


# # ── POST /api/stack/restart ───────────────────────────────────────────────────
# @app.post("/api/stack/restart")
# def stack_restart():
#     """Restart trov.service (full robot stack)."""
#     code, out, err = run_systemctl("restart", "trov.service")
#     if code != 0:
#         raise HTTPException(status_code=500, detail=f"Failed to restart trov.service: {err}")

#     return {"status": "ok", "message": "trov.service restarted"}


# # ── POST /api/localization/restart ────────────────────────────────────────────
# class LocalizationRequest(BaseModel):
#     map_name: str

# @app.post("/api/localization/restart")
# def restart_localization(req: LocalizationRequest):
#     """Restart localization with the given map name."""
#     map_path = os.path.join(MAPS_DIR, f"{req.map_name}.yaml")
#     if not os.path.isfile(map_path):
#         raise HTTPException(
#             status_code=404,
#             detail=f"Map '{req.map_name}' not found. Check /api/maps for available maps."
#         )

#     pid = run_script("restart_localization.sh", [req.map_name])
#     return {
#         "status": "ok",
#         "message": f"Localization restarting with map: {req.map_name}",
#         "pid": pid
#     }


# # ── POST /api/navigation/restart ──────────────────────────────────────────────
# @app.post("/api/navigation/restart")
# def restart_navigation():
#     """Restart the navigation stack."""
#     pid = run_script("restart_navigation.sh")
#     return {
#         "status": "ok",
#         "message": "Navigation restarting",
#         "pid": pid
#     }


# # ── POST /api/odometry/restart ────────────────────────────────────────────────
# @app.post("/api/odometry/restart")
# def restart_odometry():
#     """Restart ICP odometry."""
#     pid = run_script("restart_odometry.sh")
#     return {
#         "status": "ok",
#         "message": "Odometry restarting",
#         "pid": pid
#     }





# #!/usr/bin/env python3
# """
# TROV Unified Server — port 5000
# ────────────────────────────────
# Camera streams (MJPEG):
#   GET  /driving_camera
#   GET  /surveillance_camera

# PTZ control:
#   POST /api/ptz/move   { "operation": "LEFT", "duration": 500 }
#   POST /api/ptz/stop
#   GET  /api/ptz/info
#   GET  /api/health
# """

# import subprocess, threading, time, asyncio, traceback
# import numpy as np
# import cv2
# import aiohttp
# from flask import Flask, Response, jsonify, request
# from flask_cors import CORS
# from imouapi.api import ImouAPIClient
# from imouapi.exceptions import ImouException
# from waitress import serve

# # ─── PTZ config ───────────────────────────────────────────────────────────────
# APP_ID     = "lc09e486a9ebaa4239"
# APP_SECRET = "97a77a752d5248cbbacd25de36caef"
# DEVICE_ID  = "A889AAMPGVAFE14"

# VALID_OPERATIONS = {
#     "UP", "DOWN", "LEFT", "RIGHT",
#     "UPPER_LEFT", "UPPER_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT",
#     "ZOOM_IN", "ZOOM_OUT", "STOP"
# }

# # ─── Camera config ────────────────────────────────────────────────────────────
# # Both cameras routed through MediaMTX (localhost:8554)
# # MediaMTX handles auth + reconnect to the real cameras
# CAMERAS = {
#     'driving_camera': {
#         'url':    'rtsp://localhost:8554/driving_camera',
#         'width':  None,
#         'height': None,
#     },
#     'surveillance_camera': {
#         'url':    'rtsp://localhost:8554/surveillance_camera',
#         'width':  None,
#         'height': None,
#     },
# }

# OUTPUT_W       = 1280
# OUTPUT_H       = 720
# JPEG_QUALITY   = 80
# TARGET_FPS     = 25
# FRAME_INTERVAL = 1.0 / TARGET_FPS

# latest_frames = {cam: None for cam in CAMERAS}
# frame_locks   = {cam: threading.Lock() for cam in CAMERAS}

# # ─── Flask app ────────────────────────────────────────────────────────────────
# app = Flask(__name__)
# app.url_map.strict_slashes = False      # /driving_camera/ == /driving_camera
# CORS(app, resources={r"/*": {"origins": "*"}})


# # ══════════════════════════════════════════════════════════════════════════════
# #  CAMERA — FFmpeg capture
# # ══════════════════════════════════════════════════════════════════════════════

# def probe_stream(rtsp_url, timeout=10):
#     try:
#         r = subprocess.run(
#             [
#                 "ffprobe", "-v", "error",
#                 "-rtsp_transport", "tcp",
#                 "-select_streams", "v:0",
#                 "-show_entries", "stream=width,height,codec_name",
#                 "-of", "csv=p=0",
#                 "-i", rtsp_url,
#             ],
#             capture_output=True, text=True, timeout=timeout
#         )
#         out = r.stdout.strip()
#         print(f"[probe] {rtsp_url} → '{out}'")
#         if out:
#             parts = out.split(',')
#             if len(parts) == 3:
#                 codec, w, h = parts[0], int(parts[1]), int(parts[2])
#             elif len(parts) == 2:
#                 codec, w, h = 'unknown', int(parts[0]), int(parts[1])
#             else:
#                 return None
#             return {'codec': codec, 'width': w, 'height': h}
#     except Exception as e:
#         print(f"[probe] Error: {e}")
#     return None


# def build_ffmpeg_cmd(rtsp_url, out_w, out_h, codec=None):
#     cmd = [
#         "ffmpeg",
#         "-rtsp_transport", "tcp",
#         "-allowed_media_types", "video",
#         "-fflags", "nobuffer",
#         "-flags", "low_delay",
#     ]
#     # Explicit software decoder for HEVC — avoids Jetson hw negotiation failures
#     if codec and 'hevc' in codec.lower():
#         cmd += ["-vcodec", "hevc"]
#     cmd += [
#         "-i", rtsp_url,
#         "-an",
#         "-vcodec", "rawvideo",
#         "-pix_fmt", "bgr24",
#         "-vf", f"scale={out_w}:{out_h}",
#         "-f", "rawvideo",
#         "-",
#     ]
#     return cmd


# def capture_thread(cam_name, cam_cfg):
#     rtsp_url = cam_cfg['url']
#     w, h     = cam_cfg['width'], cam_cfg['height']
#     codec    = None

#     # Auto-detect resolution + codec via ffprobe
#     if w is None or h is None:
#         print(f"[{cam_name}] Probing stream...")
#         info = probe_stream(rtsp_url)
#         if info:
#             w     = info['width']
#             h     = info['height']
#             codec = info['codec']
#             print(f"[{cam_name}] ✅ {w}x{h}  codec={codec}")
#         else:
#             w, h = 640, 480
#             print(f"[{cam_name}] ⚠️  Probe failed — falling back to 640x480")

#     frame_size = OUTPUT_W * OUTPUT_H * 3

#     while True:
#         print(f"[{cam_name}] Starting FFmpeg ({w}x{h} → {OUTPUT_W}x{OUTPUT_H})")
#         proc = subprocess.Popen(
#             build_ffmpeg_cmd(rtsp_url, OUTPUT_W, OUTPUT_H, codec),
#             stdout=subprocess.PIPE,
#             stderr=subprocess.PIPE,
#             bufsize=10 ** 8,
#         )

#         # Log only real errors from FFmpeg stderr
#         def _log_err(p, name):
#             for line in p.stderr:
#                 line = line.decode(errors='replace').strip()
#                 if any(k in line for k in ['error', 'Error', 'failed', 'Invalid', 'Unauth', 'No such']):
#                     print(f"  [{name}][ffmpeg] {line}")

#         threading.Thread(target=_log_err, args=(proc, cam_name), daemon=True).start()

#         short = 0
#         while True:
#             raw = proc.stdout.read(frame_size)

#             if len(raw) != frame_size:
#                 short += 1
#                 if short >= 3:
#                     print(f"[{cam_name}] ⚠️  Stream lost — restarting in 3s")
#                     proc.kill()
#                     time.sleep(3)
#                     break
#                 continue

#             short = 0
#             frame = np.frombuffer(raw, np.uint8).reshape((OUTPUT_H, OUTPUT_W, 3))
#             ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
#             if ok:
#                 with frame_locks[cam_name]:
#                     latest_frames[cam_name] = jpeg.tobytes()


# def generate_mjpeg(cam_name):
#     """Yields MJPEG multipart frames throttled to TARGET_FPS."""
#     while True:
#         t0 = time.monotonic()

#         with frame_locks[cam_name]:
#             frame = latest_frames[cam_name]

#         if frame is None:
#             time.sleep(0.01)
#             continue

#         yield (
#             b'--frame\r\n'
#             b'Content-Type: image/jpeg\r\n\r\n'
#             + frame
#             + b'\r\n'
#         )

#         wait = FRAME_INTERVAL - (time.monotonic() - t0)
#         if wait > 0:
#             time.sleep(wait)


# # ══════════════════════════════════════════════════════════════════════════════
# #  CAMERA routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/driving_camera', methods=['GET'])
# def route_driving_camera():
#     return Response(
#         generate_mjpeg('driving_camera'),
#         mimetype='multipart/x-mixed-replace; boundary=frame',
#         headers={
#             'Cache-Control':              'no-cache, no-store',
#             'Access-Control-Allow-Origin': '*',
#         }
#     )


# @app.route('/surveillance_camera', methods=['GET'])
# def route_surveillance_camera():
#     return Response(
#         generate_mjpeg('surveillance_camera'),
#         mimetype='multipart/x-mixed-replace; boundary=frame',
#         headers={
#             'Cache-Control':              'no-cache, no-store',
#             'Access-Control-Allow-Origin': '*',
#         }
#     )


# # ══════════════════════════════════════════════════════════════════════════════
# #  PTZ helpers
# # ══════════════════════════════════════════════════════════════════════════════

# def run_async(coro):
#     loop = asyncio.new_event_loop()
#     try:
#         return loop.run_until_complete(coro)
#     finally:
#         loop.close()


# async def _ptz_move(operation, duration):
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         await client.async_api_controlMovePTZ(DEVICE_ID, operation, duration)


# async def _ptz_stop():
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         await client.async_api_controlMovePTZ(DEVICE_ID, "STOP", 0)


# async def _ptz_info():
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         return await client.async_api_devicePTZInfo(DEVICE_ID)


# # ══════════════════════════════════════════════════════════════════════════════
# #  PTZ routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/ptz/move', methods=['POST', 'OPTIONS'])
# def ptz_move():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     data      = request.get_json(force=True)
#     operation = str(data.get('operation', '')).upper()
#     duration  = int(data.get('duration', 500))

#     if operation not in VALID_OPERATIONS:
#         return jsonify({'success': False, 'error': f"Invalid operation '{operation}'"}), 400

#     try:
#         run_async(_ptz_move(operation, duration))
#         return jsonify({'success': True, 'operation': operation, 'duration': duration})
#     except ImouException as e:
#         return jsonify({'success': False, 'error': str(e)}), 500
#     except Exception as e:
#         traceback.print_exc()
#         return jsonify({'success': False, 'error': repr(e)}), 500


# @app.route('/api/ptz/stop', methods=['POST', 'OPTIONS'])
# def ptz_stop():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     try:
#         run_async(_ptz_stop())
#         return jsonify({'success': True, 'operation': 'STOP'})
#     except Exception as e:
#         traceback.print_exc()
#         return jsonify({'success': False, 'error': str(e)}), 500


# @app.route('/api/ptz/info', methods=['GET'])
# def ptz_info():
#     try:
#         info = run_async(_ptz_info())
#         return jsonify({'success': True, 'data': info})
#     except Exception as e:
#         return jsonify({'success': False, 'error': str(e)}), 500


# @app.route('/api/health', methods=['GET'])
# def health():
#     return jsonify({
#         'status':  'ok',
#         'device':  DEVICE_ID,
#         'cameras': {name: (latest_frames[name] is not None) for name in CAMERAS},
#     })


# def _cors_preflight():
#     """Return 200 for OPTIONS preflight with CORS headers."""
#     resp = app.make_response('')
#     resp.status_code = 200
#     resp.headers['Access-Control-Allow-Origin']  = '*'
#     resp.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
#     resp.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
#     return resp


# # ══════════════════════════════════════════════════════════════════════════════
# #  Main
# # ══════════════════════════════════════════════════════════════════════════════

# def main():
#     print("\n🎥 TROV Unified Server")
#     print("─" * 50)
#     print("  Camera streams:")
#     print("    GET  /driving_camera")
#     print("    GET  /surveillance_camera")
#     print("  PTZ control:")
#     print("    POST /api/ptz/move   { operation, duration }")
#     print("    POST /api/ptz/stop")
#     print("    GET  /api/ptz/info")
#     print("    GET  /api/health")
#     print("─" * 50)
#     print("  Requires MediaMTX running on rtsp://localhost:8554")
#     print("─" * 50)

#     # Start FFmpeg capture threads for each camera
#     for cam_name, cam_cfg in CAMERAS.items():
#         threading.Thread(
#             target=capture_thread,
#             args=(cam_name, cam_cfg),
#             daemon=True,
#         ).start()

#     # waitress: production WSGI — handles concurrent streams + POST without 501
#     print(f"\n✅ Serving on http://0.0.0.0:5000\n")
#     serve(app, host='0.0.0.0', port=5000, threads=16)


# if __name__ == '__main__':
#     main()





# #!/usr/bin/env python3
# """
# TROV Unified Server — port 5000
# ────────────────────────────────
# PTZ control:
#   POST /api/ptz/move   { "operation": "LEFT", "duration": 500 }
#   POST /api/ptz/stop
#   GET  /api/ptz/info

# Robot API:
#   GET  /api/maps
#   GET  /api/status
#   GET  /api/stack/status
#   POST /api/stack/start
#   POST /api/stack/stop
#   POST /api/stack/restart
#   POST /api/localization/restart   { "map_name": "..." }
#   POST /api/navigation/restart
#   POST /api/odometry/restart
#   GET  /api/health
# """

# import os
# import subprocess
# import asyncio
# import traceback

# import aiohttp
# from flask import Flask, jsonify, request
# from flask_cors import CORS
# from imouapi.api import ImouAPIClient
# from imouapi.exceptions import ImouException
# from waitress import serve

# # ─── PTZ config ───────────────────────────────────────────────────────────────
# APP_ID     = "lc09e486a9ebaa4239"
# APP_SECRET = "97a77a752d5248cbbacd25de36caef"
# DEVICE_ID  = "A889AAMPGVAFE14"

# VALID_OPERATIONS = {
#     "UP", "DOWN", "LEFT", "RIGHT",
#     "UPPER_LEFT", "UPPER_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT",
#     "ZOOM_IN", "ZOOM_OUT", "STOP"
# }

# # ─── Robot API config ─────────────────────────────────────────────────────────
# MAPS_DIR    = "/data/trov_ws/install/trov/share/trov/maps"
# SCRIPTS_DIR = "/data/trov_ws"
# ROS_ENV     = {**os.environ, "HOME": "/home/nvidia"}

# # ─── Flask app ────────────────────────────────────────────────────────────────
# app = Flask(__name__)
# app.url_map.strict_slashes = False
# CORS(app, resources={r"/*": {"origins": "*"}})


# # ══════════════════════════════════════════════════════════════════════════════
# #  HELPERS
# # ══════════════════════════════════════════════════════════════════════════════

# def _cors_preflight():
#     resp = app.make_response('')
#     resp.status_code = 200
#     resp.headers['Access-Control-Allow-Origin']  = '*'
#     resp.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
#     resp.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
#     return resp


# def run_script(script_name: str, args: list = []):
#     script_path = os.path.join(SCRIPTS_DIR, script_name)
#     if not os.path.isfile(script_path):
#         return None, f"Script not found: {script_path}"
#     if not os.access(script_path, os.X_OK):
#         return None, f"Script not executable: {script_path}"
#     proc = subprocess.Popen(["/bin/bash", script_path] + args, env=ROS_ENV)
#     return proc.pid, None


# def run_systemctl(action: str, service: str):
#     result = subprocess.run(
#         ["sudo", "systemctl", action, service],
#         capture_output=True, text=True
#     )
#     return result.returncode, result.stdout.strip(), result.stderr.strip()


# # ══════════════════════════════════════════════════════════════════════════════
# #  PTZ helpers
# # ══════════════════════════════════════════════════════════════════════════════

# def run_async(coro):
#     loop = asyncio.new_event_loop()
#     try:
#         return loop.run_until_complete(coro)
#     finally:
#         loop.close()


# async def _ptz_move(operation, duration):
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         await client.async_api_controlMovePTZ(DEVICE_ID, operation, duration)


# async def _ptz_stop():
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         await client.async_api_controlMovePTZ(DEVICE_ID, "STOP", 0)


# async def _ptz_info():
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         return await client.async_api_devicePTZInfo(DEVICE_ID)


# # ══════════════════════════════════════════════════════════════════════════════
# #  PTZ routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/ptz/move', methods=['POST', 'OPTIONS'])
# def ptz_move():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     data      = request.get_json(force=True)
#     operation = str(data.get('operation', '')).upper()
#     duration  = int(data.get('duration', 500))
#     if operation not in VALID_OPERATIONS:
#         return jsonify({'success': False, 'error': f"Invalid operation '{operation}'"}), 400
#     try:
#         run_async(_ptz_move(operation, duration))
#         return jsonify({'success': True, 'operation': operation, 'duration': duration})
#     except ImouException as e:
#         return jsonify({'success': False, 'error': str(e)}), 500
#     except Exception as e:
#         traceback.print_exc()
#         return jsonify({'success': False, 'error': repr(e)}), 500


# @app.route('/api/ptz/stop', methods=['POST', 'OPTIONS'])
# def ptz_stop():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     try:
#         run_async(_ptz_stop())
#         return jsonify({'success': True, 'operation': 'STOP'})
#     except Exception as e:
#         traceback.print_exc()
#         return jsonify({'success': False, 'error': str(e)}), 500


# @app.route('/api/ptz/info', methods=['GET'])
# def ptz_info():
#     try:
#         info = run_async(_ptz_info())
#         return jsonify({'success': True, 'data': info})
#     except Exception as e:
#         return jsonify({'success': False, 'error': str(e)}), 500


# # ══════════════════════════════════════════════════════════════════════════════
# #  ROBOT API routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/maps', methods=['GET'])
# def get_maps():
#     if not os.path.isdir(MAPS_DIR):
#         return jsonify({'error': f'Maps folder not found: {MAPS_DIR}'}), 500
#     maps = sorted([
#         f.replace('.yaml', '')
#         for f in os.listdir(MAPS_DIR)
#         if f.endswith('.yaml')
#     ])
#     if not maps:
#         return jsonify({'error': 'No maps found'}), 404
#     return jsonify({'maps': maps})


# @app.route('/api/status', methods=['GET'])
# def get_status():
#     processes = {
#         'localization': 'map_server',
#         'navigation':   'bt_navigator',
#         'odometry':     'icp_odometry',
#         'rosbridge':    'rosbridge_websocket',
#         'mediamtx':     'mediamtx',
#     }
#     status = {}
#     for name, keyword in processes.items():
#         result = subprocess.run(['pgrep', '-f', keyword], capture_output=True)
#         status[name] = 'running' if result.returncode == 0 else 'dead'
#     return jsonify({'status': status})


# @app.route('/api/stack/status', methods=['GET'])
# def stack_status():
#     result = subprocess.run(
#         ['systemctl', 'is-active', 'trov.service'],
#         capture_output=True, text=True
#     )
#     return jsonify({'trov_service': result.stdout.strip()})


# @app.route('/api/stack/start', methods=['POST', 'OPTIONS'])
# def stack_start():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     result = subprocess.run(
#         ['systemctl', 'is-active', 'trov.service'],
#         capture_output=True, text=True
#     )
#     if result.stdout.strip() == 'active':
#         return jsonify({'status': 'already_running', 'message': 'trov.service is already active'})
#     code, _, err = run_systemctl('start', 'trov.service')
#     if code != 0:
#         return jsonify({'error': f'Failed to start trov.service: {err}'}), 500
#     return jsonify({'status': 'ok', 'message': 'trov.service started'})


# @app.route('/api/stack/stop', methods=['POST', 'OPTIONS'])
# def stack_stop():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     code, _, err = run_systemctl('stop', 'trov.service')
#     if code != 0:
#         return jsonify({'error': f'Failed to stop trov.service: {err}'}), 500
#     return jsonify({'status': 'ok', 'message': 'trov.service stopped'})


# @app.route('/api/stack/restart', methods=['POST', 'OPTIONS'])
# def stack_restart():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     code, _, err = run_systemctl('restart', 'trov.service')
#     if code != 0:
#         return jsonify({'error': f'Failed to restart trov.service: {err}'}), 500
#     return jsonify({'status': 'ok', 'message': 'trov.service restarted'})


# @app.route('/api/localization/restart', methods=['POST', 'OPTIONS'])
# def restart_localization():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     data     = request.get_json(force=True)
#     map_name = data.get('map_name', '').strip()
#     if not map_name:
#         return jsonify({'error': 'map_name is required'}), 400
#     map_path = os.path.join(MAPS_DIR, f'{map_name}.yaml')
#     if not os.path.isfile(map_path):
#         return jsonify({'error': f"Map '{map_name}' not found. Check /api/maps for available maps."}), 404
#     pid, err = run_script('restart_localization.sh', [map_name])
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({'status': 'ok', 'message': f'Localization restarting with map: {map_name}', 'pid': pid})


# @app.route('/api/navigation/restart', methods=['POST', 'OPTIONS'])
# def restart_navigation():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     pid, err = run_script('restart_navigation.sh')
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({'status': 'ok', 'message': 'Navigation restarting', 'pid': pid})


# @app.route('/api/odometry/restart', methods=['POST', 'OPTIONS'])
# def restart_odometry():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     pid, err = run_script('restart_odometry.sh')
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({'status': 'ok', 'message': 'Odometry restarting', 'pid': pid})


# @app.route('/api/health', methods=['GET'])
# def health():
#     return jsonify({'status': 'ok', 'device': DEVICE_ID})


# # ══════════════════════════════════════════════════════════════════════════════
# #  Main
# # ══════════════════════════════════════════════════════════════════════════════

# def main():
#     print("\n🤖 TROV API Server")
#     print("─" * 50)
#     print("  PTZ control:")
#     print("    POST /api/ptz/move   { operation, duration }")
#     print("    POST /api/ptz/stop")
#     print("    GET  /api/ptz/info")
#     print("  Robot API:")
#     print("    GET  /api/maps")
#     print("    GET  /api/status")
#     print("    GET  /api/stack/status")
#     print("    POST /api/stack/start")
#     print("    POST /api/stack/stop")
#     print("    POST /api/stack/restart")
#     print("    POST /api/localization/restart  { map_name }")
#     print("    POST /api/navigation/restart")
#     print("    POST /api/odometry/restart")
#     print("    GET  /api/health")
#     print("─" * 50)

#     print(f"\n✅ Serving on http://0.0.0.0:5000\n")
#     serve(app, host='0.0.0.0', port=5000, threads=8)


# if __name__ == '__main__':
#     main()






















# #!/usr/bin/env python3
# """
# TROV Unified Server — port 5000
# ────────────────────────────────
# PTZ control:
#  POST /api/ptz/move { "operation": "LEFT", "duration": 500 }
#  POST /api/ptz/stop
#  GET  /api/ptz/info

# Robot API:
#  GET  /api/maps
#  GET  /api/status
#  GET  /api/stack/status
#  POST /api/stack/start
#  POST /api/stack/stop
#  POST /api/stack/restart
#  POST /api/localization/restart { "map_name": "..." }
#  POST /api/navigation/restart
#  POST /api/odometry/restart

# Mapping API:
#  POST /api/mapping/start
#  POST /api/mapping/stop_and_save  { "map_name": "..." }
#  GET  /api/mapping/status

#  GET  /api/health
# """

# import os
# import re
# import subprocess
# import asyncio
# import traceback

# import aiohttp
# from flask import Flask, jsonify, request
# from flask_cors import CORS
# from imouapi.api import ImouAPIClient
# from imouapi.exceptions import ImouException
# from waitress import serve

# # ─── PTZ config ───────────────────────────────────────────────────────────────
# APP_ID     = "lc09e486a9ebaa4239"
# APP_SECRET = "97a77a752d5248cbbacd25de36caef"
# DEVICE_ID  = "A889AAMPGVAFE14"

# VALID_OPERATIONS = {
#     "UP", "DOWN", "LEFT", "RIGHT",
#     "UPPER_LEFT", "UPPER_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT",
#     "ZOOM_IN", "ZOOM_OUT", "STOP"
# }

# # ─── Robot API config ─────────────────────────────────────────────────────────
# MAPS_DIR      = "/data/trov_ws/install/trov/share/trov/maps"
# SCRIPTS_DIR   = "/data/trov_ws"
# TEMP_MAP_PATH = "/tmp/trov_map_temp"
# ROS_ENV       = {**os.environ, "HOME": "/home/nvidia"}

# # ─── Flask app ────────────────────────────────────────────────────────────────
# app = Flask(__name__)
# app.url_map.strict_slashes = False
# CORS(app, resources={r"/*": {"origins": "*"}})


# # ══════════════════════════════════════════════════════════════════════════════
# # HELPERS
# # ══════════════════════════════════════════════════════════════════════════════

# def _cors_preflight():
#     resp = app.make_response('')
#     resp.status_code = 200
#     resp.headers['Access-Control-Allow-Origin']  = '*'
#     resp.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
#     resp.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
#     return resp


# def run_script(script_name: str, args: list = []):
#     script_path = os.path.join(SCRIPTS_DIR, script_name)
#     if not os.path.isfile(script_path):
#         return None, f"Script not found: {script_path}"
#     if not os.access(script_path, os.X_OK):
#         return None, f"Script not executable: {script_path}"
#     proc = subprocess.Popen(["/bin/bash", script_path] + args, env=ROS_ENV)
#     return proc.pid, None


# def run_systemctl(action: str, service: str):
#     result = subprocess.run(
#         ["sudo", "systemctl", action, service],
#         capture_output=True, text=True
#     )
#     return result.returncode, result.stdout.strip(), result.stderr.strip()


# # ══════════════════════════════════════════════════════════════════════════════
# # PTZ helpers
# # ══════════════════════════════════════════════════════════════════════════════

# def run_async(coro):
#     loop = asyncio.new_event_loop()
#     try:
#         return loop.run_until_complete(coro)
#     finally:
#         loop.close()


# async def _ptz_move(operation, duration):
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         await client.async_api_controlMovePTZ(DEVICE_ID, operation, duration)


# async def _ptz_stop():
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         await client.async_api_controlMovePTZ(DEVICE_ID, "STOP", 0)


# async def _ptz_info():
#     async with aiohttp.ClientSession() as session:
#         client = ImouAPIClient(APP_ID, APP_SECRET, session)
#         await client.async_connect()
#         return await client.async_api_devicePTZInfo(DEVICE_ID)


# # ══════════════════════════════════════════════════════════════════════════════
# # PTZ routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/ptz/move', methods=['POST', 'OPTIONS'])
# def ptz_move():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     data      = request.get_json(force=True)
#     operation = str(data.get('operation', '')).upper()
#     duration  = int(data.get('duration', 500))
#     if operation not in VALID_OPERATIONS:
#         return jsonify({'success': False, 'error': f"Invalid operation '{operation}'"}), 400
#     try:
#         run_async(_ptz_move(operation, duration))
#         return jsonify({'success': True, 'operation': operation, 'duration': duration})
#     except ImouException as e:
#         return jsonify({'success': False, 'error': str(e)}), 500
#     except Exception as e:
#         traceback.print_exc()
#         return jsonify({'success': False, 'error': repr(e)}), 500


# @app.route('/api/ptz/stop', methods=['POST', 'OPTIONS'])
# def ptz_stop():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     try:
#         run_async(_ptz_stop())
#         return jsonify({'success': True, 'operation': 'STOP'})
#     except Exception as e:
#         traceback.print_exc()
#         return jsonify({'success': False, 'error': str(e)}), 500


# @app.route('/api/ptz/info', methods=['GET'])
# def ptz_info():
#     try:
#         info = run_async(_ptz_info())
#         return jsonify({'success': True, 'data': info})
#     except Exception as e:
#         return jsonify({'success': False, 'error': str(e)}), 500


# # ══════════════════════════════════════════════════════════════════════════════
# # ROBOT API routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/maps', methods=['GET'])
# def get_maps():
#     if not os.path.isdir(MAPS_DIR):
#         return jsonify({'error': f'Maps folder not found: {MAPS_DIR}'}), 500
#     maps = sorted([
#         f.replace('.yaml', '')
#         for f in os.listdir(MAPS_DIR)
#         if f.endswith('.yaml')
#     ])
#     if not maps:
#         return jsonify({'error': 'No maps found'}), 404
#     return jsonify({'maps': maps})


# @app.route('/api/status', methods=['GET'])
# def get_status():
#     processes = {
#         'localization': 'map_server',
#         'navigation':   'bt_navigator',
#         'odometry':     'icp_odometry',
#         'rosbridge':    'rosbridge_websocket',
#         'mediamtx':     'mediamtx',
#         'mapping':      'slam_toolbox',
#     }
#     status = {}
#     for name, keyword in processes.items():
#         result = subprocess.run(['pgrep', '-f', keyword], capture_output=True)
#         status[name] = 'running' if result.returncode == 0 else 'dead'
#     return jsonify({'status': status})


# @app.route('/api/stack/status', methods=['GET'])
# def stack_status():
#     result = subprocess.run(
#         ['systemctl', 'is-active', 'trov.service'],
#         capture_output=True, text=True
#     )
#     return jsonify({'trov_service': result.stdout.strip()})


# @app.route('/api/stack/start', methods=['POST', 'OPTIONS'])
# def stack_start():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     result = subprocess.run(
#         ['systemctl', 'is-active', 'trov.service'],
#         capture_output=True, text=True
#     )
#     if result.stdout.strip() == 'active':
#         return jsonify({'status': 'already_running', 'message': 'trov.service is already active'})
#     code, _, err = run_systemctl('start', 'trov.service')
#     if code != 0:
#         return jsonify({'error': f'Failed to start trov.service: {err}'}), 500
#     return jsonify({'status': 'ok', 'message': 'trov.service started'})


# @app.route('/api/stack/stop', methods=['POST', 'OPTIONS'])
# def stack_stop():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     code, _, err = run_systemctl('stop', 'trov.service')
#     if code != 0:
#         return jsonify({'error': f'Failed to stop trov.service: {err}'}), 500
#     return jsonify({'status': 'ok', 'message': 'trov.service stopped'})


# @app.route('/api/stack/restart', methods=['POST', 'OPTIONS'])
# def stack_restart():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     code, _, err = run_systemctl('restart', 'trov.service')
#     if code != 0:
#         return jsonify({'error': f'Failed to restart trov.service: {err}'}), 500
#     return jsonify({'status': 'ok', 'message': 'trov.service restarted'})


# @app.route('/api/localization/restart', methods=['POST', 'OPTIONS'])
# def restart_localization():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     data     = request.get_json(force=True)
#     map_name = data.get('map_name', '').strip()
#     if not map_name:
#         return jsonify({'error': 'map_name is required'}), 400
#     map_path = os.path.join(MAPS_DIR, f'{map_name}.yaml')
#     if not os.path.isfile(map_path):
#         return jsonify({'error': f"Map '{map_name}' not found. Check /api/maps for available maps."}), 404
#     pid, err = run_script('restart_localization.sh', [map_name])
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({'status': 'ok', 'message': f'Localization restarting with map: {map_name}', 'pid': pid})


# @app.route('/api/navigation/restart', methods=['POST', 'OPTIONS'])
# def restart_navigation():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     pid, err = run_script('restart_navigation.sh')
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({'status': 'ok', 'message': 'Navigation restarting', 'pid': pid})


# @app.route('/api/odometry/restart', methods=['POST', 'OPTIONS'])
# def restart_odometry():
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     pid, err = run_script('restart_odometry.sh')
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({'status': 'ok', 'message': 'Odometry restarting', 'pid': pid})


# # ══════════════════════════════════════════════════════════════════════════════
# # MAPPING routes
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/mapping/start', methods=['POST', 'OPTIONS'])
# def mapping_start():
#     """Stop localization + navigation, then start slam_toolbox SLAM."""
#     if request.method == 'OPTIONS':
#         return _cors_preflight()
#     pid, err = run_script('start_mapping.sh')
#     if err:
#         return jsonify({'error': err}), 500
#     return jsonify({
#         'status': 'ok',
#         'message': 'Mapping started. Localization and navigation have been stopped.',
#         'pid': pid
#     })


# @app.route('/api/mapping/stop_and_save', methods=['POST', 'OPTIONS'])
# def mapping_stop_and_save():
#     """Stop slam_toolbox, save map, and move it to maps dir — all in one call."""
#     if request.method == 'OPTIONS':
#         return _cors_preflight()

#     data     = request.get_json(force=True)
#     map_name = data.get('map_name', '').strip()

#     # ── Validate name ──────────────────────────────────────
#     if not map_name:
#         return jsonify({'error': 'map_name is required'}), 400

#     if not re.match(r'^[a-zA-Z0-9_-]+$', map_name):
#         return jsonify({
#             'error': 'Invalid map name. Use only letters, numbers, underscores, hyphens.'
#         }), 400

#     # ── Check name collision before doing anything ─────────
#     dest_yaml = os.path.join(MAPS_DIR, f"{map_name}.yaml")
#     if os.path.isfile(dest_yaml):
#         return jsonify({
#             'error': f"Map '{map_name}' already exists. Choose a different name."
#         }), 409

#     # ── Step 1: Stop slam_toolbox and save to temp ─────────────
#     stop_result = subprocess.run(
#         ['/bin/bash', os.path.join(SCRIPTS_DIR, 'stop_mapping.sh')],
#         env=ROS_ENV,
#         capture_output=True,
#         text=True
#     )
#     if stop_result.returncode != 0:
#         return jsonify({
#             'error': f"Failed to stop mapping: {stop_result.stderr.strip()}"
#         }), 500

#     # ── Step 2: Move temp map to maps dir with given name ──
#     save_result = subprocess.run(
#         ['/bin/bash', os.path.join(SCRIPTS_DIR, 'save_map.sh'), map_name],
#         env=ROS_ENV,
#         capture_output=True,
#         text=True
#     )
#     if save_result.returncode != 0:
#         return jsonify({
#             'error': f"Mapping stopped but save failed: {save_result.stderr.strip()}"
#         }), 500

#     return jsonify({
#         'status': 'ok',
#         'message': f"Mapping stopped and map saved as '{map_name}'.",
#         'map_name': map_name
#     })


# @app.route('/api/mapping/status', methods=['GET'])
# def mapping_status():
#     """Check if slam_toolbox is running and whether a temp map is waiting to be saved."""
#     result          = subprocess.run(['pgrep', '-f', 'slam_toolbox'], capture_output=True)
#     slam_toolbox_running = result.returncode == 0

#     temp_map_ready = (
#         os.path.isfile(f"{TEMP_MAP_PATH}.yaml") and
#         os.path.isfile(f"{TEMP_MAP_PATH}.pgm")
#     )

#     return jsonify({
#         'slam_toolbox_running': slam_toolbox_running,
#         'temp_map_ready':   temp_map_ready,
#         'temp_map_path':    TEMP_MAP_PATH if temp_map_ready else None
#     })


# # ══════════════════════════════════════════════════════════════════════════════
# # Health
# # ══════════════════════════════════════════════════════════════════════════════

# @app.route('/api/health', methods=['GET'])
# def health():
#     return jsonify({'status': 'ok', 'device': DEVICE_ID})


# # ══════════════════════════════════════════════════════════════════════════════
# # Main
# # ══════════════════════════════════════════════════════════════════════════════

# def main():
#     print("\n🤖 TROV API Server")
#     print("─" * 50)
#     print(" PTZ control:")
#     print("   POST /api/ptz/move { operation, duration }")
#     print("   POST /api/ptz/stop")
#     print("   GET  /api/ptz/info")
#     print(" Robot API:")
#     print("   GET  /api/maps")
#     print("   GET  /api/status")
#     print("   GET  /api/stack/status")
#     print("   POST /api/stack/start")
#     print("   POST /api/stack/stop")
#     print("   POST /api/stack/restart")
#     print("   POST /api/localization/restart { map_name }")
#     print("   POST /api/navigation/restart")
#     print("   POST /api/odometry/restart")
#     print(" Mapping:")
#     print("   POST /api/mapping/start")
#     print("   POST /api/mapping/stop_and_save { map_name }")
#     print("   GET  /api/mapping/status")
#     print(" GET  /api/health")
#     print("─" * 50)
#     print(f"\n✅ Serving on http://0.0.0.0:5000\n")
#     serve(app, host='0.0.0.0', port=5000, threads=8)


# if __name__ == '__main__':
#     main()#!/usr/bin/env python3
# """
# TROV Unified Server — port 5000
# ────────────────────────────────
# PTZ control:
#  POST /api/ptz/move { "operation": "LEFT", "duration": 500 }
#  POST /api/ptz/stop
#  GET  /api/ptz/info

# Robot API:
#  GET  /api/maps
#  GET  /api/status
#  GET  /api/stack/status
#  POST /api/stack/start
#  POST /api/stack/stop
#  POST /api/stack/restart
#  POST /api/localization/restart { "map_name": "..." }
#  POST /api/navigation/restart
#  POST /api/odometry/restart

# Mapping API:
#  POST /api/mapping/start
#  POST /api/mapping/stop_and_save  { "map_name": "..." }
#  GET  /api/mapping/status

#  GET  /api/health
# """

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
MAPS_DIR      = "/data/trov_ws/install/trov/share/trov/maps"
SCRIPTS_DIR   = "/data/trov_ws"
TEMP_MAP_PATH = "/tmp/trov_map_temp"
ROS_ENV       = {**os.environ, "HOME": "/home/nvidia"}

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
# ROBOT API routes
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/maps', methods=['GET'])
def get_maps():
    if not os.path.isdir(MAPS_DIR):
        return jsonify({'error': f'Maps folder not found: {MAPS_DIR}'}), 500
    maps = sorted([
        f.replace('.yaml', '')
        for f in os.listdir(MAPS_DIR)
        if f.endswith('.yaml')
    ])
    if not maps:
        return jsonify({'error': 'No maps found'}), 404
    return jsonify({'maps': maps})


@app.route('/api/status', methods=['GET'])
def get_status():
    processes = {
        'localization': 'map_server',
        'navigation':   'bt_navigator',
        'odometry':     'icp_odometry',
        'rosbridge':    'rosbridge_websocket',
        'mediamtx':     'mediamtx',
        'mapping':      'slam_toolbox',
    }
    status = {}
    for name, keyword in processes.items():
        result = subprocess.run(['pgrep', '-f', keyword], capture_output=True)
        status[name] = 'running' if result.returncode == 0 else 'dead'
    return jsonify({'status': status})


@app.route('/api/stack/status', methods=['GET'])
def stack_status():
    result = subprocess.run(
        ['systemctl', 'is-active', 'trov.service'],
        capture_output=True, text=True
    )
    return jsonify({'trov_service': result.stdout.strip()})


@app.route('/api/stack/start', methods=['POST', 'OPTIONS'])
def stack_start():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    result = subprocess.run(
        ['systemctl', 'is-active', 'trov.service'],
        capture_output=True, text=True
    )
    if result.stdout.strip() == 'active':
        return jsonify({'status': 'already_running', 'message': 'trov.service is already active'})
    code, _, err = run_systemctl('start', 'trov.service')
    if code != 0:
        return jsonify({'error': f'Failed to start trov.service: {err}'}), 500
    return jsonify({'status': 'ok', 'message': 'trov.service started'})


@app.route('/api/stack/stop', methods=['POST', 'OPTIONS'])
def stack_stop():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    code, _, err = run_systemctl('stop', 'trov.service')
    if code != 0:
        return jsonify({'error': f'Failed to stop trov.service: {err}'}), 500
    return jsonify({'status': 'ok', 'message': 'trov.service stopped'})


@app.route('/api/stack/restart', methods=['POST', 'OPTIONS'])
def stack_restart():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    code, _, err = run_systemctl('restart', 'trov.service')
    if code != 0:
        return jsonify({'error': f'Failed to restart trov.service: {err}'}), 500
    return jsonify({'status': 'ok', 'message': 'trov.service restarted'})


@app.route('/api/localization/restart', methods=['POST', 'OPTIONS'])
def restart_localization():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    data     = request.get_json(force=True)
    map_name = data.get('map_name', '').strip()
    if not map_name:
        return jsonify({'error': 'map_name is required'}), 400
    map_path = os.path.join(MAPS_DIR, f'{map_name}.yaml')
    if not os.path.isfile(map_path):
        return jsonify({'error': f"Map '{map_name}' not found. Check /api/maps for available maps."}), 404
    pid, err = run_script('restart_localization.sh', [map_name])
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'status': 'ok', 'message': f'Localization restarting with map: {map_name}', 'pid': pid})


@app.route('/api/navigation/restart', methods=['POST', 'OPTIONS'])
def restart_navigation():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    pid, err = run_script('restart_navigation.sh')
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'status': 'ok', 'message': 'Navigation restarting', 'pid': pid})


@app.route('/api/odometry/restart', methods=['POST', 'OPTIONS'])
def restart_odometry():
    if request.method == 'OPTIONS':
        return _cors_preflight()
    pid, err = run_script('restart_odometry.sh')
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'status': 'ok', 'message': 'Odometry restarting', 'pid': pid})


# ══════════════════════════════════════════════════════════════════════════════
# MAPPING routes
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/mapping/start', methods=['POST', 'OPTIONS'])
def mapping_start():
    """Stop localization + navigation, then start slam_toolbox SLAM."""
    if request.method == 'OPTIONS':
        return _cors_preflight()
    pid, err = run_script('start_mapping.sh')
    if err:
        return jsonify({'error': err}), 500
    return jsonify({
        'status': 'ok',
        'message': 'Mapping started. Localization and navigation have been stopped.',
        'pid': pid
    })


@app.route('/api/mapping/stop_and_save', methods=['POST', 'OPTIONS'])
def mapping_stop_and_save():
    """Stop slam_toolbox, save map, and move it to maps dir — all in one call."""
    if request.method == 'OPTIONS':
        return _cors_preflight()

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
    dest_yaml = os.path.join(MAPS_DIR, f"{map_name}.yaml")
    if os.path.isfile(dest_yaml):
        return jsonify({
            'error': f"Map '{map_name}' already exists. Choose a different name."
        }), 409

    # ── Step 1: Stop slam_toolbox and save to temp ─────────────
    stop_result = subprocess.run(
        ['/bin/bash', os.path.join(SCRIPTS_DIR, 'stop_mapping.sh')],
        env=ROS_ENV,
        capture_output=True,
        text=True
    )
    if stop_result.returncode != 0:
        return jsonify({
            'error': f"Failed to stop mapping: {stop_result.stderr.strip()}"
        }), 500

    # ── Step 2: Move temp map to maps dir with given name ──
    save_result = subprocess.run(
        ['/bin/bash', os.path.join(SCRIPTS_DIR, 'save_map.sh'), map_name],
        env=ROS_ENV,
        capture_output=True,
        text=True
    )
    if save_result.returncode != 0:
        return jsonify({
            'error': f"Mapping stopped but save failed: {save_result.stderr.strip()}"
        }), 500

    return jsonify({
        'status': 'ok',
        'message': f"Mapping stopped and map saved as '{map_name}'.",
        'map_name': map_name
    })



@app.route('/api/mapping/status', methods=['GET'])
def mapping_status():
    """Check if gmapping is running and whether a temp map is waiting to be saved."""
    result          = subprocess.run(['pgrep', '-f', 'slam_toolbox'], capture_output=True)
    gmapping_running = result.returncode == 0

    temp_map_ready = (
        os.path.isfile(f"{TEMP_MAP_PATH}.yaml") and
        os.path.isfile(f"{TEMP_MAP_PATH}.pgm")
    )

    return jsonify({
        'gmapping_running': gmapping_running,
        'temp_map_ready':   temp_map_ready,
        'temp_map_path':    TEMP_MAP_PATH if temp_map_ready else None
    })


# ══════════════════════════════════════════════════════════════════════════════
# Health
# ══════════════════════════════════════════════════════════════════════════════

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok', 'device': DEVICE_ID})


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("\n🤖 TROV API Server")
    print("─" * 50)
    print(" PTZ control:")
    print("   POST /api/ptz/move { operation, duration }")
    print("   POST /api/ptz/stop")
    print("   GET  /api/ptz/info")
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