#!/bin/bash
export MODEL_CACHE_DIR="/home/nvidia/.cache/roboflow"
export PULSE_SERVER=unix:/run/user/1000/pulse/native

cd /home/nvidia/Downloads/surveillance_AI
source env/bin/activate
exec python ai_publish.py
