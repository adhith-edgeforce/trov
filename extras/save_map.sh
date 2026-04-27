#!/bin/bash
# ============================================================
# save_map.sh
# Moves the temp map (saved by stop_mapping.sh) into the
# maps directory with the name provided by the user.
#
# Usage:
#   ./save_map.sh <map_name>
#   ./save_map.sh warehouse2
# ============================================================

MAPS_DIR="/data/trov_ws/install/trov/share/trov/maps"
TEMP_MAP_PATH="/tmp/trov_map_temp"
MAP_NAME="${1:-}"

# ── Validate map name ──────────────────────────────────────
if [ -z "$MAP_NAME" ]; then
    echo "ERROR: No map name provided"
    echo "Usage: ./save_map.sh <map_name>"
    exit 1
fi

# Only allow alphanumeric, underscore, hyphen — no spaces or slashes
if [[ ! "$MAP_NAME" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo "ERROR: Invalid map name '$MAP_NAME'. Use only letters, numbers, underscores, hyphens."
    exit 1
fi

# ── Check temp map exists ──────────────────────────────────
if [ ! -f "${TEMP_MAP_PATH}.yaml" ] || [ ! -f "${TEMP_MAP_PATH}.pgm" ]; then
    echo "ERROR: Temp map not found at $TEMP_MAP_PATH (.yaml + .pgm)"
    echo "Did you run stop_mapping.sh first?"
    exit 1
fi

# ── Check maps directory exists ────────────────────────────
if [ ! -d "$MAPS_DIR" ]; then
    echo "ERROR: Maps directory not found: $MAPS_DIR"
    exit 1
fi

# ── Check for name collision ───────────────────────────────
DEST_YAML="$MAPS_DIR/$MAP_NAME.yaml"
DEST_PGM="$MAPS_DIR/$MAP_NAME.pgm"

if [ -f "$DEST_YAML" ]; then
    echo "ERROR: Map '$MAP_NAME' already exists. Choose a different name."
    exit 1
fi

# ── Copy map files ─────────────────────────────────────────
cp "${TEMP_MAP_PATH}.yaml" "$DEST_YAML"
cp "${TEMP_MAP_PATH}.pgm"  "$DEST_PGM"

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to copy map files to $MAPS_DIR"
    exit 1
fi

# ── Update the image path inside the yaml ─────────────────
# map_saver writes the absolute temp path — fix it to point to the new location
sed -i "s|^image:.*|image: ${MAP_NAME}.pgm|" "$DEST_YAML"

# ── Clean up temp files ────────────────────────────────────
rm -f "${TEMP_MAP_PATH}.yaml" "${TEMP_MAP_PATH}.pgm"

echo "OK: Map saved as '$MAP_NAME'"
echo "   YAML: $DEST_YAML"
echo "   PGM:  $DEST_PGM"
exit 0