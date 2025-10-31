#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_FILEPATH="$SCRIPT_DIR/arm.urdf"

rm -rf "$SCRIPT_DIR/assets"
rm $URDF_FILEPATH

onshape-to-robot "$SCRIPT_DIR/onshape_to_robot_config.json"

sed -i 's|package://|package://hardware/urdf/|g' "$URDF_FILEPATH"
