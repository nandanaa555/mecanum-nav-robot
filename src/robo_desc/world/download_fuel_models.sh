#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────
# Pre-download all Fuel models used in warehouse_world.sdf
# Run ONCE before launching Gazebo so it loads instantly.
# Usage:  chmod +x download_fuel_models.sh && ./download_fuel_models.sh
# ─────────────────────────────────────────────────────────────────

set -e

FUEL_DIR="$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models"
mkdir -p "$FUEL_DIR"

MODELS=(
  "aws_robomaker_warehouse_WallB_01"
  "aws_robomaker_warehouse_ShelfE_01"
  "aws_robomaker_warehouse_ShelfD_01"
  "aws_robomaker_warehouse_ShelfF_01"
  "Cardboard Box"
  "Pallet"
)

BASE="https://fuel.gazebosim.org/1.0/OpenRobotics/models"

for MODEL in "${MODELS[@]}"; do
  ENCODED=$(python3 -c "import urllib.parse; print(urllib.parse.quote('$MODEL'))")
  echo "⬇  Downloading: $MODEL"
  gz fuel download -u "${BASE}/${ENCODED}" 2>/dev/null || \
    gz fuel download --url "${BASE}/${ENCODED}" 2>/dev/null || \
    echo "  ↳ Will auto-download on first launch"
done

echo ""
echo "✅ Done. Launch with:"
echo "   gz sim -r warehouse_world.sdf"
