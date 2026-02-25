#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MPL_DIR="${MPLCONFIGDIR:-/var/folders/sz/1j1qn1v96gs82h7ywn4p48q40000gn/T/matplotlib}"

MPLCONFIGDIR="$MPL_DIR" MPLBACKEND=Agg /opt/homebrew/bin/python3 "$ROOT_DIR/scripts/generate_figs.py"
