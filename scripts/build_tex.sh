#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$ROOT_DIR/docs"
TEX_FILE="$OUT_DIR/algorithms.tex"

if ! command -v pdflatex >/dev/null 2>&1; then
  echo "pdflatex not found. Please install TeX Live or MacTeX." >&2
  exit 1
fi

pdflatex -interaction=nonstopmode -halt-on-error -output-directory "$OUT_DIR" "$TEX_FILE" >/dev/null

echo "Built: $OUT_DIR/algorithms.pdf"
