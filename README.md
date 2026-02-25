# roballgame mockup

This repo contains a small 2D collision-aware joystick mockup and a LaTeX note describing the four velocity-shaping models.

## Run the mockup
```bash
python3 src/mockup.py
```

Controls:
- Arrows or WASD: move
- 1-4: switch model
- R: randomize circle obstacles
- `[` / `]`: adjust `d_slow` (start slowing distance)
- `-` / `=`: adjust `d_stop` (contact distance)
- `,` / `.`: adjust repulsion strength

## LaTeX doc
The math is in `docs/algorithms.tex`.

Build (requires `pdflatex` on PATH):
```bash
./scripts/build_tex.sh
```

## Notes
- The mockup uses only circles and boundary walls for fast analytic distance checks.
- Rectangular obstacles and line segments are included to approximate tables/walls/rails.
- Logs are written to `logs/vel_dist.csv` (distance vs. speed over time).
- "Moko" integration is a placeholder and not implemented yet.
