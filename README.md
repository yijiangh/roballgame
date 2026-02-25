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

## Generate figures (matplotlib)
Figures are generated into `docs/figures/` and included by LaTeX.
```bash
./scripts/generate_figs.sh
```

## Install requirements
This project uses:
- Python (with Tkinter for the GUI)
- LaTeX (`pdflatex`) for the PDF
- Matplotlib for figure generation

### Option A: Homebrew (recommended on macOS)
```bash
brew install python@3.14 python-tk@3.14 python-matplotlib
brew install --cask mactex
```

### Option B: Virtualenv (Python only)
```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```
Note: this installs matplotlib only. Tkinter still needs to be available in your Python install.

## VS Code (LaTeX Workshop)
- Make sure `pdflatex` is on PATH (restart VS Code after installing TeX).
- Open `docs/algorithms.tex` and build with LaTeX Workshop.

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
