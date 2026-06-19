#!/usr/bin/env bash
# Regenerate every report figure (one consistent style) into the docs fig/ folder.
#
# Run from the mapf repo root:
#     bash viz/make_figures.sh
#
# Produces, in $FIG_DIR:
#     arena_trails.png        -- static trajectory view (from the SVG)
#     arena_anim.png          -- one animation frame (screenshot of the HTML)
#     operating_envelope.png  -- runtime-vs-agents plot
#
# Requirements (already used during development, all on macOS):
#   * Docker running (for the C++ exporter)
#   * python3 (standard library only)
#   * qlmanage + magick (ImageMagick) for SVG->PNG
#   * Google Chrome for the animation screenshot
set -euo pipefail

FIG_DIR="${FIG_DIR:-../../docs/fig}"
mkdir -p "$FIG_DIR" viz/data viz/out
FIG_DIR="$(cd "$FIG_DIR" && pwd)"
echo "Figures -> $FIG_DIR"

# 1) Solve an arena instance with PIBT and export it to JSON (inside Docker).
docker compose exec mapf zsh -lc 'cd /mapf && bazel run //solver:export_solution -- \
    --width 32 --height 20 --agents 40 --obstacles --solver pibt_solver \
    --seed 42 --out viz/data/arena.json'

# 2) Render the HTML animation and the trajectory SVG (host, stdlib only).
python3 viz/visualize.py viz/data/arena.json --out viz/out

# 3) Render the operating-envelope plot to SVG.
python3 viz/plot_envelope.py --out viz/out/operating_envelope.svg

# Helper: SVG -> trimmed PNG via Quick Look + ImageMagick.
svg2png() {  # $1 = svg, $2 = out png
    local tmp="/tmp/$(basename "$1").png"
    rm -f "$tmp"
    qlmanage -t -s 1500 -o /tmp "$1" >/dev/null 2>&1
    magick "$tmp" -fuzz 3% -trim +repage "$2"
}

svg2png viz/out/arena_trails.svg        "$FIG_DIR/arena_trails.png"
svg2png viz/out/operating_envelope.svg  "$FIG_DIR/operating_envelope.png"

# 4) Screenshot the HTML animation (an early frame) for the static figure.
CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
if [ -x "$CHROME" ]; then
    "$CHROME" --headless=new --disable-gpu --hide-scrollbars \
        --virtual-time-budget=3000 --window-size=1100,760 \
        --screenshot="$FIG_DIR/arena_anim.png" \
        "file://$(pwd)/viz/out/arena.html" >/dev/null 2>&1
else
    echo "WARN: Chrome not found; open viz/out/arena.html and screenshot it manually."
fi

echo "Done. Figures written to $FIG_DIR:"
ls -1 "$FIG_DIR"
