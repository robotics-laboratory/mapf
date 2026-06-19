#!/usr/bin/env python3
"""vizualizer

reads json produced by `solver:export_solution` and emits:
  * <name>.html        -- self-contained interactive animation (canvas, HUD,
                          play/pause, time slider, speed, trails toggle).
  * <name>_trails.svg  -- static "all trajectories" image for the report.

usage: python3 viz/visualize.py viz/data/arena.json --out viz/out
"""

import argparse
import json
import os


def make_colors(n):
    """Evenly spaced, visually distinct HSL colours."""
    return [f"hsl({int(i * 360 / max(n, 1)) % 360}, 72%, 52%)" for i in range(n)]


def cell_center(x, y, cell):
    return (x * cell + cell / 2.0, y * cell + cell / 2.0)


def gen_svg(data, colors, cell=24):
    w, h = data["width"], data["height"]
    W, H = w * cell, h * cell
    parts = []
    parts.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}" font-family="sans-serif">'
    )
    parts.append(f'<rect width="{W}" height="{H}" fill="#fbfbfd"/>')

    for x in range(w + 1):
        parts.append(f'<line x1="{x*cell}" y1="0" x2="{x*cell}" y2="{H}" stroke="#e7e7ee" stroke-width="1"/>')
    for y in range(h + 1):
        parts.append(f'<line x1="0" y1="{y*cell}" x2="{W}" y2="{y*cell}" stroke="#e7e7ee" stroke-width="1"/>')


    for ox, oy in data.get("obstacles", []):
        parts.append(f'<rect x="{ox*cell}" y="{oy*cell}" width="{cell}" height="{cell}" fill="#23232b"/>')

    for i, agent in enumerate(data["agents"]):
        path = agent["path"]
        if not path:
            continue
        color = colors[i % len(colors)]
        pts = " ".join(f"{cx:.1f},{cy:.1f}" for cx, cy in (cell_center(px, py, cell) for px, py in path))
        parts.append(
            f'<polyline points="{pts}" fill="none" stroke="{color}" '
            f'stroke-width="{cell*0.16:.1f}" stroke-opacity="0.65" '
            f'stroke-linejoin="round" stroke-linecap="round"/>'
        )

    for i, agent in enumerate(data["agents"]):
        color = colors[i % len(colors)]
        sx, sy = cell_center(*agent["start"], cell)
        gx, gy = agent["goal"]
        parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{cell*0.22:.1f}" fill="{color}"/>')
        parts.append(
            f'<rect x="{gx*cell + cell*0.28:.1f}" y="{gy*cell + cell*0.28:.1f}" '
            f'width="{cell*0.44:.1f}" height="{cell*0.44:.1f}" fill="none" '
            f'stroke="{color}" stroke-width="2"/>'
        )

    parts.append("</svg>")
    return "\n".join(parts)


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>__TITLE__</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #14141a; color: #e9e9f0; font-family: ui-monospace, monospace; }
  .wrap { max-width: 1100px; margin: 0 auto; padding: 16px; }
  .hud { display: flex; gap: 22px; flex-wrap: wrap; align-items: center;
         background: #1e1e27; border: 1px solid #2c2c38; border-radius: 8px;
         padding: 10px 14px; margin-bottom: 12px; font-size: 14px; }
  .hud b { color: #7ad0ff; }
  canvas { background: #fbfbfd; border-radius: 8px; width: 100%; height: auto; display: block; }
  .controls { display: flex; gap: 14px; align-items: center; margin-top: 12px; flex-wrap: wrap; }
  button { background: #2b6cf0; color: #fff; border: 0; border-radius: 6px;
           padding: 8px 16px; font: inherit; cursor: pointer; }
  button:hover { background: #3f7bf6; }
  input[type=range] { width: 360px; }
  label { font-size: 13px; opacity: 0.85; }
</style>
</head>
<body>
<div class="wrap">
  <div class="hud">
    <span>solved by <b>__SOLVER__</b></span>
    <span>agents: <b id="nag"></b></span>
    <span>map: <b>__WIDTH__&times;__HEIGHT__</b></span>
    <span>makespan: <b id="mk"></b></span>
    <span>time step: <b id="ts"></b></span>
    <span id="status"></span>
  </div>
  <canvas id="cv"></canvas>
  <div class="controls">
    <button id="play">Pause</button>
    <input type="range" id="seek" min="0" value="0" step="0.01"/>
    <label>speed <input type="range" id="speed" min="0.2" max="6" step="0.1" value="2"/></label>
    <label><input type="checkbox" id="trails" checked/> trails</label>
  </div>
</div>
<script>
const DATA = __DATA__;
const COLORS = __COLORS__;
const W = DATA.width, H = DATA.height;
const agents = DATA.agents;
const maxT = Math.max(1, ...agents.map(a => a.path.length - 1));

const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const DPR = window.devicePixelRatio || 1;
let cell = Math.max(8, Math.floor(960 / W));
cv.width = W * cell * DPR; cv.height = H * cell * DPR;
cv.style.height = (H * cell) + 'px';
ctx.scale(DPR, DPR);

document.getElementById('nag').textContent = agents.length;
document.getElementById('mk').textContent = (DATA.makespan != null && DATA.makespan < 0) ? '—' : (DATA.makespan != null ? DATA.makespan : maxT);
const st = document.getElementById('status');
if (DATA.solved === false) { st.innerHTML = '<b style="color:#ff7a7a">NO SOLUTION (deadlock)</b>'; }
else if (DATA.solved === true) { st.innerHTML = '<b style="color:#86d29a">SOLVED</b>'; }
const seek = document.getElementById('seek');
seek.max = maxT;

function posAt(agent, t) {
  const p = agent.path;
  if (p.length === 0) return null;
  const i = Math.min(Math.floor(t), p.length - 1);
  const j = Math.min(i + 1, p.length - 1);
  const f = t - i;
  return [p[i][0] + (p[j][0] - p[i][0]) * f, p[i][1] + (p[j][1] - p[i][1]) * f];
}

function draw(t) {
  ctx.clearRect(0, 0, cv.width, cv.height);
  // grid
  ctx.strokeStyle = '#e7e7ee'; ctx.lineWidth = 1;
  for (let x = 0; x <= W; x++) { ctx.beginPath(); ctx.moveTo(x*cell,0); ctx.lineTo(x*cell,H*cell); ctx.stroke(); }
  for (let y = 0; y <= H; y++) { ctx.beginPath(); ctx.moveTo(0,y*cell); ctx.lineTo(W*cell,y*cell); ctx.stroke(); }
  // obstacles
  ctx.fillStyle = '#23232b';
  for (const [ox, oy] of (DATA.obstacles || [])) ctx.fillRect(ox*cell, oy*cell, cell, cell);
  // trails
  if (document.getElementById('trails').checked) {
    for (let i = 0; i < agents.length; i++) {
      const p = agents[i].path; if (p.length < 2) continue;
      ctx.strokeStyle = COLORS[i % COLORS.length]; ctx.globalAlpha = 0.30;
      ctx.lineWidth = Math.max(1.5, cell*0.13); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      const upto = Math.min(Math.floor(t), p.length - 1);
      ctx.moveTo(p[0][0]*cell+cell/2, p[0][1]*cell+cell/2);
      for (let k = 1; k <= upto; k++) ctx.lineTo(p[k][0]*cell+cell/2, p[k][1]*cell+cell/2);
      ctx.stroke(); ctx.globalAlpha = 1;
    }
  }
  // goals
  for (let i = 0; i < agents.length; i++) {
    const [gx, gy] = agents[i].goal;
    ctx.strokeStyle = COLORS[i % COLORS.length]; ctx.lineWidth = 2;
    ctx.strokeRect(gx*cell+cell*0.28, gy*cell+cell*0.28, cell*0.44, cell*0.44);
  }
  // agents
  for (let i = 0; i < agents.length; i++) {
    const pos = posAt(agents[i], t); if (!pos) continue;
    ctx.fillStyle = COLORS[i % COLORS.length];
    ctx.beginPath();
    ctx.arc(pos[0]*cell+cell/2, pos[1]*cell+cell/2, cell*0.32, 0, 2*Math.PI);
    ctx.fill();
    ctx.strokeStyle = 'rgba(0,0,0,0.35)'; ctx.lineWidth = 1; ctx.stroke();
  }
  document.getElementById('ts').textContent = Math.min(Math.round(t), maxT) + ' / ' + maxT;
}

let t = 0, playing = true, last = performance.now();
function loop(now) {
  const dt = (now - last) / 1000; last = now;
  if (playing) {
    t += dt * parseFloat(document.getElementById('speed').value);
    if (t >= maxT) { t = maxT; playing = false; document.getElementById('play').textContent = 'Replay'; }
    seek.value = t;
  }
  draw(t);
  requestAnimationFrame(loop);
}
document.getElementById('play').onclick = () => {
  if (t >= maxT) { t = 0; }
  playing = !playing;
  document.getElementById('play').textContent = playing ? 'Pause' : 'Play';
};
seek.oninput = () => { t = parseFloat(seek.value); playing = false; document.getElementById('play').textContent = 'Play'; };
requestAnimationFrame(loop);
</script>
</body>
</html>
"""


def gen_html(data, colors):
    return (
        HTML_TEMPLATE
        .replace("__TITLE__", f"MAPF {data['solver']} {data['width']}x{data['height']}")
        .replace("__SOLVER__", str(data["solver"]))
        .replace("__WIDTH__", str(data["width"]))
        .replace("__HEIGHT__", str(data["height"]))
        .replace("__COMP__", f"{data.get('comp_time_ms', 0):.1f}")
        .replace("__DATA__", json.dumps(data))
        .replace("__COLORS__", json.dumps(colors))
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="JSON file from export_solution")
    ap.add_argument("--out", default="viz/out", help="output directory")
    args = ap.parse_args()

    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)

    colors = make_colors(len(data["agents"]))
    os.makedirs(args.out, exist_ok=True)
    name = os.path.splitext(os.path.basename(args.input))[0]

    html_path = os.path.join(args.out, name + ".html")
    svg_path = os.path.join(args.out, name + "_trails.svg")
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(gen_html(data, colors))
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(gen_svg(data, colors))

    print(f"wrote {html_path}")
    print(f"wrote {svg_path}")
    print(f"  {len(data['agents'])} agents, makespan {data.get('makespan')}, "
          f"comp_time {data.get('comp_time_ms')} ms")


if __name__ == "__main__":
    main()
