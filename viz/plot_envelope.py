#!/usr/bin/env python3

#usage: python3 viz/plot_envelope.py --out viz/out/operating_envelope.svg


import argparse
import math
import os


ASTAR = [(2, 1.04), (3, 31.85), (4, 410.27)]
PIBT = [(4, 0.224), (16, 1.305), (32, 2.642), (50, 10.874), (100, 25.558), (200, 138.711)]

W, H = 760, 460
L, R, T, B = 78, 24, 24, 56 
PW, PH = W - L - R, H - T - B 
X_MAX = 210.0
LOG_MIN, LOG_MAX = -1.0, 3.0


def x_px(agents):
    return L + (agents / X_MAX) * PW


def y_px(ms):
    lv = max(LOG_MIN, min(LOG_MAX, math.log10(ms)))
    return T + (LOG_MAX - lv) / (LOG_MAX - LOG_MIN) * PH


def polyline(points, color, dash=False):
    pts = " ".join(f"{x_px(a):.1f},{y_px(t):.1f}" for a, t in points)
    d = ' stroke-dasharray="6 5"' if dash else ""
    return f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.5"{d}/>'


def markers(points, color):
    return "\n".join(
        f'<circle cx="{x_px(a):.1f}" cy="{y_px(t):.1f}" r="4.5" fill="{color}" '
        f'stroke="white" stroke-width="1"/>'
        for a, t in points
    )


def build_svg():
    s = []
    s.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}" font-family="sans-serif" font-size="13">'
    )
    s.append(f'<rect width="{W}" height="{H}" fill="white"/>')

    for e in range(int(LOG_MIN), int(LOG_MAX) + 1):
        y = y_px(10 ** e)
        s.append(f'<line x1="{L}" y1="{y:.1f}" x2="{L+PW}" y2="{y:.1f}" stroke="#e8e8ee"/>')
        label = {-1: "0.1", 0: "1", 1: "10", 2: "100", 3: "1000"}[e]
        s.append(f'<text x="{L-10}" y="{y+4:.1f}" text-anchor="end" fill="#555">{label}</text>')


    for a in [0, 50, 100, 150, 200]:
        x = x_px(a)
        s.append(f'<line x1="{x:.1f}" y1="{T+PH}" x2="{x:.1f}" y2="{T+PH+5}" stroke="#888"/>')
        s.append(f'<text x="{x:.1f}" y="{T+PH+20}" text-anchor="middle" fill="#555">{a}</text>')
        
    s.append(f'<line x1="{L}" y1="{T}" x2="{L}" y2="{T+PH}" stroke="#444"/>')
    s.append(f'<line x1="{L}" y1="{T+PH}" x2="{L+PW}" y2="{T+PH}" stroke="#444"/>')


    s.append(f'<text x="{L+PW/2:.0f}" y="{H-12}" text-anchor="middle" fill="#333">'
             f'number of agents</text>')
    s.append(f'<text x="16" y="{T+PH/2:.0f}" text-anchor="middle" fill="#333" '
             f'transform="rotate(-90 16 {T+PH/2:.0f})">runtime (ms, log scale)</text>')

    astar_color, pibt_color = "#d1495b", "#2b6cf0"
    s.append(polyline(ASTAR, astar_color))
    s.append(markers(ASTAR, astar_color))
    s.append(polyline(PIBT, pibt_color))
    s.append(markers(PIBT, pibt_color))

    s.append(f'<text x="{x_px(14):.1f}" y="{y_px(430):.1f}" fill="{astar_color}">A* becomes '
             f'intractable beyond ~4 agents</text>')


    lx, ly = L + PW - 132, T + 16
    s.append(f'<rect x="{lx-10}" y="{ly-14}" width="150" height="46" fill="white" '
             f'stroke="#ddd" rx="4"/>')
    s.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+24}" y2="{ly}" stroke="{astar_color}" '
             f'stroke-width="2.5"/><text x="{lx+32}" y="{ly+4}" fill="#333">A* (optimal)</text>')
    s.append(f'<line x1="{lx}" y1="{ly+20}" x2="{lx+24}" y2="{ly+20}" stroke="{pibt_color}" '
             f'stroke-width="2.5"/><text x="{lx+32}" y="{ly+24}" fill="#333">PIBT</text>')

    s.append("</svg>")
    return "\n".join(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="viz/out/operating_envelope.svg")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(build_svg())
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
