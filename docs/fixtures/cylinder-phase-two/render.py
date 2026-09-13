"""Render source ownership from an exported mesh, without geometry reconstruction.
Usage: python3 render.py /path/to/shared.mesh /path/to/ownership.svg
Then render the SVG with a browser for PNG output.
"""
import math
import sys
from pathlib import Path

vertices, triangles = [], []
for line in Path(sys.argv[1]).read_text().splitlines():
    row = line.split()
    if row and row[0] == "v":
        vertices.append(tuple(map(float, row[1:])))
    elif row and row[0] == "t":
        triangles.append(tuple(map(int, row[1:])))

palette = ["#e4ab54", "#d18736", "#e3827c", "#bb605b", "#64a8a3", "#377b83", "#9a8fc4", "#7968a6"]
labels = ["0 wall / endpoint -", "1 wall / endpoint +", "2 disk / endpoint +", "3 disk / endpoint -", "4 upper tool rim", "5 lower tool rim", "6 left wall contact", "7 right wall contact"]
# Orthographic view: preserve shape and draw actual facets in depth order.
def project(p):
    x, y, z = p[0] - 3, p[1], p[2]
    a, b = 0.55, 0.5
    horizontal = math.cos(a)*x - math.sin(a)*y
    depth = math.sin(a)*x + math.cos(a)*y
    vertical = math.cos(b)*z - math.sin(b)*depth
    depth = math.sin(b)*z + math.cos(b)*depth
    return 480 + 195*horizontal, 365 - 195*vertical, depth

points = list(map(project, vertices))
triangles.sort(key=lambda t: sum(points[v][2] for v in t[:3]))
svg = ["<svg xmlns='http://www.w3.org/2000/svg' width='960' height='760' viewBox='0 0 960 760'>",
       "<rect width='960' height='760' fill='#fafaf8'/>",
       "<g font-family='sans-serif' fill='#243535'><text x='30' y='35' font-size='23'>Phase 2 cylinder: shared source domains</text>",
       "<text x='30' y='62' font-size='15'>Analytic support IDs; all faces shown are actual exported facets</text></g>"]
for t in triangles:
    ps = " ".join(f"{points[v][0]:.3f},{points[v][1]:.3f}" for v in t[:3])
    color = palette[t[3]]
    svg.append(f"<polygon points='{ps}' fill='{color}' stroke='{color}' stroke-width='0.25'/>")
for i, (color, label) in enumerate(zip(palette, labels)):
    x, y = 30 + i%4*232, 685 + i//4*32
    svg.append(f"<rect x='{x}' y='{y-13}' width='16' height='16' rx='3' fill='{color}'/>")
    svg.append(f"<text x='{x+24}' y='{y}' font-family='sans-serif' font-size='13' fill='#243535'>{label}</text>")
svg.append("</svg>")
Path(sys.argv[2]).write_text("\n".join(svg))
