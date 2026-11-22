#!/usr/bin/env python3
"""The Utah teapot's whole faces by an independent quadrature, a third reader beside OCCT and the
bench's measurer (#993 stage 1): numpy alone, from each file's own control points, nothing of the
kernel's, OCCT's or the measurer's.

    python3 -I scripts/step-corpus/teapot.py           # each file's faces, and the quad tables

brep-bench's `integration/utah-teapot` is the union of four solids made from Newell's 32 bicubic
patches (`cases/integration/utah-teapot.md`); the rim, the bottom, the knob and the lid rings are
cut by nothing, so each of their 16 patches is a face over its whole domain [0, 1]². Each
ADVANCED_FACE on a bicubic B-spline surface of each writer's file is matched to the case's patch
by its control net (to 1e-12, up to the net's order), and its area is the 40-point Gauss–Legendre
rule over 16 × 16 cells of |S_u × S_v| (8 × 8 cells agree to 1e-16: converged). The faces of the
four whole rings go to `oracle/<path>.quad.faces.tsv` (the measurer's columns), which the gate
and `table.py` take as a third reader. The case's area (52.2260167859, "no closed form: cad-kernel,
OCCT and cadrs agree to 4e-9; the mean of their values") is 3.7e-4 under its whole faces': all
three of those kernels, the bench's measurer and OCCT read the knob's four patches low (each
0.294363892581 by this rule; the measurer 0.294270388, OCCT 0.294041293), a pole at u = 0 each.
The 18 trimmed faces (the body cut by the spout and the handle, the spout's and the handle's
patches, the two planes) are not integrated here: that needs the trimming curves."""
import json
import os
import sys
from math import comb

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import check  # noqa: E402
import stepscan  # noqa: E402

CORPUS = os.environ.get("STEP_CORPUS", "/home/user/step-corpus")
WHOLE = ("rim", "bottom", "knob", "lid")
def bern(t, d=0):
    if d == 0: return np.stack([comb(3, i)*t**i*(1-t)**(3-i) for i in range(4)], -1)
    b2 = np.stack([comb(2, i)*t**i*(1-t)**(2-i) for i in range(3)], -1); z = np.zeros_like(t)[..., None]
    return 3*(np.concatenate([z, b2], -1) - np.concatenate([b2, z], -1))
def area(P, n=40, cells=16, du=1.0, dv=1.0):
    x, w = np.polynomial.legendre.leggauss(n); e = np.linspace(0, 1, cells+1)
    us = np.concatenate([(a+b)/2+(b-a)/2*x for a, b in zip(e, e[1:])]); ws = np.concatenate([(b-a)/2*w for a, b in zip(e, e[1:])])
    B, D = bern(us), bern(us, 1)
    Su = np.einsum('ui,vj,ijk->uvk', D, B, P); Sv = np.einsum('ui,vj,ijk->uvk', B, D, P)
    return np.einsum('u,v,uv->', ws, ws, np.linalg.norm(np.cross(Su, Sv), axis=-1))
def ring(prof, exc=None):
    P = np.zeros((4, 4, 3))
    for i, (r, z) in enumerate(prof): P[i] = [(r, 0, z), (r, -0.56*r, z), (0.56*r, -r, z), (0, -r, z)]
    for i, row in (exc or {}).items(): P[i] = row
    return P
def rot(P, k):
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k]; R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]); return P @ R.T
rings = {'rim': [(1.4, 2.4), (1.3375, 2.53125), (1.4375, 2.53125), (1.5, 2.4)], 'upper body': [(1.5, 2.4), (1.75, 1.875), (2, 1.35), (2, 0.9)],
         'lower body': [(2, 0.9), (2, 0.45), (1.5, 0.225), (1.5, 0.15)], 'bottom': [(1.5, 0.15), (1.5, 0.075), (1.425, 0), (0, 0)],
         'knob': [(0, 3.15), (0.8, 3.15), (0, 2.85), (0.2, 2.7)], 'lid': [(0.2, 2.7), (0.4, 2.55), (1.3, 2.55), (1.3, 2.4)]}
knob1 = {1: [(0.8, 0, 3.15), (0.8, -0.45, 3.15), (0.45, -0.8, 3.15), (0, -0.8, 3.15)]}
patches = {}
for n, prof in rings.items():
    P = ring(prof, knob1 if n == 'knob' else None)
    for k in range(4): patches[f'{n} {k}'] = rot(P, k)
def key(P):  # a net up to its rows' and columns' order and transposition
    return [P, P[::-1], P[:, ::-1], P[::-1, ::-1], P.transpose(1, 0, 2), P.transpose(1, 0, 2)[::-1], P.transpose(1, 0, 2)[:, ::-1], P.transpose(1, 0, 2)[::-1, ::-1]]
def faces(path):
    raw = open(path, 'rb').read().decode('latin-1'); ents = {}
    for st in stepscan.statements(raw):
        m = stepscan.ENTITY.match(st.strip())
        if m: ents[int(m.group(1))] = st.strip()[m.end():]
    out = []
    for e, rhs in ents.items():
        if not rhs.startswith('ADVANCED_FACE'): continue
        s = check.args(rhs)[2]; srhs = ents[s]
        if 'B_SPLINE_SURFACE' not in srhs: continue
        a = check.args(srhs) if srhs.startswith('B_SPLINE') else None
        assert a is not None and a[1] == 3.0 and a[2] == 3.0 and a[8] == [4.0, 4.0] and a[9] == [4.0, 4.0], (path, e, srhs[:100])
        P = np.array([[check.args(ents[p])[1] for p in row] for row in a[3]], float)
        u0, u1 = a[10]; v0, v1 = a[11]
        name = next((n for n, Q in patches.items() if any(np.allclose(Q, X, atol=1e-12) for X in key(P))), '-')
        out.append((e, name, float(area(P)), (u1-u0, v1-v0)))
    return sorted(out)
def main():
    root = os.path.join(CORPUS, "brep-bench")
    for k in sorted(os.listdir(root)):
        path = os.path.join(root, k, "integration", "utah-teapot.step")
        if not os.path.exists(path):
            continue
        rel = os.path.relpath(path, CORPUS)
        sha = stepscan.data_sha256(open(path, "rb").read())
        rows = faces(path)
        whole = [(e, a) for e, n, a, _ in rows if n.split(" ")[0] in WHOLE]
        for e, n, a, _ in rows:
            print(f"{rel}\t#{e}\t{n}\t{a:.15g}")
        out = os.path.join(CORPUS, "oracle", rel + ".quad.faces.tsv")
        with open(out, "w") as fh:
            fh.write(f"#\t{sha}\nentity\tarea\n" + "".join(f"{e}\t{a!r}\n" for e, a in whole))
        print(f"{out}: {len(whole)} whole faces, {sum(a for _, a in whole):.12g} mm²")


if __name__ == "__main__":
    main()
