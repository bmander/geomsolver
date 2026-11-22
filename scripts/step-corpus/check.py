#!/usr/bin/env python3
"""Whether a STEP file's solid is a solid's boundary by ISO 10303-42, checked without the kernel's
reader: the evidence behind a row of the corpus's account (`step_corpus/account.tsv`) that says a
file is defective (#993 stage 1).

    python3 -I scripts/step-corpus/check.py FILE.step...           # a report a file
    python3 -I scripts/step-corpus/check.py --text FILE.step...    # the text's checks alone
    python3 -I scripts/step-corpus/check.py --json FILE.step...    # a JSON line a file

Two readings, each apart from the kernel:

* **the text's own** (standard library only, `stepscan`'s parse): each EDGE_LOOP of the solids'
  faces walked by vertex identity (`loop breaks`: an oriented edge ending at a VERTEX_POINT the
  next does not start at, with both vertices' coordinates and the distance between them); each
  closed shell's edges counted by the direction their uses run, `ORIENTED_EDGE.orientation` as
  its bound's orientation takes it (`uses`: ISO 10303-42 has a closed shell use each edge twice,
  once each way, as a manifold's boundary does); each LINE whose VECTOR has no magnitude (a
  point: the vertices of an edge on it, which EDGE_CURVE says lie on its curve, do not); a real
  written as `NaN` or `Inf` (no real of Part 21);
* **OCCT's**, healing off as `oracle.py` reads (`read.step.sequence` naming none):
  `BRepCheck_Analyzer`'s statuses for each solid, shell, face, wire, edge and vertex that is not
  `NoError`, on its own and in each context it is checked in, each named by the STEP entity OCCT
  records it was translated from (`EntityFromShapeResult`); the largest edge and vertex
  tolerance OCCT's translation set (it raises a tolerance to cover what it finds, so a tolerance
  far past the file's stated uncertainty is the file's edges standing off their faces), with
  the entity, against the uncertainty and against `LOOSE` (the reader's `sound::LOOSE`, 1e-3) of
  the solid's size (its bounding box's diagonal).

OCCT's translation itself still mends what it mends (a vertex's tolerance raised, a wire's order,
pcurves computed): a file OCCT reads valid can still break the standard, and the text's checks
say where. "OCCT heals it silently" is evidence the file is defective, not that it is sound.
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import stepscan  # noqa: E402

LOOSE = 1e-3
NUMBER = re.compile(r"[-+]?(\d+\.?\d*|\.\d+)([EeDd][-+]?\d+)?")


def args(rhs):
    """A simple entity's parameters, nested lists of `#n` (int), numbers (float), enums
    (`.T.`), strings and `$`/`*` as written."""
    s = rhs.strip()
    i = s.find("(")
    out, stack, tok = [], [], ""

    def flush():
        nonlocal tok
        t = tok.strip()
        tok = ""
        if not t:
            return
        cur = stack[-1]
        if t.startswith("#"):
            cur.append(int(t[1:]))
        elif NUMBER.fullmatch(t):
            cur.append(float(t.replace("D", "E").replace("d", "e")))
        else:
            cur.append(t)
    stack.append(out)
    for c in s[i + 1:]:
        if c == "(":
            new = []
            if tok.strip():
                # a typed parameter, LENGTH_MEASURE(1.): its value
                tok = ""
            stack[-1].append(new)
            stack.append(new)
        elif c == ")":
            flush()
            if len(stack) == 1:
                break
            stack.pop()
        elif c == ",":
            flush()
        else:
            tok += c
    return out


def text_checks(path):
    s = stepscan.scan(path)
    raw = open(path, "rb").read().decode("latin-1")
    ents = {}
    for st in stepscan.statements(raw):
        m = stepscan.ENTITY.match(st.strip())
        if m:
            rhs = st.strip()[m.end():]
            ents[int(m.group(1))] = (stepscan.kinds_of(rhs), rhs)
    kind = lambda e: ents[e][0][0] if e in ents and ents[e][0] else "?"  # noqa: E731
    a = lambda e: args(ents[e][1])  # noqa: E731

    def point(v):
        try:
            p = a(v)[1]  # VERTEX_POINT('', #point)
            return [float(x) for x in a(p)[1]]
        except (IndexError, TypeError, ValueError, KeyError):
            return None

    def under(root):
        seen, stack = set(), [root]
        while stack:
            e = stack.pop()
            if e in seen or e not in ents:
                continue
            seen.add(e)
            stack.extend(int(r) for r in stepscan.REF.findall(ents[e][1]))
        return seen
    out = {"path": path, "uncertainty": s["uncertainty"], "units": s["units"], "breaks": [],
           "uses": [], "pointlines": [], "nan": bool(re.search(r"(?<![A-Za-z_])(NaN|Inf)\b", raw))}
    for solid in s["solids"]:
        ids = under(solid["entity"])
        shells = [e for e in ids if kind(e) in ("CLOSED_SHELL", "OPEN_SHELL")]
        for sh in sorted(shells):
            count = {}
            faces = [f for f in a(sh)[1] if isinstance(f, int)]
            for f in faces:
                if kind(f) not in ("ADVANCED_FACE", "FACE_SURFACE"):
                    continue
                for b in a(f)[1]:
                    if kind(b) not in ("FACE_BOUND", "FACE_OUTER_BOUND"):
                        continue
                    bound = a(b)
                    loop, bsense = bound[1], bound[2] == ".T."
                    if kind(loop) != "EDGE_LOOP":
                        continue
                    oes = [x for x in a(loop)[1] if isinstance(x, int)]
                    ends = []
                    for oe in oes:
                        o = a(oe)
                        edge, sense = o[3], o[4] == ".T."
                        ec = a(edge)
                        v0, v1 = ec[1], ec[2]
                        ends.append((oe, edge, v0 if sense else v1, v1 if sense else v0))
                        c = count.setdefault(edge, [0, 0])
                        c[0 if sense == bsense else 1] += 1
                    for (oe, e, _, end), (oe2, e2, start, _) in zip(ends, ends[1:] + ends[:1]):
                        if end != start:
                            p, q = point(end), point(start)
                            d = math.dist(p, q) if p and q else float("nan")
                            out["breaks"].append({"face": f, "loop": loop, "edge": e, "next": e2,
                                                  "ends": end, "starts": start, "at": p,
                                                  "next_at": q, "apart": d})
            if kind(sh) == "CLOSED_SHELL":
                for e, (al, ag) in sorted(count.items()):
                    if (al, ag) != (1, 1):
                        out["uses"].append({"shell": sh, "edge": e, "along": al, "against": ag,
                                            "at": point(a(e)[1])})
        for e in sorted(ids):
            if kind(e) != "LINE":
                continue
            try:
                vec = a(e)[2]
                mag = float(a(vec)[2])
            except (IndexError, TypeError, ValueError, KeyError):
                continue
            if mag == 0.0:
                on = [x for x in ids if kind(x) == "EDGE_CURVE" and a(x)[3] == e]
                spans = []
                for x in on:
                    p, q = point(a(x)[1]), point(a(x)[2])
                    spans.append({"edge": x, "apart": math.dist(p, q) if p and q else None})
                out["pointlines"].append({"line": e, "vector": vec, "edges": spans})
    return out


def occt_checks(path):
    from OCP.BRepCheck import BRepCheck_Analyzer, BRepCheck_NoError
    from OCP.BRep import BRep_Tool
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.TopAbs import (TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_VERTEX,
                            TopAbs_WIRE)
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedMapOfShape
    from OCP.TopoDS import TopoDS
    import oracle
    shape, r = oracle.read(path, healing=False, reader=True)
    tr, model = r.WS().TransferReader(), r.StepModel()
    labels = list(stepscan.entities(path))
    keep, number = [], {}
    if model.NbEntities() == len(labels):
        for n in range(1, model.NbEntities() + 1):
            v = model.Value(n)
            keep.append(v)
            number[id(v)] = labels[n - 1]

    def name(sh):
        try:
            e = tr.EntityFromShapeResult(sh, 1)
        except Exception:  # noqa: BLE001
            return 0
        return number.get(id(e), 0) if e is not None else 0
    an = BRepCheck_Analyzer(shape)
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, False)
    x0, y0, z0, x1, y1, z1 = box.Get()
    size = math.dist((x0, y0, z0), (x1, y1, z1))
    out = {"valid": bool(an.IsValid()), "size": size, "statuses": []}
    words = {"SOLID": TopAbs_SOLID, "SHELL": TopAbs_SHELL, "FACE": TopAbs_FACE,
             "WIRE": TopAbs_WIRE, "EDGE": TopAbs_EDGE, "VERTEX": TopAbs_VERTEX}
    shapes = {}
    for word, k in words.items():
        m = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape, k, m)
        shapes[word] = [m.FindKey(i) for i in range(1, m.Extent() + 1)]
    for word, ss in shapes.items():
        for sh in ss:
            res = an.Result(sh)
            if res is None:
                continue
            bad = [str(st).split(".")[-1] for st in res.Status() if st != BRepCheck_NoError]
            ctx = []
            res.InitContextIterator()
            while res.MoreShapeInContext():
                c = res.ContextualShape()
                st = [str(x).split(".")[-1] for x in res.StatusOnShape(c) if x != BRepCheck_NoError]
                if st:
                    ctx.append({"in": c.ShapeType().name.replace("TopAbs_", ""),
                                "entity": name(c), "status": st})
                res.NextShapeInContext()
            if bad or ctx:
                out["statuses"].append({"shape": word, "entity": name(sh), "status": bad,
                                        "context": ctx})
    te = max(((BRep_Tool.Tolerance_s(TopoDS.Edge_s(e)), name(e)) for e in shapes["EDGE"]),
             default=(0.0, 0))
    tv = max(((BRep_Tool.Tolerance_s(TopoDS.Vertex_s(v)), name(v)) for v in shapes["VERTEX"]),
             default=(0.0, 0))
    out.update(tol_edge=te[0], tol_edge_entity=te[1], tol_vertex=tv[0], tol_vertex_entity=tv[1],
               loose=te[0] / size if size > 0 else float("inf"))
    if len(shapes["EDGE"]) <= OFF_AT_MOST:
        out["off"] = standing_off(shape, shapes["EDGE"], name)
    return out


OFF_AT_MOST = 3000


def standing_off(shape, edges, name):
    """How far each edge stands off its faces and its vertices, measured on OCCT's reading of the
    file's own curves and surfaces (no tolerance of OCCT's in it): `CHECKS` points of the edge's
    curve in space each projected onto every face's surface the edge bounds, and each vertex onto
    the edge's curve. The three edges farthest off, with their entities."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GeomAPI import GeomAPI_ProjectPointOnCurve, GeomAPI_ProjectPointOnSurf
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
    from OCP.TopoDS import TopoDS
    anc = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, anc)
    worst = []
    for e in edges:
        edge = TopoDS.Edge_s(e)
        if BRep_Tool.Degenerated_s(edge):
            continue
        try:
            c = BRepAdaptor_Curve(edge)
        except Exception:  # noqa: BLE001
            continue
        t0, t1 = c.FirstParameter(), c.LastParameter()
        pts = [c.Value(t0 + (t1 - t0) * k / (CHECKS - 1)) for k in range(CHECKS)]
        face_off = 0.0
        i = anc.FindIndex(edge)
        faces = list(anc.FindFromIndex(i)) if i else []
        for f in faces:
            s = BRep_Tool.Surface_s(TopoDS.Face_s(f))
            for p in pts:
                pr = GeomAPI_ProjectPointOnSurf(p, s)
                if pr.NbPoints() > 0:
                    face_off = max(face_off, pr.LowerDistance())
        vert_off = 0.0
        curve = BRep_Tool.Curve_s(edge, 0.0, 0.0)
        if curve is not None:
            ends = (c.Value(t0), c.Value(t1))
            for v in (TopExp.FirstVertex_s(edge), TopExp.LastVertex_s(edge)):
                p = BRep_Tool.Pnt_s(v)
                # the nearest of its feet and the curve's two ends (a foot is a square one only)
                pr = GeomAPI_ProjectPointOnCurve(p, curve)
                d = min([p.Distance(q) for q in ends] + ([pr.LowerDistance()] if pr.NbPoints() else []))
                vert_off = max(vert_off, d)
        worst.append((max(face_off, vert_off), face_off, vert_off, name(edge)))
    worst.sort(reverse=True)
    return [{"edge": w[3], "faces": w[1], "vertices": w[2]} for w in worst[:3]]


CHECKS = 23


def report(path, text_only=False):
    t = text_checks(path)
    if not text_only:
        try:
            t["occt"] = occt_checks(path)
        except Exception as e:  # noqa: BLE001 - OCCT's failure is said
            t["occt"] = {"error": str(e)}
    return t


def say(t):
    print(f"{t['path']}: units {t['units']}, uncertainty {t['uncertainty'] or '-'}"
          + (", a real written NaN or Inf" if t["nan"] else ""))
    for b in t["breaks"]:
        print(f"  loop #{b['loop']} of face #{b['face']}: edge #{b['edge']} ends at vertex "
              f"#{b['ends']} {b['at']}, the next (#{b['next']}) starts at #{b['starts']} "
              f"{b['next_at']}: {b['apart']:.3g} apart")
    for u in t["uses"]:
        print(f"  shell #{u['shell']}: edge #{u['edge']} used {u['along']} times along and "
              f"{u['against']} against (start {u['at']})")
    for p in t["pointlines"]:
        spans = ", ".join(f"#{s['edge']} between vertices {s['apart']:.3g} apart"
                          for s in p["edges"] if s["apart"] is not None)
        print(f"  LINE #{p['line']}: its VECTOR #{p['vector']} has magnitude 0, a point; "
              f"edges on it: {spans or 'none'}")
    o = t.get("occt")
    if o is None:
        return
    if "error" in o:
        print(f"  OCCT: {o['error']}")
        return
    u = [float(x) for x in t["uncertainty"].split(",")] if t["uncertainty"] else []
    print(f"  OCCT (healing off): valid {o['valid']}; size {o['size']:.4g}; tol_edge "
          f"{o['tol_edge']:.3g} (#{o['tol_edge_entity']}), {o['loose']:.2g} of the size"
          + (f", {o['tol_edge'] / min(u):.3g} times the uncertainty" if u and min(u) > 0 else "")
          + f"; tol_vertex {o['tol_vertex']:.3g} (#{o['tol_vertex_entity']})")
    for w in o.get("off", []):
        print(f"  edge #{w['edge']} stands {w['faces']:.3g} off its faces' surfaces, its vertices "
              f"{w['vertices']:.3g} off its curve (OCCT's projections)")
    for s in o["statuses"][:40]:
        ctx = "; ".join(f"in {c['in']} #{c['entity']}: {','.join(c['status'])}" for c in s["context"][:4])
        print(f"  {s['shape']} #{s['entity']}: {','.join(s['status']) or '-'}" + (f"; {ctx}" if ctx else ""))
    if len(o["statuses"]) > 40:
        print(f"  … {len(o['statuses']) - 40} more")


def main():
    argv = sys.argv[1:]
    text_only = "--text" in argv
    as_json = "--json" in argv
    for p in [a for a in argv if not a.startswith("--")]:
        t = report(p, text_only)
        if as_json:
            print(json.dumps(t), flush=True)
        else:
            say(t)
            sys.stdout.flush()


if __name__ == "__main__":
    main()
